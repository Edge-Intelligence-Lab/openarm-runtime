"""Own only local processes started by this platform; serialize shared hardware."""
from dataclasses import dataclass
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time


SHUTDOWN_TIMEOUT = 12.0
MOTION_SHUTDOWN_TIMEOUT = 40.0
KILL_TIMEOUT = 2.0


def tcp_reachable(port, timeout=0.15):
    """Check a loopback listener, not the identity or health of its hardware."""
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=timeout):
            return True
    except OSError:
        return False


@dataclass
class _Service:
    page: object
    process: object = None
    external: bool = False
    standby: bool = False
    generation: int = 0
    error: str | None = None

    @property
    def group(self):
        return getattr(self.page, "exclusive_group", None)

    @property
    def label(self):
        return getattr(self.page, "label", self.page.id)


class ServiceManager:
    """Launch missing services and explicitly switch mutually exclusive groups.

    Commands are prevalidated local argument arrays. Construction is passive.
    Existing listeners never become owned. There is no automatic restart.
    """

    def __init__(self, pages, log_dir):
        self._services = {}
        for page in pages:
            if not re.fullmatch(r"[a-z][a-z0-9_-]*", page.id):
                raise ValueError("Service IDs must be simple lowercase names")
            if page.id in self._services:
                raise ValueError(f"Duplicate service ID: {page.id}")
            self._services[page.id] = _Service(page)
        self.log_dir = Path(log_dir)
        self._started = False
        self._closed = False
        self._closing = False
        self._blocked_groups = {}
        self._motion_owner = None
        self._lock = threading.RLock()

    def _launch(self, service):
        page = service.page
        service.standby = False
        service.error = None
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            # The child inherits the log descriptor; parent handles close even
            # when process creation fails.
            with (self.log_dir / f"{page.id}.log").open("ab", buffering=0) as log:
                service.process = subprocess.Popen(
                    list(page.command), cwd=page.cwd, stdin=subprocess.DEVNULL,
                    stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=(os.name == "posix"),
                )
            service.external = False
            service.generation += 1
        except (OSError, ValueError) as exc:
            service.error = f"{type(exc).__name__}: {exc}"

    def start(self):
        with self._lock:
            if self._started or self._closing or self._closed:
                raise RuntimeError("Service manager has already started or closed")
            self._started = True
            reachable = {name: tcp_reachable(s.page.port) for name, s in self._services.items()}
            occupied_groups = {s.group for name, s in self._services.items()
                               if s.group and reachable[name]}
            selected_groups = set()
            for name, service in self._services.items():
                if reachable[name]:
                    service.external = True
                elif getattr(service.page,'lazy',False):
                    service.standby = True
                elif service.group in occupied_groups or service.group in selected_groups:
                    service.standby = True
                elif service.page.command:
                    # A failed first candidate does not silently launch a peer.
                    if service.group:
                        selected_groups.add(service.group)
                    self._launch(service)
        return self

    def status(self):
        with self._lock:
            statuses = {}
            for name, service in self._services.items():
                reachable = tcp_reachable(service.page.port)
                process = service.process
                code = process.poll() if process is not None else None
                if process is not None and code is not None and not self._closing and service.error is None:
                    service.error = f"Service exited with status {code}; see {name}.log"
                if service.error:
                    state = "error"
                elif process is not None:
                    state = "running" if reachable else "starting"
                elif reachable:
                    state = "external"
                elif service.standby and not self._closed:
                    state = "standby"
                elif service.external or self._closed:
                    state = "stopped"
                elif not service.page.command:
                    state = "unconfigured"
                else:
                    state = "stopped"
                statuses[name] = dict(reachable=reachable, state=state,
                                      managed=process is not None, error=service.error,
                                      generation=service.generation)
            return statuses

    def _activate_independent(self, target):
        """Retry stopped capture services only after cleaning up owned children."""
        if target.process is not None:
            if target.process.poll() is None:
                return
            # A failed parent may have left capture subprocesses behind. Reap
            # the whole owned session before opening the same devices again.
            failures, _ = self._stop_services([target])
            if failures:
                target.error = f"{target.label}清理失败，暂时无法重试：" + "; ".join(failures)
                raise ValueError(target.error)
        if tcp_reachable(target.page.port):
            target.external = True
            target.standby = False
            target.error = None
        elif target.page.command:
            self._launch(target)
        else:
            raise ValueError(f"{target.label}尚未配置启动命令。")

    def activate(self, name):
        """Select a module; retry capture services, coordinate motion peers."""
        with self._lock:
            if not self._started or self._closing or self._closed:
                raise ValueError("Platform services are not running")
            if name not in self._services:
                raise ValueError(f"Unknown service: {name}")
            target = self._services[name]
            if target.group == 'motion' and self._motion_owner is not None:
                raise ValueError('请先停止模型执行并等待清理完成，再切换控制服务')
            if not target.group:
                self._activate_independent(target)
                return self.status()
            if target.group in self._blocked_groups:
                raise ValueError(self._blocked_groups[target.group])
            peers = [s for s in self._services.values() if s is not target and s.group == target.group]
            for peer in peers:
                if peer.process is None and tcp_reachable(peer.page.port):
                    peer.external = True
                    raise ValueError(f"请先停止独立运行的{peer.label}服务（端口 {peer.page.port}），再切换到{target.label}；平台不会停止外部服务。")
            if target.process is not None:
                if target.process.poll() is None:
                    return self.status()
                message = f"{target.label}服务异常退出，无法确认硬件已安全停止；请检查后重启平台。"
                target.error = message
                self._blocked_groups[target.group] = message
                raise ValueError(message)
            target_reachable = tcp_reachable(target.page.port)
            if not target_reachable and not target.page.command:
                raise ValueError(f"{target.label}尚未配置启动命令。")
            for peer in peers:
                if peer.process is None:
                    continue
                if peer.process.poll() is not None:
                    message = f"{peer.label}服务异常退出，无法确认硬件已安全停止；请检查后重启平台。"
                    peer.error = message
                    self._blocked_groups[target.group] = message
                    raise ValueError(message)
                failures, forced = self._stop_services([peer])
                if failures or forced:
                    message = f"{peer.label}未能正常退出，已阻止切换；请检查硬件和服务后重启平台。"
                    if failures:
                        message += " " + "; ".join(failures)
                    peer.error = message
                    self._blocked_groups[target.group] = message
                    raise ValueError(message)
                peer.standby = True
            if target_reachable:
                target.external = True
                target.standby = False
            else:
                self._launch(target)
            return self.status()

    def deactivate(self,name):
        """Explicitly close one owned task without starting a replacement."""
        with self._lock:
            if name not in self._services or self._closing or self._closed:
                raise ValueError('服务不存在或平台正在关闭')
            target=self._services[name]
            if target.process is None and tcp_reachable(target.page.port):
                raise ValueError('外部启动的服务不能由本平台停止')
            if target.process is not None:
                failures,forced=self._stop_services([target])
                if failures or forced:
                    message='服务未确认正常退出，请现场检查后再切换控制'
                    if target.group:self._blocked_groups[target.group]=message
                    raise ValueError(message)
            target.standby=True
            return self.status()

    def claim_motion(self, owner):
        """Explicit policy start only; use the existing normal motion handover.

        A loaded model does not own this lease. External controllers are never
        stopped. Failed/forced cleanup blocks subsequent motion as before.
        """
        with self._lock:
            if not self._started or self._closing or self._closed:
                raise ValueError('平台服务尚未就绪或正在关闭')
            if self._motion_owner is not None or 'motion' in self._blocked_groups:
                raise ValueError(self._blocked_groups.get('motion', '已有模型执行占用控制'))
            peers = [s for s in self._services.values() if s.group == 'motion']
            for peer in peers:
                if peer.process is None and tcp_reachable(peer.page.port):
                    raise ValueError(f'请先由操作员停止外部控制服务：{peer.label}')
                if peer.process is not None and peer.process.poll() is not None:
                    self._blocked_groups['motion'] = '控制服务异常退出，请现场检查后重启平台'
                    raise ValueError(self._blocked_groups['motion'])
            for peer in peers:
                if peer.process is not None:
                    failures, forced = self._stop_services([peer])
                    if failures or forced:
                        self._blocked_groups['motion'] = '控制服务未正常退出，已阻止模型执行'
                        raise ValueError(self._blocked_groups['motion'])
                    peer.standby = True
            self._motion_owner = owner

    def release_motion(self, owner, *, clean):
        with self._lock:
            if self._motion_owner != owner:
                raise ValueError('模型执行不拥有当前控制租约')
            self._motion_owner = None
            if not clean:
                self._blocked_groups['motion'] = '模型会话未确认正常清理，请现场检查后重启平台'

    @staticmethod
    def _signal(process, sig):
        try:
            if os.name == "posix":
                os.killpg(process.pid, sig)
            elif process.poll() is None:
                process.kill() if sig == signal.SIGKILL else process.terminate()
        except ProcessLookupError:
            pass

    @staticmethod
    def _alive(process):
        if os.name != "posix":
            return process.poll() is None
        try:
            os.killpg(process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def _stop_services(self, owned):
        """Return failures and whether any non-motion group needed SIGKILL."""
        failures = []
        forced = False
        motion = [s for s in owned if s.group == "motion"]
        ordinary = [s for s in owned if s.group != "motion"]
        for services, timeout in ((motion, MOTION_SHUTDOWN_TIMEOUT), (ordinary, SHUTDOWN_TIMEOUT)):
            if not services:
                continue
            remaining = list(services)
            for service in services:
                try:
                    if service.group == "motion":
                        # Let the controlling parent complete its verified disable
                        # path. Never forcibly kill a process owning motor control.
                        if service.process.poll() is None:
                            service.process.terminate()
                    else:
                        self._signal(service.process, signal.SIGTERM)
                except OSError as exc:
                    failures.append(str(exc))
            deadline = time.monotonic() + timeout
            while remaining and time.monotonic() < deadline:
                for service in remaining[:]:
                    service.process.poll()
                    try:
                        alive = self._alive(service.process)
                    except OSError as exc:
                        failures.append(str(exc))
                        alive = True
                    if not alive:
                        remaining.remove(service)
                if remaining:
                    time.sleep(min(0.05, max(0, deadline - time.monotonic())))
            for service in remaining:
                if service.group == "motion":
                    service.error = f"{service.label}关闭超时；仍保留进程，未强制终止。请在原页面停止硬件后检查服务。"
                    failures.append(service.error)
                    self._blocked_groups[service.group] = service.error
                else:
                    forced = True
                    try:
                        self._signal(service.process, signal.SIGKILL)
                    except OSError as exc:
                        failures.append(str(exc))
            deadline = time.monotonic() + KILL_TIMEOUT
            for service in services:
                if service.group == "motion" and service in remaining:
                    continue
                try:
                    code = service.process.wait(timeout=max(0, deadline - time.monotonic()))
                    if service.group == "motion" and code != 0:
                        failures.append(f"{service.label}退出状态为 {code}，无法确认正常关闭。")
                    service.process = None
                except (subprocess.TimeoutExpired, OSError) as exc:
                    failures.append(str(exc))
        return failures, forced

    def close(self):
        with self._lock:
            if self._closed:
                return
            if self._motion_owner is not None:
                raise RuntimeError('模型执行尚未确认停止，不能关闭其依赖服务；未强制终止控制进程')
            self._closing = True
            owned = [s for s in self._services.values() if s.process is not None]
            failures, _ = self._stop_services(owned)
            self._closed = not any(s.process is not None for s in self._services.values())
            if failures:
                raise RuntimeError("Service cleanup failed: " + "; ".join(failures))

    __enter__ = start

    def __exit__(self, *_):
        self.close()
