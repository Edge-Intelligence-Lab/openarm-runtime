#!/usr/bin/env python3
"""Repository entry: checkout code + project environment + external configuration."""
import argparse
import os
from pathlib import Path
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description='启动当前仓库的统一网页；不依赖仓库外启动脚本')
    parser.add_argument('--profile', type=Path,
                        default=Path.home()/'.config/openarm-runtime/platform.json',
                        help='外部个人平台配置（默认 ~/.config/openarm-runtime/platform.json）')
    parser.add_argument('--port', type=int, help='覆盖网页端口')
    parser.add_argument('--attach-only', action='store_true', help='只查看既有服务，不启动或停止设备')
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parent
    python = root / '.venv' / 'bin' / 'python'
    profile = args.profile.expanduser().resolve()
    if not python.is_file() or not profile.is_file():
        print(f'需要项目 .venv 和平台配置：{profile}\n参见 docs/platform.md', file=sys.stderr)
        return 1
    os.chdir(root)
    command = [str(python), '-m', 'openarm_runtime', 'platform', '--profile', str(profile)]
    if args.port is not None:
        command += ['--port', str(args.port)]
    if args.attach_only:
        command.append('--attach-only')
    # The launcher must not silently run an old installed wheel or a temporary
    # deployment's source just because its PYTHONPATH happens to be inherited.
    env = dict(os.environ)
    env['OPENARM_RUNTIME_SOURCE'] = str(root/'src')
    env['PYTHONPATH'] = str(root/'src') + (os.pathsep+env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
    os.execve(str(python), command, env)


if __name__ == '__main__':
    raise SystemExit(main())
