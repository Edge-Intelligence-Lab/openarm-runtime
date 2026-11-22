'use strict';

const enter = document.getElementById('enter');
const stop = document.getElementById('stop');
const resetButton = document.getElementById('reset-scene');
const sceneButton = document.getElementById('choose-scene');
const sceneDialog = document.getElementById('scene-dialog');
const sceneFeedback = document.getElementById('scene-feedback');
const currentScene = document.getElementById('current-scene');
const sceneChoices = Array.from(sceneDialog.querySelectorAll('[data-scene]'));
const sceneGuides = document.querySelectorAll('[data-scene-guide]');
const startPosition = document.getElementById('start-position');
const status = document.getElementById('status');
const statusZh = document.getElementById('status-zh');
const statusEn = document.getElementById('status-en');
const canvas = document.getElementById('xr');
const headCamera = {name: 'head', preview: document.getElementById('head-camera'), interval: 65, accent: '#f5ba75'};
const cameras = [
    {name: 'overview', preview: document.getElementById('overview'), interval: 50, accent: '#81e0c9'},
    {...headCamera, interval: 85},
];
if (document.body.classList.contains('practice')) {
    cameras.splice(1, 0,
        {name: 'closeup', preview: document.getElementById('closeup-camera'), interval: 60, accent: '#aabaff'});
}
const primaryPanel = {x: -.21, width: .46, height: .56};
const secondaryPanel = {x: .46, width: .18, height: .30};
const states = {
    checking: ['正在检查 WebXR…', 'Checking WebXR…'],
    connecting: ['正在连接…', 'Connecting…'],
    aligning: ['保持手柄稳定，正在对齐…', 'Hold controllers steady to align…'],
    following: ['自动跟随中', 'Following controllers'],
    paired: ['双臂共抬中', 'Two-arm lift active'],
    reconnecting: ['连接中断，正在重连…', 'Reconnecting…', true],
    busy: ['仿真机械臂正在使用，请稍后进入', 'Simulation in use. Try again shortly.', true],
    exited: ['已退出 VR', 'Exited VR'],
    resetting: ['正在重置练习…', 'Resetting practice…'],
    resetBlocked: ['无法重置，请确认机械臂没有被其他页面使用', 'Reset unavailable. Check whether another tab is controlling the arms.', true],
    resetFailed: ['重置未确认，请检查连接后重试', 'Reset not confirmed. Check the connection and try again.', true],
    secure: ['请在 PICO 浏览器中打开可信的 HTTPS 地址', 'Open a trusted HTTPS address in PICO Browser.', true],
    unsupported: ['请使用支持沉浸式 WebXR 的 PICO 浏览器', 'Use PICO Browser with immersive WebXR support.', true],
    ready: ['可以进入 VR', 'Ready to enter VR'],
};
let activeStatus = states.checking;

function showStatus(next) {
    if (next === activeStatus) return;
    activeStatus = next;
    statusZh.textContent = next[0];
    statusEn.textContent = next[1];
    status.dataset.state = next[2] ? 'warning' : 'normal';
}

let session;
let connecting = false;
let pageActive = true;
const ALIGN_TIME_MS = 2000;
const countdownStatus = [
    null,
    ['1 · 双手保持自然下垂', '1 · Keep your hands at your sides'],
    ['2 · 双手保持自然下垂', '2 · Keep your hands at your sides'],
];
const REALIGN_TIME_MS = 250;
const ALIGN_RADIUS_M = .025;
const RESET_HOLD_MS = 1200;
const RESET_TIMEOUT_MS = 3500;
function updatePreview() {
    const showPreview = pageActive && !document.hidden && !session;
    for (const camera of cameras) {
        if (!showPreview) {
            camera.preview.removeAttribute('src');
        } else if (!camera.preview.hasAttribute('src')) {
            camera.preview.src = `/apps/vision/stream/${camera.name}.mjpg`;
        }
    }
    picoTelemetry.setActive(pageActive && (!document.hidden || Boolean(session)));
}

function operatorHeading({x, y, z, w}) {
    // WebXR's forward axis is -Z. Project the viewer's forward vector onto the floor.
    const forwardX = -2 * (x * z + w * y);
    const forwardZ = 2 * (x * x + y * y) - 1;
    if (Math.hypot(forwardX, forwardZ) < 0.2) return null;
    const yaw = Math.atan2(-forwardX, -forwardZ);
    return {
        sin: Math.sin(yaw), cos: Math.cos(yaw),
        inverseHalfSin: Math.sin(-yaw / 2), halfCos: Math.cos(yaw / 2),
    };
}

function jawFromThumbstick(gamepad) {
    const y = gamepad?.mapping === 'xr-standard' ? gamepad.axes?.[3] : undefined;
    const threshold = .5;
    if (y <= -threshold) return 'open';
    if (y >= threshold) return 'close';
    return null;
}

function operatorHandPose(grip, reference, heading) {
    const dx = grip.position.x - reference.x;
    const dz = grip.position.z - reference.z;
    const {x, y, z, w} = grip.orientation;
    const {sin, cos, inverseHalfSin, halfCos} = heading;
    return {
        position: {
            x: cos * dx - sin * dz,
            y: grip.position.y - reference.y,
            z: sin * dx + cos * dz,
        },
        // Left-multiply by the inverse heading so the wrist uses the same axes as position.
        orientation: {
            x: halfCos * x + inverseHalfSin * z,
            y: halfCos * y + inverseHalfSin * w,
            z: halfCos * z - inverseHalfSin * x,
            w: halfCos * w - inverseHalfSin * y,
        },
    };
}

function controlEndpoint() {
    const endpoint = new URL('/simulation/pico/input', location.href);
    endpoint.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    return endpoint;
}

function commandFromPage(command, responseKey, onSuccess, onFailure) {
    if (session || connecting || resetButton.disabled) return;
    const enterWasDisabled = enter.disabled;
    resetButton.disabled = enter.disabled = sceneButton.disabled = true;
    for (const choice of sceneChoices) choice.disabled = true;
    const connection = new WebSocket(controlEndpoint());
    let settled = false;
    const timer = setTimeout(() => finish(states.resetFailed), RESET_TIMEOUT_MS);

    function finish(failure, value) {
        if (settled) return;
        settled = true;
        clearTimeout(timer);
        connection.close();
        enter.disabled = enterWasDisabled;
        resetButton.disabled = sceneButton.disabled = false;
        for (const choice of sceneChoices) choice.disabled = false;
        if (failure) onFailure(failure);
        else onSuccess(value);
    }

    connection.addEventListener('open', () => connection.send(JSON.stringify(command)));
    connection.addEventListener('close', () => finish(states.resetFailed));
    connection.addEventListener('message', event => {
        const value = JSON.parse(event.data)[responseKey];
        finish(value ? null : states.resetBlocked, value);
    });
}

function showScene(scene) {
    const selected = sceneChoices.find(choice => choice.dataset.scene === scene);
    if (!selected) return;
    for (const choice of sceneChoices) choice.setAttribute('aria-pressed', String(choice === selected));
    for (const guide of sceneGuides) guide.hidden = guide.dataset.sceneGuide !== scene;
    currentScene.textContent = Array.from(selected.querySelectorAll('span'), item => item.textContent).join(' / ');
}

function monitor(gl) {
    function shader(kind, source) {
        const result = gl.createShader(kind);
        gl.shaderSource(result, source);
        gl.compileShader(result);
        if (!gl.getShaderParameter(result, gl.COMPILE_STATUS)) {
            throw new Error(gl.getShaderInfoLog(result));
        }
        return result;
    }

    const vertex = shader(gl.VERTEX_SHADER, `attribute vec2 point;
        uniform vec2 size; uniform vec2 center; varying vec2 uv;
        void main() { uv = (point + 1.0)*0.5; gl_Position=vec4(point*size + center, 0., 1.); }`);
    const fragment = shader(gl.FRAGMENT_SHADER, `precision mediump float;
        uniform sampler2D camera; varying vec2 uv;
        void main() { gl_FragColor = texture2D(camera, uv); }`);
    const program = gl.createProgram();
    gl.attachShader(program, vertex);
    gl.attachShader(program, fragment);
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
        throw new Error(gl.getProgramInfoLog(program));
    }
    gl.useProgram(program);
    gl.deleteShader(vertex);
    gl.deleteShader(fragment);
    const buffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
    const point = gl.getAttribLocation(program, 'point');
    gl.enableVertexAttribArray(point);
    gl.vertexAttribPointer(point, 2, gl.FLOAT, false, 0, 0);
    const size = gl.getUniformLocation(program, 'size');
    const center = gl.getUniformLocation(program, 'center');
    function newTexture() {
        const texture = gl.createTexture();
        gl.bindTexture(gl.TEXTURE_2D, texture);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        return texture;
    }
    function uploadCanvas(texture, artwork) {
        gl.bindTexture(gl.TEXTURE_2D, texture);
        gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, artwork);
        gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
    }
    function artwork(width, height) {
        const surface = document.createElement('canvas');
        surface.width = width;
        surface.height = height;
        return [surface, surface.getContext('2d')];
    }
    const panelWidth = 704;
    const panelHeight = 578;
    const picture = {x: 16, y: 67, width: 672, height: 504};
    const screens = cameras.map(camera => {
        const [surface, ctx] = artwork(panelWidth, panelHeight);
        ctx.fillStyle = '#132227';
        ctx.fillRect(0, 0, panelWidth, panelHeight);
        ctx.strokeStyle = camera.accent;
        ctx.lineWidth = 6;
        ctx.strokeRect(3, 3, panelWidth - 6, panelHeight - 6);
        ctx.fillStyle = '#090f12';
        ctx.fillRect(picture.x, picture.y, picture.width, picture.height);
        ctx.fillStyle = camera.accent;
        ctx.font = 'bold 38px system-ui, sans-serif';
        const [labelZh, labelEn] = camera.labels ?? ['zh', 'en'].map(
            language => document.getElementById(`${camera.name}-${language}`).textContent);
        ctx.fillText(labelZh, 21, 49, 252);
        ctx.fillStyle = '#edf5f4';
        ctx.font = '34px system-ui, sans-serif';
        ctx.fillText(labelEn, 263, 49, 420);
        const panel = newTexture();
        uploadCanvas(panel, surface);
        return {...camera, panel, texture: newTexture(), aspect: 0, updated: -Infinity};
    });
    const [statusSurface, statusContext] = artwork(704, 86);
    const statusTexture = newTexture();
    const [telemetrySurface, telemetryContext] = artwork(1120, 214);
    const telemetryTexture = newTexture();
    let paintedTelemetry;
    let paintedStatus;

    function drawTexture(texture, x, y, width, height) {
        gl.bindTexture(gl.TEXTURE_2D, texture);
        gl.uniform2f(center, x, y);
        gl.uniform2f(size, width, height);
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    }

    function refreshStatus(status) {
        if (paintedStatus === status) return;
        paintedStatus = status;
        const ctx = statusContext;
        ctx.fillStyle = '#132227';
        ctx.fillRect(0, 0, statusSurface.width, statusSurface.height);
        ctx.fillStyle = status[2] ? '#f5ba75' : '#81e0c9';
        ctx.fillRect(0, 0, 7, statusSurface.height);
        ctx.font = 'bold 31px system-ui, sans-serif';
        ctx.fillText(status[0], 20, 35, 664);
        ctx.fillStyle = '#c5d6d7';
        ctx.font = '25px system-ui, sans-serif';
        ctx.fillText(status[1], 20, 70, 664);
        uploadCanvas(statusTexture, statusSurface);
    }

    function refreshTelemetry() {
        const snapshot = picoTelemetry.current();
        if (snapshot === paintedTelemetry) return;
        paintedTelemetry = snapshot;
        const ctx = telemetryContext;
        ctx.fillStyle = '#132227';
        ctx.fillRect(0, 0, telemetrySurface.width, telemetrySurface.height);
        ctx.strokeStyle = '#30474e';
        ctx.lineWidth = 4;
        ctx.strokeRect(2, 2, telemetrySurface.width - 4, telemetrySurface.height - 4);
        ctx.fillStyle = '#81e0c9';
        ctx.font = 'bold 28px system-ui, sans-serif';
        ctx.fillText('实时仿真数据 / Live simulation telemetry', 22, 38);
        ctx.fillStyle = '#a8bfc2';
        ctx.font = '23px system-ui, sans-serif';
        const labels = ['J1°', 'J2°', 'J3°', 'J4°', 'J5°', 'J6°', 'J7°', '开合° / Jaw', '夹力 N / Grip'];
        labels.forEach((label, index) => ctx.fillText(label, 116 + index * 101, 86));
        ctx.font = '27px system-ui, sans-serif';
        for (const [index, side] of ['left', 'right'].entries()) {
            const y = 134 + index * 47;
            const arm = snapshot?.arms[side];
            const values = arm ? [...arm.joint_deg, arm.jaw_opening_deg, arm.grip_force_n] : [];
            ctx.fillStyle = index === 0 ? '#81e0c9' : '#f5ba75';
            ctx.fillText(index === 0 ? '左 / L' : '右 / R', 22, y);
            ctx.fillStyle = '#edf5f4';
            labels.forEach((_, column) => {
                const value = values[column];
                ctx.fillText(Number.isFinite(value) ? value.toFixed(1) : '—', 116 + column * 101, y);
            });
        }
        uploadCanvas(telemetryTexture, telemetrySurface);
    }

    return {
        upload(index, bitmap) {
            const screen = screens[index];
            gl.bindTexture(gl.TEXTURE_2D, screen.texture);
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, bitmap);
            screen.aspect = bitmap.width / bitmap.height;
            screen.updated = performance.now();
        },
        cameraFresh(index, now) {
            return now - screens[index].updated < 500;
        },
        draw(layer, pose, now, primaryCamera, countdown) {
            gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
            gl.clearColor(0.035, 0.05, 0.045, 1);
            gl.clear(gl.COLOR_BUFFER_BIT);
            if (!pose) return;
            refreshStatus(countdownStatus[countdown] ?? activeStatus);
            refreshTelemetry();
            for (const view of pose.views) {
                const viewport = layer.getViewport(view);
                gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
                const viewportRatio = viewport.height / viewport.width;
                let secondaryIndex = 0;
                for (const [index, screen] of screens.entries()) {
                    const primary = index === primaryCamera;
                    const panel = primary ? primaryPanel : secondaryPanel;
                    const ratio = panelWidth / panelHeight * viewportRatio;
                    const width = Math.min(panel.width, panel.height * ratio);
                    const height = width / ratio;
                    const centerY = primary || screens.length === 2 ? .04 : .37 - .66 * secondaryIndex++;
                    drawTexture(screen.panel, panel.x, centerY, width, height);
                    if (now - screen.updated >= 500) continue;
                    const videoRatio = screen.aspect * viewportRatio;
                    const videoWidth = Math.min(width * picture.width / panelWidth,
                        height * picture.height / panelHeight * videoRatio);
                    const videoY = centerY + height * (1 - 2 * (picture.y + picture.height / 2) / panelHeight);
                    drawTexture(screen.texture, panel.x, videoY, videoWidth, videoWidth / videoRatio);
                }
                const telemetryRatio = telemetrySurface.width / telemetrySurface.height * viewportRatio;
                const telemetryWidth = Math.min(.59, .14 * telemetryRatio);
                const telemetryHeight = telemetryWidth / telemetryRatio;
                drawTexture(telemetryTexture, -.02, -.55 - telemetryHeight, telemetryWidth, telemetryHeight);
                const statusWidth = Math.min(countdown ? .46 : .35,
                    (countdown ? .09 : .055) * statusSurface.width / statusSurface.height * viewportRatio);
                const statusY = countdown ? .71 : -.91;
                drawTexture(statusTexture, -.21, statusY, statusWidth,
                    statusWidth / (statusSurface.width / statusSurface.height * viewportRatio));
            }
        },
        close() {
            for (const screen of screens) {
                gl.deleteTexture(screen.texture);
                gl.deleteTexture(screen.panel);
            }
            gl.deleteTexture(statusTexture);
            gl.deleteTexture(telemetryTexture);
            gl.deleteBuffer(buffer);
            gl.deleteProgram(program);
        },
    };
}

async function start() {
    if (connecting || session) return;
    connecting = true;
    enter.disabled = true;
    resetButton.disabled = true;
    sceneButton.disabled = true;
    showStatus(states.connecting);
    let xr;
    let socket;
    let display;
    let active = true;
    let reconnectTimer;
    let resetTimer;
    let resetting = false;
    let startRetryUntil;
    const cameraTimers = [];
    const cameraSequences = [];
    let inFlight = false;
    let lastSent = -Infinity;
    let started = false;
    let controlReady = false;
    let heading = null;
    let primaryCamera = 0;
    const hands = Object.fromEntries(['left', 'right'].map(side =>
        [side, {anchor: null, stableSince: Infinity, aligned: false, armed: false,
            pendingRelease: true, gripperClosed: false}]));
    const camera = new AbortController();
    const squeezed = new Set();

    function cleanup() {
        if (!active) return;
        active = false;
        camera.abort();
        for (const timer of cameraTimers) clearTimeout(timer);
        clearTimeout(reconnectTimer);
        clearTimeout(resetTimer);
        socket?.close();
        display?.close();
        session = undefined;
        stop.disabled = true;
        enter.disabled = false;
        resetButton.disabled = false;
        sceneButton.disabled = false;
        updatePreview();
    }

    try {
        xr = await navigator.xr.requestSession('immersive-vr');
        session = xr;
        updatePreview();
        xr.addEventListener('end', () => {
            cleanup();
            showStatus(states.exited);
        }, {once: true});
        stop.disabled = false;
        const gl = canvas.getContext('webgl', {alpha: false, xrCompatible: true});
        if (!gl) throw new Error('浏览器无法初始化 WebGL');
        await gl.makeXRCompatible();
        if (!active) return;
        const layer = new XRWebGLLayer(xr, gl);
        xr.updateRenderState({baseLayer: layer});
        const space = await xr.requestReferenceSpace('local');
        if (!active) return;
        display = monitor(gl);

        const endpoint = controlEndpoint();
        xr.addEventListener('squeezestart', event => squeezed.add(event.inputSource));
        xr.addEventListener('squeezeend', event => squeezed.delete(event.inputSource));
        function disarm(hand) {
            hand.anchor = null;
            hand.stableSince = Infinity;
            hand.armed = false;
            hand.pendingRelease = true;
        }
        const disarmAll = () => Object.values(hands).forEach(disarm);
        function disconnectControl() {
            const connection = socket;
            socket = undefined;
            clearTimeout(reconnectTimer);
            clearTimeout(resetTimer);
            controlReady = inFlight = resetting = false;
            disarmAll();
            connection?.close();
        }
        function scheduleReconnect(delay) {
            clearTimeout(reconnectTimer);
            if (active && xr.visibilityState !== 'hidden') {
                reconnectTimer = setTimeout(connectControl, delay);
            }
        }
        function connectControl() {
            if (!active || socket || xr.visibilityState === 'hidden') return;
            const connection = new WebSocket(endpoint);
            socket = connection;
            connection.addEventListener('open', () => {
                if (!active || socket !== connection) return;
                lastSent = -Infinity;
                if (!started) startRetryUntil ??= performance.now() + 2400;
                controlReady = false;
                inFlight = true;
                // A new transport must acknowledge a start or a neutral packet before following.
                connection.send(JSON.stringify(started ? {left: null, right: null} : {start: true}));
                showStatus(states.aligning);
            });
            connection.addEventListener('close', () => {
                if (!active || socket !== connection) return;
                disconnectControl();
                showStatus(states.reconnecting);
                scheduleReconnect(800);
            });
            connection.addEventListener('message', event => {
                if (!active || socket !== connection) return;
                inFlight = false;
                const reply = JSON.parse(event.data);
                if (Object.hasOwn(reply, 'reset')) {
                    clearTimeout(resetTimer);
                    if (reply.reset) {
                        cleanup();
                        xr.end().catch(() => {}).finally(() => window.location.reload());
                    } else {
                        resetting = false;
                        showStatus(states.resetBlocked);
                    }
                    return;
                }
                if (resetting) return;
                if (Object.hasOwn(reply, 'started')) {
                    if (!reply.started) {
                        if (performance.now() < startRetryUntil) {
                            disconnectControl();
                            scheduleReconnect(300);
                            showStatus(states.reconnecting);
                            return;
                        }
                        xr.end().catch(cleanup).finally(() => {
                            showStatus(states.busy);
                        });
                        return;
                    }
                    started = true;
                }
                controlReady = started;
                for (const side of reply.blocked ?? []) disarm(hands[side]);
                showStatus(reply.paired ? states.paired : reply.following ? states.following : states.aligning);
            });
        }
        function resetInVR() {
            if (resetting || !controlReady || socket?.readyState !== WebSocket.OPEN) return;
            resetting = true;
            disarmAll();
            showStatus(states.resetting);
            const connection = socket;
            connection.send(JSON.stringify({reset: true}));
            resetTimer = setTimeout(() => {
                if (resetting && socket === connection) connection.close();
            }, RESET_TIMEOUT_MS);
        }
        xr.addEventListener('inputsourceschange', event => {
            for (const source of event.removed) {
                squeezed.delete(source);
                const hand = hands[source.handedness];
                if (hand) {
                    disarm(hand);
                    hand.gripStarted = undefined;
                }
            }
        });
        xr.addEventListener('visibilitychange', () => {
            disarmAll();
            for (const hand of Object.values(hands)) hand.gripStarted = undefined;
            if (xr.visibilityState === 'hidden') {
                startRetryUntil = undefined;
                disconnectControl();
            } else {
                connectControl();
            }
        });
        space.addEventListener('reset', () => {
            heading = null;
            disarmAll();
        });

        async function updateCamera(index) {
            const feed = cameras[index];
            const requested = performance.now();
            try {
                const path = `/apps/vision/frame/${feed.name}.jpg`;
                const response = await fetch(path,
                    {cache: 'no-store', signal: camera.signal});
                if (!response.ok) return;
                const sequence = response.headers.get('X-Frame-Sequence');
                const jpeg = await response.blob();
                if (sequence !== null && sequence === cameraSequences[index]) return;
                const bitmap = await createImageBitmap(jpeg, {imageOrientation: 'flipY'});
                try {
                    if (active) {
                        display.upload(index, bitmap);
                        cameraSequences[index] = sequence;
                    }
                } finally {
                    bitmap.close();
                }
            } catch {
                // An interrupted feed can recover independently on its next fetch.
            } finally {
                if (active) {
                    const interval = index === primaryCamera ? feed.interval : 160;
                    const delay = Math.max(0, interval - (performance.now() - requested));
                    cameraTimers[index] = setTimeout(() => updateCamera(index), delay);
                }
            }
        }

        function frame(now, xrFrame) {
            if (!active) return;
            xr.requestAnimationFrame(frame);
            const visible = xr.visibilityState === 'visible';
            const viewer = visible ? xrFrame.getViewerPose(space) : null;
            const reference = viewer && !viewer.emulatedPosition ? viewer.transform : null;
            const freshImage = cameras[primaryCamera].name === 'closeup'
                ? display.cameraFresh(primaryCamera, now)
                : cameras.some((_, index) => display.cameraFresh(index, now));
            const canTrack = controlReady && reference && freshImage;
            if (canTrack && !heading) heading = operatorHeading(reference.orientation);
            const poses = {};
            let alignmentRemaining = 0;
            let switchView = false;
            for (const [side, hand] of Object.entries(hands)) {
                const input = Array.from(xr.inputSources).find(source => source.handedness === side && source.gripSpace);
                const buttons = input?.gamepad?.mapping === 'xr-standard' ? input.gamepad.buttons : [];
                const gripButton = buttons[1];
                const held = gripButton?.pressed || squeezed.has(input);
                const pressed = Boolean(viewer && input && held);
                hand.jawDirection = jawFromThumbstick(input?.gamepad);
                if (viewer && input) {
                    if (pressed && hand.gripStarted === undefined) {
                        hand.gripStarted = now;
                        hand.longGrip = false;
                    }
                    if (hand.gripStarted !== undefined) {
                        if (!hand.longGrip && now - hand.gripStarted >= RESET_HOLD_MS) {
                            hand.longGrip = true;
                            resetInVR();
                        }
                        if (!pressed) {
                            if (!hand.longGrip) switchView = true;
                            hand.gripStarted = undefined;
                        }
                    }
                }
                const grip = viewer && input ? xrFrame.getPose(input.gripSpace, space) : null;
                const pose = canTrack && heading && grip && !grip.emulatedPosition
                    ? operatorHandPose(grip.transform, reference.position, heading) : null;
                if (!pose) disarm(hand);
                else if (!hand.armed) {
                    const {x, y, z} = pose.position;
                    const anchor = hand.anchor;
                    if (!anchor || Math.hypot(x - anchor.x, y - anchor.y, z - anchor.z) > ALIGN_RADIUS_M) {
                        hand.anchor = {x, y, z};
                        hand.stableSince = now;
                    } else if (hand.aligned && now - hand.stableSince >= REALIGN_TIME_MS) {
                        hand.armed = true;
                    }
                }
                if (pose && !hand.aligned) {
                    alignmentRemaining = Math.max(alignmentRemaining, ALIGN_TIME_MS - (now - hand.stableSince));
                }
                poses[side] = pose ? [
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                ] : null;
            }
            for (const [side, hand] of Object.entries(hands)) {
                if (poses[side] && !hand.aligned && alignmentRemaining === 0) {
                    hand.aligned = hand.armed = true;
                }
                if (!hand.armed || hand.pendingRelease) poses[side] = null;
                if (poses[side]) {
                    if (hand.jawDirection) hand.gripperClosed = hand.jawDirection === 'close';
                    poses[side].push(hand.gripperClosed);
                }
            }
            if (switchView && !resetting) {
                primaryCamera = (primaryCamera + 1) % cameras.length;
            }
            display.draw(layer, viewer, now, primaryCamera,
                resetting ? 0 : Math.ceil(alignmentRemaining / 1000));
            if (resetting || !controlReady || socket?.readyState !== WebSocket.OPEN || now - lastSent < 33) return;
            if (inFlight) return;
            // A release on a frame skipped by the transport must still reach the server.
            for (const hand of Object.values(hands)) hand.pendingRelease = false;
            socket.send(JSON.stringify(poses));
            inFlight = true;
            lastSent = now;
        }

        connectControl();
        cameras.forEach((_, index) => updateCamera(index));
        xr.requestAnimationFrame(frame);
    } catch (error) {
        cleanup();
        if (xr) await xr.end().catch(() => {});
        showStatus(['无法进入 VR', `Unable to enter VR. ${error.message}`, true]);
    } finally {
        connecting = false;
    }
}

async function checkSupport() {
    if (!window.isSecureContext) {
        showStatus(states.secure);
        return;
    }
    if (!await navigator.xr?.isSessionSupported('immersive-vr')) {
        showStatus(states.unsupported);
        return;
    }
    enter.disabled = false;
    showStatus(states.ready);
}

enter.addEventListener('click', () => startPosition.showModal());
document.getElementById('start-follow').addEventListener('click', () => {
    startPosition.close();
    start();
});
stop.addEventListener('click', () => session?.end());
resetButton.addEventListener('click', () => {
    showStatus(states.resetting);
    commandFromPage({reset: true}, 'reset', () => window.location.reload(), showStatus);
});
sceneButton.addEventListener('click', () => {
    sceneFeedback.textContent = '';
    sceneDialog.showModal();
});
for (const choice of sceneChoices) choice.addEventListener('click', () => {
    sceneFeedback.textContent = '正在加载场景… / Loading scene…';
    commandFromPage({scene: choice.dataset.scene}, 'scene', scene => {
        showScene(scene);
        sceneDialog.close();
    }, failure => {
        sceneFeedback.textContent = failure.slice(0, 2).join(' / ');
    });
});
document.addEventListener('pico-scene', event => showScene(event.detail));
document.addEventListener('visibilitychange', () => {
    updatePreview();
});
window.addEventListener('pageshow', () => {
    pageActive = true;
    updatePreview();
});
window.addEventListener('pagehide', () => {
    pageActive = false;
    updatePreview();
    session?.end();
});
updatePreview();
checkSupport().catch(error => showStatus(['WebXR 检查失败', `WebXR check failed. ${error.message}`, true]));
