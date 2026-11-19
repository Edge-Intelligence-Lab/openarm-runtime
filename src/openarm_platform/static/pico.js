'use strict';

const enter = document.getElementById('enter');
const stop = document.getElementById('stop');
const status = document.getElementById('status');
const preview = document.getElementById('preview');
const canvas = document.getElementById('xr');
const scene = '/apps/vision/frame/overview.jpg';
let session;
let connecting = false;
preview.src = '/apps/vision/stream/overview.mjpg';

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
        uniform vec2 size; varying vec2 uv;
        void main() { uv = (point + 1.0)*0.5; gl_Position=vec4(point*size, 0., 1.); }`);
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
    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    let aspect = 0;

    return {
        upload(bitmap) {
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, bitmap);
            aspect = bitmap.width / bitmap.height;
        },
        draw(layer, pose) {
            gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
            gl.clearColor(0.035, 0.05, 0.045, 1);
            gl.clear(gl.COLOR_BUFFER_BIT);
            if (!pose || !aspect) return;
            for (const view of pose.views) {
                const viewport = layer.getViewport(view);
                gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
                const ratio = aspect * viewport.height / viewport.width;
                gl.uniform2f(size, 0.68 * Math.min(1, ratio), 0.68 * Math.min(1, 1 / ratio));
                gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
            }
        },
        close() {
            gl.deleteTexture(texture);
            gl.deleteBuffer(buffer);
            gl.deleteProgram(program);
        },
    };
}

async function start() {
    if (connecting || session) return;
    connecting = true;
    enter.disabled = true;
    status.textContent = '正在连接…';
    let xr;
    let socket;
    let display;
    let active = true;
    let cameraTimer;
    let inFlight = false;
    let lastSent = -Infinity;
    let lastImage = -Infinity;
    let armed = false;
    let pendingRelease = true;
    const camera = new AbortController();
    const squeezed = new Set();

    function cleanup() {
        if (!active) return;
        active = false;
        camera.abort();
        clearTimeout(cameraTimer);
        socket?.close();
        display?.close();
        session = undefined;
        stop.disabled = true;
        enter.disabled = false;
        preview.src = '/apps/vision/stream/overview.mjpg';
    }

    try {
        xr = await navigator.xr.requestSession('immersive-vr');
        session = xr;
        xr.addEventListener('end', () => {
            cleanup();
            status.textContent = '已退出 VR';
        }, {once: true});
        stop.disabled = false;
        const gl = canvas.getContext('webgl', {alpha: false, xrCompatible: true});
        if (!gl) throw new Error('浏览器无法初始化 WebGL');
        await gl.makeXRCompatible();
        const layer = new XRWebGLLayer(xr, gl);
        xr.updateRenderState({baseLayer: layer});
        const space = await xr.requestReferenceSpace('local');
        if (!active) return;
        display = monitor(gl);
        preview.removeAttribute('src');

        const endpoint = new URL('/simulation/pico/input', location.href);
        endpoint.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
        socket = new WebSocket(endpoint);
        socket.addEventListener('close', () => {
            if (active) xr.end().catch(cleanup).finally(() => {
                status.textContent = '仿真连接已断开，请重新进入 VR';
            });
        });
        socket.addEventListener('message', event => {
            inFlight = false;
            status.textContent = JSON.parse(event.data).following ? '跟随中' : '已连接，等待握键与有效追踪';
        });
        xr.addEventListener('squeezestart', event => squeezed.add(event.inputSource));
        function disarm() {
            armed = false;
            pendingRelease = true;
        }

        xr.addEventListener('squeezeend', event => {
            squeezed.delete(event.inputSource);
            if (event.inputSource.handedness === 'right') pendingRelease = true;
        });
        xr.addEventListener('inputsourceschange', event => {
            for (const source of event.removed) {
                squeezed.delete(source);
                if (source.handedness === 'right') disarm();
            }
        });
        xr.addEventListener('visibilitychange', disarm);
        space.addEventListener('reset', disarm);

        async function updateCamera() {
            try {
                const response = await fetch(scene, {cache: 'no-store', signal: camera.signal});
                if (!response.ok) return;
                const bitmap = await createImageBitmap(await response.blob(), {imageOrientation: 'flipY'});
                try {
                    if (active) {
                        display.upload(bitmap);
                        lastImage = performance.now();
                    }
                } finally {
                    bitmap.close();
                }
            } catch {
                // If the camera remains unavailable, the input loop drops tracking.
            } finally {
                if (active) cameraTimer = setTimeout(updateCamera, 80);
            }
        }

        function frame(now, xrFrame) {
            if (!active) return;
            xr.requestAnimationFrame(frame);
            const visible = xr.visibilityState === 'visible';
            const viewer = visible ? xrFrame.getViewerPose(space) : null;
            const freshImage = now - lastImage < 500;
            display.draw(layer, freshImage ? viewer : null);
            const input = Array.from(xr.inputSources).find(source => source.handedness === 'right' && source.gripSpace);
            const button = input?.gamepad?.mapping === 'xr-standard' ? input.gamepad.buttons[1] : undefined;
            const pressed = Boolean(input && (button?.pressed || squeezed.has(input)));
            const grip = viewer && input ? xrFrame.getPose(input.gripSpace, space) : null;
            const tracked = grip && !grip.emulatedPosition && freshImage;
            if (!tracked) disarm();
            else if (!pressed) {
                armed = true;
                pendingRelease = true;
            }
            if (socket.readyState !== WebSocket.OPEN || inFlight || now - lastSent < 40) return;
            // A release on a frame skipped by the transport must still reach the server.
            const position = tracked && armed && pressed && !pendingRelease ? grip.transform.position : null;
            pendingRelease = false;
            socket.send(JSON.stringify(position ? [position.x, position.y, position.z] : null));
            inFlight = true;
            lastSent = now;
        }

        updateCamera();
        xr.requestAnimationFrame(frame);
    } catch (error) {
        cleanup();
        if (xr) await xr.end().catch(() => {});
        status.textContent = `无法进入 VR：${error.message}`;
    } finally {
        connecting = false;
    }
}

async function checkSupport() {
    if (!window.isSecureContext) {
        status.textContent = '请在 PICO 浏览器中用可信的 HTTPS 地址打开此页。';
        return;
    }
    if (!await navigator.xr?.isSessionSupported('immersive-vr')) {
        status.textContent = '请在支持沉浸式 WebXR 的 PICO 浏览器中打开此页。';
        return;
    }
    enter.disabled = false;
    status.textContent = '可以进入 VR';
}

enter.addEventListener('click', start);
stop.addEventListener('click', () => session?.end());
window.addEventListener('pagehide', () => session?.end());
checkSupport().catch(error => { status.textContent = `WebXR 检查失败：${error.message}`; });
