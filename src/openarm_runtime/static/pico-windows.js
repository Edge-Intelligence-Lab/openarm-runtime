const offsets = {head: 0, left: -.53, right: .53};
const defaultWidth = {head: .58, left: .36, right: .36};
const add = (a, b) => a.map((value, index) => value + b[index]);
const sub = (a, b) => a.map((value, index) => value - b[index]);
const mul = (a, scalar) => a.map(value => value * scalar);
const dot = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);
const axis = (matrix, offset) => Array.from(matrix.slice(offset, offset + 3));

export function pickWindow(panels, ray, right, up, normal) {
  let nearest = null;
  for (const panel of panels) {
    const denominator = dot(ray.direction, normal);
    if (Math.abs(denominator) < 1e-5) continue;
    const distance = dot(sub(panel.center, ray.origin), normal) / denominator;
    if (distance < .12 || (nearest && distance >= nearest.distance)) continue;
    const point = add(ray.origin, mul(ray.direction, distance));
    const local = sub(point, panel.center);
    if (Math.abs(dot(local, right)) <= panel.width / 2
        && Math.abs(dot(local, up)) <= panel.width / (2 * panel.aspect)) {
      nearest = {panel, point, distance};
    }
  }
  return nearest;
}

export function createCameraWindows(session, streams, initialScale, initiallyVisible, visibilityChanged) {
  const gl = document.createElement("canvas").getContext("webgl", {xrCompatible: true, alpha: true});
  if (!gl) throw new Error("无法初始化 WebGL");

  function shader(type, source) {
    const result = gl.createShader(type);
    gl.shaderSource(result, source);
    gl.compileShader(result);
    if (!gl.getShaderParameter(result, gl.COMPILE_STATUS)) throw new Error("无法初始化相机着色器");
    return result;
  }

  const program = gl.createProgram();
  const vertex = shader(gl.VERTEX_SHADER, `
    attribute vec2 corner;
    uniform mat4 projection, view;
    uniform vec3 center, right, up;
    varying vec2 uv;
    void main() {
      uv = vec2(corner.x + .5, .5 - corner.y);
      gl_Position = projection * view * vec4(center + corner.x * right + corner.y * up, 1.);
    }`);
  const fragment = shader(gl.FRAGMENT_SHADER, `
    precision mediump float;
    uniform sampler2D feed;
    uniform bool selected;
    varying vec2 uv;
    void main() {
      if (selected && (uv.x < .012 || uv.x > .988 || uv.y < .018 || uv.y > .982))
        gl_FragColor = vec4(.31, .93, .77, 1.);
      else gl_FragColor = texture2D(feed, uv);
    }`);
  gl.attachShader(program, vertex);
  gl.attachShader(program, fragment);
  gl.linkProgram(program);
  gl.deleteShader(vertex);
  gl.deleteShader(fragment);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error("无法初始化相机画面");
  gl.useProgram(program);
  const buffer = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-.5,-.5, .5,-.5, -.5,.5, .5,.5]), gl.STATIC_DRAW);
  const corner = gl.getAttribLocation(program, "corner");
  gl.enableVertexAttribArray(corner);
  gl.vertexAttribPointer(corner, 2, gl.FLOAT, false, 0, 0);
  const uniforms = Object.fromEntries(["projection", "view", "center", "right", "up", "selected"]
    .map(name => [name, gl.getUniformLocation(program, name)]));
  const layer = new XRWebGLLayer(session, gl);
  session.updateRenderState({baseLayer: layer});

  const panels = [...streams.entries()].map(([name, image]) => {
    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    for (const [parameter, value] of [
      [gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE], [gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE],
      [gl.TEXTURE_MIN_FILTER, gl.LINEAR], [gl.TEXTURE_MAG_FILTER, gl.LINEAR],
    ]) gl.texParameteri(gl.TEXTURE_2D, parameter, value);
    return {name, image, texture, width: defaultWidth[name] * initialScale, aspect: 16 / 9,
      center: null, nextUpload: 0, hasFrame: false};
  });

  let right, up, normal;
  let visible = initiallyVisible;
  let editing = false;
  let buttons = {toggle: false, edit: false, trigger: false};
  let selected = null;
  let drag = null;
  let lastFrame;

  function update(viewer, frame, space, controller, now) {
    if (!viewer) return editing;
    if (!right) {
      const matrix = viewer.transform.matrix;
      right = axis(matrix, 0);
      up = axis(matrix, 4);
      normal = axis(matrix, 8);
      const position = axis(matrix, 12);
      const center = add(add(add(position, mul(normal, -1.38)), mul(up, -.32)), mul(right, .20));
      for (const [index, panel] of panels.entries()) {
        panel.center = add(center, mul(right, offsets[panel.name] * initialScale));
        panel.nextUpload = now + index * 28;
      }
    }
    const elapsed = Math.min(.05, (now - (lastFrame ?? now)) / 1000);
    lastFrame = now;

    const pad = controller?.gamepad;
    const toggle = Boolean(pad?.buttons[4]?.pressed);
    const edit = Boolean(pad?.buttons[5]?.pressed);
    const trigger = Boolean(pad?.buttons[0]?.pressed);
    if (toggle && !buttons.toggle) {
      visible = !visible;
      drag = null;
      if (!visible) for (const panel of panels) panel.hasFrame = false;
      visibilityChanged(visible);
    }
    if (edit && !buttons.edit) {
      editing = !editing;
      drag = null;
      selected = null;
      if (editing && !visible) {
        visible = true;
        visibilityChanged(true);
      }
    }

    if (visible && editing && controller) {
      const pose = frame.getPose(controller.targetRaySpace, space);
      if (pose && !pose.emulatedPosition) {
        const matrix = pose.transform.matrix;
        const ray = {origin: axis(matrix, 12), direction: mul(axis(matrix, 8), -1)};
        const hit = pickWindow(panels, ray, right, up, normal);
        if (hit && !trigger) selected = hit.panel;
        if (hit && trigger && !buttons.trigger) {
          selected = hit.panel;
          drag = {panel: selected, offset: sub(selected.center, hit.point), distance: hit.distance};
        }
        const stick = pad.axes[3] || 0;
        if (drag && trigger) {
          drag.panel.center = add(add(ray.origin, mul(ray.direction, drag.distance)), drag.offset);
        }
        if (selected && Math.abs(stick) > .18) {
          selected.width = Math.min(1.5, Math.max(.14, selected.width * Math.exp(-stick * elapsed * 1.5)));
        }
      }
    }
    if (!trigger) drag = null;
    buttons = {toggle, edit, trigger};
    return editing;
  }

  function render(viewer, now) {
    gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT);
    if (!viewer || !right || !visible) return;
    const panel = panels.find(item => item.image.naturalWidth && now >= item.nextUpload);
    if (panel) {
      panel.nextUpload = now + 120;
      gl.bindTexture(gl.TEXTURE_2D, panel.texture);
      try {
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, panel.image);
        panel.aspect = panel.image.naturalWidth / panel.image.naturalHeight;
        panel.hasFrame = true;
      } catch (error) {
        if (error.name !== "InvalidStateError") throw error;
      }
    }
    for (const view of viewer.views) {
      const viewport = layer.getViewport(view);
      gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
      gl.uniformMatrix4fv(uniforms.projection, false, view.projectionMatrix);
      gl.uniformMatrix4fv(uniforms.view, false, view.transform.inverse.matrix);
      for (const item of panels) {
        if (!item.hasFrame) continue;
        gl.bindTexture(gl.TEXTURE_2D, item.texture);
        gl.uniform3fv(uniforms.center, item.center);
        gl.uniform3fv(uniforms.right, mul(right, item.width));
        gl.uniform3fv(uniforms.up, mul(up, item.width / item.aspect));
        gl.uniform1i(uniforms.selected, Number(editing && item === selected));
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
      }
    }
  }

  function reset() {
    right = null;
    selected = drag = null;
  }
  return {update, render, reset};
}
