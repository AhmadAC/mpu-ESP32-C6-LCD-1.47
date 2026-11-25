import { servoAngles } from "./servos.js";

const Scene3D = (function () {
  const DEG = Math.PI / 180;
  const LIGHT = norm([0.45, -0.55, 0.75]);

  let canvas = null;
  let ctx = null;
  let cssW = 300;
  let cssH = 300;
  let mode = "arm";
  let camAz = -38 * DEG;
  let camEl = 24 * DEG;
  let camDist = 8.8;
  let dragId = null;
  let dragX = 0;
  let dragY = 0;
  let pinch = {};
  let running = false;
  let isStationary = false;

  const shown = { roll: 0, pitch: 0, yaw: 0 };
  const target = { roll: 0, pitch: 0, yaw: 0 };

  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function scale(a, k) { return [a[0] * k, a[1] * k, a[2] * k]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }
  function norm(a) {
    const l = Math.sqrt(dot(a, a)) || 1;
    return [a[0] / l, a[1] / l, a[2] / l];
  }
  function mv(R, v) {
    return [
      R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
      R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
      R[6] * v[0] + R[7] * v[1] + R[8] * v[2]
    ];
  }
  function mm(A, B) {
    const o = new Array(9);
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        o[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
      }
    }
    return o;
  }
  function rotX(a) { const c = Math.cos(a), s = Math.sin(a); return [1, 0, 0, 0, c, -s, 0, s, c]; }
  function rotY(a) { const c = Math.cos(a), s = Math.sin(a); return [c, 0, s, 0, 1, 0, -s, 0, c]; }
  function rotZ(a) { const c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; }

  function eulerToR(rollDeg, pitchDeg, yawDeg) {
    return mm(rotZ(yawDeg * DEG), mm(rotY(pitchDeg * DEG), rotX(rollDeg * DEG)));
  }

  function camera() {
    const target3 = mode === "arm" ? [0.6, 0, 0.2] : [0, 0, 0.2];
    const pos = add(target3, [
      camDist * Math.cos(camEl) * Math.cos(camAz),
      camDist * Math.cos(camEl) * Math.sin(camAz),
      camDist * Math.sin(camEl)
    ]);
    const f = norm(sub(target3, pos));
    const right = norm(cross(f, [0, 0, 1]));
    const up = cross(right, f);
    const focal = Math.min(cssW, cssH) * 1.15;
    return { pos, f, right, up, focal };
  }

  function project(cam, p) {
    const v = sub(p, cam.pos);
    const z = dot(v, cam.f);
    const zc = z < 0.05 ? 0.05 : z;
    return [
      cssW / 2 + (cam.focal * dot(v, cam.right)) / zc,
      cssH / 2 - (cam.focal * dot(v, cam.up)) / zc,
      z
    ];
  }

  function boxPolys(center, R, half, color) {
    const sgn = [
      [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
      [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
    ];
    const v = sgn.map(s => add(center, mv(R, [s[0] * half[0], s[1] * half[1], s[2] * half[2]])));
    const faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 4, 7, 3]];
    return faces.map(f => ({ pts: f.map(i => v[i]), color }));
  }

  function prismPolys(center, R, radius, halfLen, sides, color) {
    const ring = [];
    for (let i = 0; i < sides; i++) {
      const a = (i / sides) * Math.PI * 2;
      ring.push([Math.cos(a) * radius, Math.sin(a) * radius]);
    }
    const w = (x, y, z) => add(center, mv(R, [x, y, z]));
    const out = [];
    for (let i = 0; i < sides; i++) {
      const a = ring[i];
      const b = ring[(i + 1) % sides];
      out.push({ pts: [w(a[0], -halfLen, a[1]), w(b[0], -halfLen, b[1]), w(b[0], halfLen, b[1]), w(a[0], halfLen, a[1])], color });
    }
    out.push({ pts: ring.map(r => w(r[0], halfLen, r[1])), color });
    out.push({ pts: ring.map(r => w(r[0], -halfLen, r[1])).reverse(), color });
    return out;
  }

  function shade(color, n) {
    const k = 0.35 + 0.65 * Math.max(0, dot(n, LIGHT));
    return `rgb(${Math.round(color[0] * k)},${Math.round(color[1] * k)},${Math.round(color[2] * k)})`;
  }

  function drawPolys(cam, polys) {
    const items = [];
    for (const poly of polys) {
      const p = poly.pts;
      if (p.length < 3) continue;
      const c = scale(p.reduce((acc, q) => add(acc, q), [0, 0, 0]), 1 / p.length);
      const toCam = sub(cam.pos, c);
      let n = norm(cross(sub(p[1], p[0]), sub(p[2], p[0])));
      if (dot(n, toCam) < 0) {
        n = scale(n, -1);
      }
      items.push({ poly, n, depth: Math.sqrt(dot(toCam, toCam)) });
    }
    items.sort((a, b) => b.depth - a.depth);

    for (const it of items) {
      const pr = it.poly.pts.map(q => project(cam, q));
      ctx.beginPath();
      ctx.moveTo(pr[0][0], pr[0][1]);
      for (let i = 1; i < pr.length; i++) ctx.lineTo(pr[i][0], pr[i][1]);
      ctx.closePath();
      ctx.fillStyle = shade(it.poly.color, it.n);
      ctx.fill();
      ctx.strokeStyle = "rgba(2,6,23,0.55)";
      ctx.lineWidth = 0.8;
      ctx.stroke();
    }
  }

  function line3(cam, a, b, color, width) {
    const pa = project(cam, a);
    const pb = project(cam, b);
    ctx.beginPath();
    ctx.moveTo(pa[0], pa[1]);
    ctx.lineTo(pb[0], pb[1]);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.lineCap = "round";
    ctx.stroke();
  }

  function label3(cam, p, text, color) {
    const pp = project(cam, p);
    ctx.fillStyle = color;
    ctx.font = "700 11px -apple-system, Helvetica, Arial, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, pp[0], pp[1]);
  }

  function drawGrid(cam, z, extent, step) {
    ctx.save();
    for (let g = -extent; g <= extent + 1e-6; g += step) {
      const major = Math.abs(g) < 1e-6;
      const col = major ? "rgba(148,163,184,0.38)" : "rgba(148,163,184,0.14)";
      line3(cam, [-extent, g, z], [extent, g, z], col, 1);
      line3(cam, [g, -extent, z], [g, extent, z], col, 1);
    }
    ctx.restore();
  }

  function drawAxes(cam, origin, R, len, width, alpha, withLabels) {
    const axes = [
      { v: [1, 0, 0], c: `rgba(248,113,113,${alpha})`, t: "X" },
      { v: [0, 1, 0], c: `rgba(74,222,128,${alpha})`, t: "Y" },
      { v: [0, 0, 1], c: `rgba(96,165,250,${alpha})`, t: "Z" }
    ];
    for (const ax of axes) {
      const tip = add(origin, scale(mv(R, ax.v), len));
      line3(cam, origin, tip, ax.c, width);
      if (withLabels) label3(cam, add(origin, scale(mv(R, ax.v), len * 1.14)), ax.t, ax.c);
    }
  }

  function buildImuBoard(R) {
    const polys = [];
    const o = [0, 0, 0.4];

    polys.push(...boxPolys(o, R, [0.04, 1.05, 0.8], [18, 55, 115]));

    for (const side of [-1, 1]) {
      const hole = add(o, mv(R, [0, side * 0.82, 0.55]));
      polys.push(...prismPolys(hole, mm(R, rotY(Math.PI / 2)), 0.12, 0.05, 12, [212, 175, 55]));
    }

    const chipC = add(o, mv(R, [0.06, 0.0, 0.0]));
    polys.push(...boxPolys(chipC, R, [0.03, 0.32, 0.32], [28, 32, 38]));

    const dotC = add(o, mv(R, [0.09, -0.22, 0.22]));
    polys.push(...boxPolys(dotC, R, [0.01, 0.04, 0.04], [220, 220, 220]));

    const ledC = add(o, mv(R, [0.06, 0.65, -0.2]));
    polys.push(...boxPolys(ledC, R, [0.03, 0.08, 0.08], [34, 197, 94]));

    const regC = add(o, mv(R, [0.06, 0.0, 0.52]));
    polys.push(...boxPolys(regC, R, [0.02, 0.18, 0.12], [47, 55, 69]));

    const headerC = add(o, mv(R, [-0.12, 0.0, -0.8]));
    polys.push(...boxPolys(headerC, R, [0.12, 1.02, 0.12], [15, 23, 42]));

    for (let i = 0; i < 8; i++) {
      const yPos = -0.875 + i * 0.25;
      const pinLeg = add(o, mv(R, [-0.3, yPos, -0.8]));
      polys.push(...boxPolys(pinLeg, R, [0.15, 0.035, 0.035], [234, 179, 8]));
      const pinFoot = add(o, mv(R, [-0.45, yPos, -1.1]));
      polys.push(...boxPolys(pinFoot, R, [0.035, 0.035, 0.3], [234, 179, 8]));
    }

    return polys;
  }

  function buildArm(angles, geom) {
    const s1 = angles[0] !== undefined ? angles[0] : 90;
    const s2 = angles[1] !== undefined ? angles[1] : 90;
    const s3 = angles[2] !== undefined ? angles[2] : 90;
    const s4 = angles[3] !== undefined ? angles[3] : 90;

    const polys = [];
    const I = [1, 0, 0, 0, 1, 0, 0, 0, 1];
    const groundZ = -1.65;

    const groundPlate = [0, 0, groundZ + 0.08];
    polys.push(...boxPolys(groundPlate, I, [1.35, 1.15, 0.08], [30, 41, 59]));

    const scissorNorm = Math.max(0, Math.min(180, s4)) / 180.0;
    const liftH = 0.12 + scissorNorm * 1.25;

    const tiers = 2;
    const tierH = liftH / tiers;
    for (let t = 0; t < tiers; t++) {
      const tz = groundZ + 0.16 + (t + 0.5) * tierH;
      const strutAngle = 0.15 + (1.0 - scissorNorm) * 0.45;
      const halfStrut = Math.max(0.28, Math.sqrt(0.18 + tierH * tierH * 0.4));

      for (const side of [-0.65, 0.65]) {
        const R1 = rotY(strutAngle);
        const R2 = rotY(-strutAngle);
        const c1 = [0, side, tz];
        const c2 = [0, side, tz];
        polys.push(...boxPolys(c1, R1, [0.045, 0.03, halfStrut], [245, 158, 11]));
        polys.push(...boxPolys(c2, R2, [0.045, 0.03, halfStrut], [217, 119, 6]));
      }
    }

    const platformZ = groundZ + 0.16 + liftH;
    const upperPlatform = [0, 0, platformZ + 0.09];
    polys.push(...boxPolys(upperPlatform, I, [1.25, 1.05, 0.09], [71, 85, 105]));

    const pedestal = [0, 0, platformZ + 0.09 + 0.45];
    polys.push(...boxPolys(pedestal, I, [0.42, 0.42, 0.45], [100, 116, 139]));

    const S = [0, 0, platformZ + 0.09 + 0.9];
    polys.push(...prismPolys(S, I, 0.34, 0.46, 14, [245, 158, 11]));

    const phi1 = (s3 - 90) * DEG;
    const R1 = rotY(phi1);
    const L1 = geom.l1;
    const c1 = add(S, mv(R1, [0, 0, L1 / 2]));
    polys.push(...boxPolys(c1, R1, [0.2, 0.2, L1 / 2], [56, 189, 248]));

    const E = add(S, mv(R1, [0, 0, L1]));
    polys.push(...prismPolys(E, I, 0.27, 0.34, 14, [245, 158, 11]));

    const phi2 = phi1 + (90 + (s2 - 90)) * DEG;
    const R2 = rotY(phi2);
    const L2 = geom.l2;
    const c2 = add(E, mv(R2, [0, 0, L2 / 2]));
    polys.push(...boxPolys(c2, R2, [0.16, 0.16, L2 / 2], [14, 165, 233]));

    const W = add(E, mv(R2, [0, 0, L2]));
    const palm = add(W, mv(R2, [0, 0, 0.14]));
    polys.push(...boxPolys(palm, R2, [0.2, 0.42, 0.14], [100, 116, 139]));

    const open = Math.max(0, Math.min(180, s1)) / 180 * 50 * DEG;
    for (const side of [-1, 1]) {
      const Rf = mm(R2, rotX(-side * open));
      const base = add(W, mv(R2, [0, side * 0.34, 0.28]));
      const fc = add(base, mv(Rf, [0, 0, 0.4]));
      polys.push(...boxPolys(fc, Rf, [0.09, 0.07, 0.4], [52, 211, 153]));
    }

    return { polys, groundZ, S, E, W };
  }

  function smoothAngle(cur, tgt, wrap) {
    let d = tgt - cur;
    if (wrap) {
      while (d > 180) d -= 360;
      while (d < -180) d += 360;
    }
    if (isStationary && Math.abs(d) < 0.2) return tgt;
    return cur + d * 0.35;
  }

  function render() {
    if (!ctx) return;
    shown.roll = smoothAngle(shown.roll, target.roll, false);
    shown.pitch = smoothAngle(shown.pitch, target.pitch, false);
    shown.yaw = smoothAngle(shown.yaw, target.yaw, true);

    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssW, cssH);
    const bg = ctx.createLinearGradient(0, 0, 0, cssH);
    bg.addColorStop(0, "#0b1224");
    bg.addColorStop(1, "#020617");
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, cssW, cssH);

    const cam = camera();
    const I = [1, 0, 0, 0, 1, 0, 0, 0, 1];

    if (mode === "imu") {
      drawGrid(cam, -1.3, 3, 1);
      drawAxes(cam, [0, 0, -1.3], I, 1.2, 1.4, 0.35, false);
      const R = eulerToR(shown.roll, shown.pitch, shown.yaw);
      drawPolys(cam, buildImuBoard(R));
      drawAxes(cam, [0, 0, 0.4], R, 1.6, 2.4, 0.95, true);
    } else {
      const angles = (typeof servoAngles !== "undefined") ? servoAngles : [90, 90, 90, 90];
      const arm = buildArm(angles, { l1: 1.7, l2: 1.45 });
      drawGrid(cam, arm.groundZ, 3, 1);
      drawAxes(cam, [0, 0, arm.groundZ], I, 1.2, 1.4, 0.35, true);
      drawPolys(cam, arm.polys);
    }
  }

  function loop() {
    if (!running) return;
    render();
    window.requestAnimationFrame(loop);
  }

  function resize() {
    if (!canvas) return;
    const rect = canvas.parentElement ? canvas.parentElement.getBoundingClientRect() : { width: 320 };
    const w = Math.max(200, Math.floor(rect.width || 320));
    const h = Math.max(220, Math.floor(w * 0.82));
    const dpr = window.devicePixelRatio || 1;
    cssW = w;
    cssH = h;
    canvas.style.width = `${w}px`;
    canvas.style.height = `${h}px`;
    canvas.width = Math.floor(w * dpr);
    canvas.height = Math.floor(h * dpr);
  }

  function onPointerDown(e) {
    canvas.setPointerCapture(e.pointerId);
    pinch[e.pointerId] = { x: e.clientX, y: e.clientY };
    if (Object.keys(pinch).length === 1) {
      dragId = e.pointerId;
      dragX = e.clientX;
      dragY = e.clientY;
    }
  }

  function onPointerMove(e) {
    const ids = Object.keys(pinch);
    if (ids.length === 2 && pinch[e.pointerId]) {
      const other = ids.find(k => Number(k) !== e.pointerId);
      const before = Math.hypot(pinch[e.pointerId].x - pinch[other].x, pinch[e.pointerId].y - pinch[other].y);
      pinch[e.pointerId] = { x: e.clientX, y: e.clientY };
      const after = Math.hypot(pinch[e.pointerId].x - pinch[other].x, pinch[e.pointerId].y - pinch[other].y);
      if (before > 0) camDist = Math.max(3.5, Math.min(14, camDist * (before / Math.max(after, 1))));
      return;
    }
    if (pinch[e.pointerId]) pinch[e.pointerId] = { x: e.clientX, y: e.clientY };
    if (e.pointerId !== dragId) return;
    camAz -= (e.clientX - dragX) * 0.008;
    camEl += (e.clientY - dragY) * 0.008;
    camEl = Math.max(-5 * DEG, Math.min(85 * DEG, camEl));
    dragX = e.clientX;
    dragY = e.clientY;
  }

  function onPointerUp(e) {
    delete pinch[e.pointerId];
    if (e.pointerId === dragId) dragId = null;
  }

  function onWheel(e) {
    e.preventDefault();
    camDist = Math.max(3.5, Math.min(14, camDist * (1 + e.deltaY * 0.001)));
  }

  function init(canvasEl) {
    canvas = canvasEl;
    if (!canvas) return;
    ctx = canvas.getContext("2d");
    resize();
    window.addEventListener("resize", resize);
    window.addEventListener("orientationchange", () => setTimeout(resize, 250));
    canvas.addEventListener("pointerdown", onPointerDown);
    canvas.addEventListener("pointermove", onPointerMove);
    canvas.addEventListener("pointerup", onPointerUp);
    canvas.addEventListener("pointercancel", onPointerUp);
    canvas.addEventListener("wheel", onWheel, { passive: false });
    if (!running) {
      running = true;
      window.requestAnimationFrame(loop);
    }
  }

  function setOrientation(roll, pitch, yaw, still = false) {
    isStationary = still;
    target.roll = roll;
    target.pitch = pitch;
    target.yaw = yaw;
  }

  function setMode(next) {
    mode = next === "imu" ? "imu" : "arm";
  }

  function getMode() {
    return mode;
  }

  function resetView() {
    camAz = -38 * DEG;
    camEl = 24 * DEG;
    camDist = 8.8;
  }

  return { init, resize, setOrientation, setMode, getMode, resetView };
})();

window.Scene3D = Scene3D;

export { Scene3D };