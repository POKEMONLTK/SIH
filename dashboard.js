import * as THREE from "three";

/* =========================================================
   DOM
========================================================= */

const $ = id => document.getElementById(id);

const startBtn = $("startBtn");
const stateEl = $("state");
const timeEl = $("time");
const altitudeEl = $("altitude");
const velocityEl = $("velocity");
const accelerationEl = $("acceleration");
const flightTimeEl = $("flightTime");
const posXEl = $("posX");
const posYEl = $("posY");
const posZEl = $("posZ");
const errorEl = $("error");
const accelValueEl = $("accelValue");
const gyroValueEl = $("gyroValue");
const positionValueEl = $("positionValue");
const heroPosition = $("heroPosition");
const nav3DPosition = $("nav3DPosition");
const connectionEl = $("connection");
const pageNumber = $("pageNumber");

/* =========================================================
   SIMULATION
   One source of truth. Everything on the page reads from
   this state; nothing else controls the telemetry.
========================================================= */

const SIM_DURATION = 25;
const MAX_HISTORY = 180;

let running = false;
let simulationTime = 0;
let lastFrame = null;
const history = [];

function generateState(t) {
  const x = 180 * t + 18 * Math.sin(t * 0.35);
  const y = 80 * Math.sin(t * 0.23);
  const z = 4200 * (1 - Math.exp(-t / 7));

  const vx = 180 + 6 * Math.cos(t * 0.35);
  const vy = 80 * 0.23 * Math.cos(t * 0.23);
  const vz = (4200 / 7) * Math.exp(-t / 7);

  const ax = -2.3 * Math.sin(t * 0.35);
  const ay = -4.2 * Math.sin(t * 0.23);
  const az = -(4200 / 49) * Math.exp(-t / 7);

  const referenceX = 180 * t;
  const referenceY = 65 * Math.sin(t * 0.20);
  const referenceZ = 4200 * (1 - Math.exp(-t / 7));

  const noiseX = t === 0 ? 0 : Math.sin(t * 2.7) * 5;
  const noiseY = t === 0 ? 0 : Math.cos(t * 2.1) * 4;
  const noiseZ = t === 0 ? 0 : Math.sin(t * 1.7) * 6;

  const estimatedX = x + noiseX;
  const estimatedY = y + noiseY;
  const estimatedZ = z + noiseZ;

  const error = Math.hypot(
    estimatedX - x,
    estimatedY - y,
    estimatedZ - z
  );

  const accelSensor =
    Math.hypot(ax, ay, az) + Math.sin(t * 6) * 0.4;

  const gyroSensor =
    Math.sin(t * 1.8) * 0.15 +
    Math.sin(t * 7) * 0.025;

  return {
    t, x, y, z,
    vx, vy, vz,
    ax, ay, az,
    referenceX, referenceY, referenceZ,
    estimatedX, estimatedY, estimatedZ,
    error,
    accelSensor,
    gyroSensor
  };
}

/* =========================================================
   CANVAS UTILITIES
========================================================= */

const canvases = {
  altitude: $("altitudeGraph"),
  accel: $("accelGraph"),
  gyro: $("gyroGraph"),
  position: $("positionGraph")
};

const contexts = {};

function resizeCanvas(canvas) {
  if (!canvas) return null;

  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);

  canvas.width = Math.max(1, Math.round(rect.width * dpr));
  canvas.height = Math.max(1, Math.round(rect.height * dpr));

  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  return ctx;
}

function resizeGraphs() {
  contexts.altitude = resizeCanvas(canvases.altitude);
  contexts.accel = resizeCanvas(canvases.accel);
  contexts.gyro = resizeCanvas(canvases.gyro);
  contexts.position = resizeCanvas(canvases.position);
}

function drawGrid(ctx, width, height, rows = 5) {
  ctx.strokeStyle = "rgba(255,255,255,.065)";
  ctx.lineWidth = 1;

  for (let i = 1; i <= rows; i++) {
    const y = (height / (rows + 1)) * i;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
}

function drawSeries(ctx, values, min, max) {
  if (!ctx || values.length < 2) return;

  const canvas = ctx.canvas;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;

  ctx.beginPath();

  values.forEach((value, i) => {
    const x = (i / (MAX_HISTORY - 1)) * width;
    const normalized = (value - min) / (max - min);
    const y = height - Math.max(0, Math.min(1, normalized)) * height;

    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });

  ctx.strokeStyle = "#92d61b";
  ctx.lineWidth = 1.6;
  ctx.stroke();
}

function drawAltitudeGraph() {
  const ctx = contexts.altitude;
  if (!ctx) return;

  const width = canvases.altitude.clientWidth;
  const height = canvases.altitude.clientHeight;

  ctx.clearRect(0, 0, width, height);
  drawGrid(ctx, width, height);

  drawSeries(
    ctx,
    history.map(s => s.z),
    0,
    4500
  );
}

function drawSensorGraph(ctx, values, min, max) {
  if (!ctx) return;

  const canvas = ctx.canvas;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;

  ctx.clearRect(0, 0, width, height);
  drawGrid(ctx, width, height);

  ctx.strokeStyle = "rgba(255,255,255,.1)";
  ctx.beginPath();
  ctx.moveTo(0, height / 2);
  ctx.lineTo(width, height / 2);
  ctx.stroke();

  drawSeries(ctx, values, min, max);
}

function drawGraphs() {
  drawAltitudeGraph();

  drawSensorGraph(
    contexts.accel,
    history.map(s => s.accelSensor),
    0,
    120
  );

  drawSensorGraph(
    contexts.gyro,
    history.map(s => s.gyroSensor),
    -0.25,
    0.25
  );

  const currentX = history.length ? history[history.length - 1].x : 0;

  drawSensorGraph(
    contexts.position,
    history.map(s => s.x),
    0,
    Math.max(5000, currentX + 100)
  );
}

/* =========================================================
   MAIN 3D DIGITAL TWIN
========================================================= */

const twinContainer = $("twinContainer");

const twinScene = new THREE.Scene();
twinScene.background = new THREE.Color(0x040807);

const twinCamera = new THREE.PerspectiveCamera(55, 1, 0.1, 10000);
twinCamera.position.set(-7, 5, 10);

const twinRenderer = new THREE.WebGLRenderer({ antialias: true });
twinRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
twinRenderer.outputColorSpace = THREE.SRGBColorSpace;
twinContainer.appendChild(twinRenderer.domElement);

twinScene.add(new THREE.AmbientLight(0xffffff, 1.4));

const keyLight = new THREE.DirectionalLight(0xffffff, 2);
keyLight.position.set(5, 8, 5);
twinScene.add(keyLight);

/*
  Stylized abstract flight object for the dashboard.
  It is intentionally simple and non-operational.
*/
const twin = new THREE.Group();

const bodyMaterial = new THREE.MeshStandardMaterial({
  color: 0x8a928d,
  roughness: 0.55,
  metalness: 0.35
});

const body = new THREE.Mesh(
  new THREE.CylinderGeometry(0.42, 0.42, 3.0, 24),
  bodyMaterial
);
body.rotation.z = Math.PI / 2;
twin.add(body);

const nose = new THREE.Mesh(
  new THREE.ConeGeometry(0.42, 1.05, 24),
  bodyMaterial
);
nose.rotation.z = -Math.PI / 2;
nose.position.x = 2.0;
twin.add(nose);

const ring = new THREE.Mesh(
  new THREE.TorusGeometry(0.43, 0.045, 8, 24),
  bodyMaterial
);
ring.rotation.y = Math.PI / 2;
ring.position.x = -1.28;
twin.add(ring);

const accentRing = new THREE.Mesh(
  new THREE.TorusGeometry(0.445, 0.025, 8, 24),
  new THREE.MeshBasicMaterial({ color: 0x92d61b })
);
accentRing.rotation.y = Math.PI / 2;
accentRing.position.x = 0.45;
twin.add(accentRing);

twinScene.add(twin);

/* World */
const grid = new THREE.GridHelper(120, 60, 0x25302c, 0x18211e);
grid.position.y = -2.5;
twinScene.add(grid);

/* Reference + estimated trails */
const referencePoints = [];
for (let t = 0; t <= SIM_DURATION; t += 0.25) {
  const s = generateState(t);
  referencePoints.push(
    new THREE.Vector3(
      s.referenceX * 0.012,
      s.referenceZ * 0.006,
      s.referenceY * 0.035
    )
  );
}

const referenceLine = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints(referencePoints),
  new THREE.LineBasicMaterial({
    color: 0x68716d,
    transparent: true,
    opacity: 0.42
  })
);
twinScene.add(referenceLine);

const estimatedPoints = [];
const estimatedLine = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({
    color: 0x92d61b,
    transparent: true,
    opacity: 0.85
  })
);
twinScene.add(estimatedLine);

function updateTwin(data) {
  const position = new THREE.Vector3(
    data.estimatedX * 0.012,
    data.estimatedZ * 0.006,
    data.estimatedY * 0.035
  );

  /* The vehicle is a real 3D object moving through the scene. */
  twin.position.copy(position);

  const direction = new THREE.Vector3(
    data.vx,
    data.vz,
    data.vy
  ).normalize();

  twin.quaternion.setFromUnitVectors(
    new THREE.Vector3(1, 0, 0),
    direction
  );

  /*
     Chase-camera presentation: the camera stays behind and above the
     vehicle, with smooth lag. This makes the movement obvious instead
     of making the vehicle look pinned to the screen.
  */
  const forward = direction.clone();
  const cameraOffset = forward.clone().multiplyScalar(-7);
  cameraOffset.y += 3.2;

  const desiredCamera = position.clone().add(cameraOffset);
  twinCamera.position.lerp(desiredCamera, 0.075);

  const lookTarget = position.clone().add(forward.multiplyScalar(5));
  lookTarget.y += 0.7;
  twinCamera.lookAt(lookTarget);

  estimatedPoints.push(position.clone());
  if (estimatedPoints.length > MAX_HISTORY) estimatedPoints.shift();

  estimatedLine.geometry.dispose();
  estimatedLine.geometry =
    new THREE.BufferGeometry().setFromPoints(estimatedPoints);
}

function resizeTwin() {
  const width = twinContainer.clientWidth;
  const height = twinContainer.clientHeight;

  if (!width || !height) return;

  twinCamera.aspect = width / height;
  twinCamera.updateProjectionMatrix();
  twinRenderer.setSize(width, height, false);
}

function renderTwin() {
  requestAnimationFrame(renderTwin);
  twinRenderer.render(twinScene, twinCamera);
}

/* =========================================================
   PAGE 3 — 3D XYZ NAVIGATION
========================================================= */

const navigation3D = $("navigation3D");

let navScene;
let navCamera;
let navRenderer;
let navReferenceLine;
let navEstimateLine;
let navMarker;

const navReferencePoints = [];
const navEstimatePoints = [];

let navDragging = false;
let navPreviousX = 0;
let navPreviousY = 0;
let navRotationX = -0.35;
let navRotationY = 0.7;
let navDistance = 10;

const NAV_SCALE = {
  x: 0.012,
  altitude: 0.006,
  lateral: 0.035
};

function navPosition(x, altitude, lateral) {
  return new THREE.Vector3(
    x * NAV_SCALE.x,
    altitude * NAV_SCALE.altitude,
    lateral * NAV_SCALE.lateral
  );
}

function buildNavReference() {
  navReferencePoints.length = 0;

  for (let t = 0; t <= SIM_DURATION; t += 0.2) {
    const s = generateState(t);

    navReferencePoints.push(
      navPosition(
        s.referenceX,
        s.referenceZ,
        s.referenceY
      )
    );
  }
}

function initNavigation3D() {
  if (!navigation3D) return;

  navScene = new THREE.Scene();
  navScene.background = new THREE.Color(0x040807);

  navCamera = new THREE.PerspectiveCamera(
    42,
    navigation3D.clientWidth / navigation3D.clientHeight,
    0.1,
    1000
  );

  navRenderer = new THREE.WebGLRenderer({ antialias: true });
  navRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  navRenderer.outputColorSpace = THREE.SRGBColorSpace;
  navigation3D.appendChild(navRenderer.domElement);

  const navGrid = new THREE.GridHelper(
    22, 22, 0x39443f, 0x18211e
  );
  navScene.add(navGrid);

  navScene.add(new THREE.AxesHelper(5));

  buildNavReference();

  navReferenceLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(navReferencePoints),
    new THREE.LineBasicMaterial({
      color: 0x68716d,
      transparent: true,
      opacity: 0.55
    })
  );
  navScene.add(navReferenceLine);

  navEstimateLine = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({
      color: 0x92d61b,
      transparent: true,
      opacity: 0.9
    })
  );
  navScene.add(navEstimateLine);

  navMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.13, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x92d61b })
  );
  navScene.add(navMarker);

  resizeNavigation3D();
  updateNavigationCamera();
  setupNavigationControls();
  renderNavigation3D();
}

function updateNavigation3D(data) {
  if (!navScene || !navEstimateLine || !navMarker) return;

  const position = navPosition(
    data.estimatedX,
    data.estimatedZ,
    data.estimatedY
  );

  navMarker.position.copy(position);

  navEstimatePoints.push(position.clone());
  if (navEstimatePoints.length > MAX_HISTORY) navEstimatePoints.shift();

  navEstimateLine.geometry.dispose();
  navEstimateLine.geometry =
    new THREE.BufferGeometry().setFromPoints(navEstimatePoints);

  nav3DPosition.textContent =
    `${data.estimatedX.toFixed(0).padStart(4, "0")} / ` +
    `${data.estimatedY.toFixed(0).padStart(4, "0")} / ` +
    `${data.estimatedZ.toFixed(0).padStart(4, "0")}`;
}

function resizeNavigation3D() {
  if (!navRenderer || !navCamera || !navigation3D) return;

  const width = navigation3D.clientWidth;
  const height = navigation3D.clientHeight;

  if (!width || !height) return;

  navCamera.aspect = width / height;
  navCamera.updateProjectionMatrix();
  navRenderer.setSize(width, height, false);
}

function updateNavigationCamera() {
  if (!navCamera) return;

  const target = new THREE.Vector3(1.5, 2, 0);

  const x =
    Math.cos(navRotationX) *
    Math.sin(navRotationY) *
    navDistance;

  const y =
    Math.sin(navRotationX) *
    navDistance;

  const z =
    Math.cos(navRotationX) *
    Math.cos(navRotationY) *
    navDistance;

  navCamera.position.set(
    target.x + x,
    target.y + y,
    target.z + z
  );

  navCamera.lookAt(target);
}

function setupNavigationControls() {
  navigation3D.addEventListener("pointerdown", event => {
    navDragging = true;
    navPreviousX = event.clientX;
    navPreviousY = event.clientY;
    navigation3D.setPointerCapture(event.pointerId);
  });

  navigation3D.addEventListener("pointermove", event => {
    if (!navDragging) return;

    const dx = event.clientX - navPreviousX;
    const dy = event.clientY - navPreviousY;

    navPreviousX = event.clientX;
    navPreviousY = event.clientY;

    navRotationY -= dx * 0.008;
    navRotationX -= dy * 0.008;
    navRotationX = Math.max(-1.2, Math.min(1.2, navRotationX));

    updateNavigationCamera();
  });

  const stopDrag = () => { navDragging = false; };

  navigation3D.addEventListener("pointerup", stopDrag);
  navigation3D.addEventListener("pointercancel", stopDrag);

  navigation3D.addEventListener("wheel", event => {
    event.preventDefault();

    navDistance += event.deltaY * 0.01;
    navDistance = Math.max(4, Math.min(18, navDistance));

    updateNavigationCamera();
  }, { passive: false });
}

function renderNavigation3D() {
  requestAnimationFrame(renderNavigation3D);

  if (navRenderer && navScene && navCamera) {
    navRenderer.render(navScene, navCamera);
  }
}

/* =========================================================
   MAIN UPDATE
========================================================= */

function updateSimulation(t) {
  const data = generateState(t);

  /* Shared telemetry */
  altitudeEl.textContent = data.z.toFixed(0);

  const speed = Math.hypot(data.vx, data.vy, data.vz);
  velocityEl.textContent = speed.toFixed(0);

  const acceleration = Math.hypot(data.ax, data.ay, data.az);
  accelerationEl.textContent = acceleration.toFixed(1);

  flightTimeEl.textContent = t.toFixed(1);

  /* Position */
  posXEl.textContent = data.x.toFixed(2);
  posYEl.textContent = data.y.toFixed(2);
  posZEl.textContent = data.z.toFixed(2);
  errorEl.textContent = data.error.toFixed(2);

  /* Sensors */
  accelValueEl.textContent = data.accelSensor.toFixed(2);
  gyroValueEl.textContent = data.gyroSensor.toFixed(3);
  positionValueEl.textContent =
    Math.hypot(data.x, data.y).toFixed(1);

  heroPosition.textContent =
    `${data.x.toFixed(0).padStart(4, "0")} / ` +
    `${data.y.toFixed(0).padStart(4, "0")} / ` +
    `${data.z.toFixed(0).padStart(4, "0")}`;

  /* History */
  history.push(data);
  if (history.length > MAX_HISTORY) history.shift();

  /* Both 3D views use this exact same state */
  updateTwin(data);
  updateNavigation3D(data);

  drawGraphs();

  timeEl.textContent =
    `00:${t.toFixed(1).padStart(4, "0")}`;
}

function simulationLoop(timestamp) {
  if (!running) return;

  if (lastFrame === null) lastFrame = timestamp;

  const delta = (timestamp - lastFrame) / 1000;
  lastFrame = timestamp;

  simulationTime += delta;

  if (simulationTime >= SIM_DURATION) {
    simulationTime = SIM_DURATION;
    updateSimulation(simulationTime);

    running = false;
    stateEl.textContent = "COMPLETE";
    connectionEl.textContent = "SIMULATION COMPLETE";
    startBtn.textContent = "RESTART SIMULATION";
    return;
  }

  updateSimulation(simulationTime);
  requestAnimationFrame(simulationLoop);
}

/* =========================================================
   START / RESTART
========================================================= */

function resetSimulation() {
  running = false;
  simulationTime = 0;
  lastFrame = null;

  history.length = 0;
  estimatedPoints.length = 0;
  navEstimatePoints.length = 0;

  estimatedLine.geometry.dispose();
  estimatedLine.geometry = new THREE.BufferGeometry();

  if (navEstimateLine) {
    navEstimateLine.geometry.dispose();
    navEstimateLine.geometry = new THREE.BufferGeometry();
  }

  updateSimulation(0);

  stateEl.textContent = "STANDBY";
  connectionEl.textContent = "SIMULATION READY";
  startBtn.textContent = "START SIMULATION";
}

startBtn.addEventListener("click", () => {
  if (running) return;

  if (simulationTime >= SIM_DURATION) {
    resetSimulation();
  }

  running = true;
  stateEl.textContent = "RUNNING";
  connectionEl.textContent = "SIMULATION ACTIVE";
  startBtn.textContent = "SIMULATION RUNNING";

  lastFrame = null;
  requestAnimationFrame(simulationLoop);
});

/* =========================================================
   SCROLL PAGE COUNTER
========================================================= */

const scenes = [...document.querySelectorAll(".scene")];

const sceneObserver = new IntersectionObserver(
  entries => {
    for (const entry of entries) {
      if (!entry.isIntersecting) continue;

      const index = scenes.indexOf(entry.target) + 1;
      pageNumber.textContent =
        `${String(index).padStart(2, "0")} / 05`;
    }
  },
  { threshold: 0.55 }
);

scenes.forEach(scene => sceneObserver.observe(scene));

/* =========================================================
   RESIZE + INITIALIZATION
========================================================= */

window.addEventListener("resize", () => {
  resizeGraphs();
  resizeTwin();
  resizeNavigation3D();
});

resizeGraphs();
resizeTwin();
initNavigation3D();
updateSimulation(0);
renderTwin();
