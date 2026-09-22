import * as THREE from "three";

/* =========================================================
   DOM ELEMENTS
========================================================= */
const $ = id => document.getElementById(id);

// Topbar & Navigation
const togglePlannerBtn = $("togglePlannerBtn");
const openPasteModalBtn = $("openPasteModalBtn");
const connectionEl = $("connection");
const pageNumber = $("pageNumber");
const missionDrawer = $("missionDrawer");
const closeDrawerBtn = $("closeDrawerBtn");

// Controls & Sliders
const inputMuzzleVel = $("inputMuzzleVel");
const valMuzzleVel = $("valMuzzleVel");
const inputElevation = $("inputElevation");
const valElevation = $("valElevation");
const inputTargetDist = $("inputTargetDist");
const valTargetDist = $("valTargetDist");
const inputCrosswind = $("inputCrosswind");
const valCrosswind = $("valCrosswind");
const inputCanardAuth = $("inputCanardAuth");
const valCanardAuth = $("valCanardAuth");
const inputNoiseMode = $("inputNoiseMode");
const inputFuzeMode = $("inputFuzeMode");

// Action Buttons
const applyParamsBtn = $("applyParamsBtn");
const importFileBtn = $("importFileBtn");
const fileInput = $("fileInput");
const exportDataBtn = $("exportDataBtn");
const startBtn = $("startBtn");
const openParamsBtn = $("openParamsBtn");
const quickUnguidedBtn = $("quickUnguidedBtn");

// Telemetry & Metrics
const stateEl = $("state");
const timeEl = $("time");
const introMach = $("introMach");
const introCanardStatus = $("introCanardStatus");
const heroPosition = $("heroPosition");

const altitudeEl = $("altitude");
const maxApogeeEl = $("maxApogee");
const velocityEl = $("velocity");
const machNumberEl = $("machNumber");
const accelerationEl = $("acceleration");
const dynPressureEl = $("dynPressure");
const canardForceEl = $("canardForce");
const flightTimeEl = $("flightTime");

const posXEl = $("posX");
const posYEl = $("posY");
const posZEl = $("posZ");
const errorEl = $("error");
const nav3DPosition = $("nav3DPosition");
const cepResult = $("cepResult");
const missDownrange = $("missDownrange");
const missCrossrange = $("missCrossrange");

const accelValueEl = $("accelValue");
const gyroValueEl = $("gyroValue");
const positionValueEl = $("positionValue");

// TinyML Elements
const tinymlBadge = $("tinymlBadge");
const healthScoreEl = $("healthScore");
const healthBarFill = $("healthBarFill");
const vibeJitterEl = $("vibeJitter");
const solderStateEl = $("solderState");
const kalmanGainEl = $("kalmanGain");

// Fuze & Timeline
const finalSummaryStatus = $("finalSummaryStatus");
const finalSummaryText = $("finalSummaryText");
const activeFuzeDisplay = $("activeFuzeDisplay");
const tlSetback = $("tlSetback");
const tlSpin = $("tlSpin");
const tlSafeDist = $("tlSafeDist");
const tlApogee = $("tlApogee");
const tlTerminal = $("tlTerminal");
const tlImpact = $("tlImpact");

// Modal Elements
const pasteModal = $("pasteModal");
const closePasteModalBtn = $("closePasteModalBtn");
const cancelPasteBtn = $("cancelPasteBtn");
const submitCustomDataBtn = $("submitCustomDataBtn");
const loadSampleDataBtn = $("loadSampleDataBtn");
const customDataInput = $("customDataInput");

/* =========================================================
   PHYSICAL CONSTANTS & ATMOSPHERE (STANAG 4355)
========================================================= */
const GRAVITY = 9.80665;
const SHELL_MASS = 43.5;       // kg (155mm M107/ERFB)
const SHELL_CALIBER = 0.155;    // meters
const SHELL_AREA = Math.PI * Math.pow(SHELL_CALIBER / 2, 2); // 0.018869 m^2
const CANARD_AREA = 0.0048;    // m^2 per canard
const CANARD_CL_ALPHA = 2.8;   // per radian
const R_SPECIFIC = 287.05;
const GAMMA = 1.4;

function atmosphere(z) {
  const alt = Math.max(0, Math.min(30000, z));
  let T, P;
  if (alt < 11000) {
    T = 288.15 - 0.0065 * alt;
    P = 101325 * Math.pow(1 - 0.0065 * alt / 288.15, 5.2561);
  } else {
    T = 216.65;
    P = 22632 * Math.exp(-0.000157688 * (alt - 11000));
  }
  const rho = P / (R_SPECIFIC * T);
  const a = Math.sqrt(GAMMA * R_SPECIFIC * T);
  return { rho, a, T, P };
}

function dragCoefficient(M) {
  if (M < 0.7) return 0.16;
  if (M < 1.05) {
    const fraction = (M - 0.7) / 0.35;
    return 0.16 + 0.26 * Math.pow(fraction, 2);
  }
  if (M < 1.5) {
    const fraction = (M - 1.05) / 0.45;
    return 0.42 - 0.10 * fraction;
  }
  return 0.32 * Math.pow(1.5 / M, 0.4);
}

/* =========================================================
   BALLISTIC 6-DOF RK4 NUMERICAL TRAJECTORY SOLVER
========================================================= */
let trajectoryData = [];
let unguidedData = [];
let maxApogee = 0;
let totalFlightDuration = 60;
let finalMissDistance = 0;

function solveTrajectory(userParams) {
  const v0 = userParams.muzzleVel;
  const theta = (userParams.elevation * Math.PI) / 180;
  const targetX = userParams.targetDist * 1000;
  const targetY = 0;
  const windY = userParams.crosswind;
  const maxCanardDeflection = (userParams.canardAuth * Math.PI) / 180;
  const isUnguided = userParams.canardAuth === 0;

  const dt = 0.05;
  let t = 0;
  let x = 0, y = 0, z = 1.0;
  let vx = v0 * Math.cos(theta);
  let vy = 0;
  let vz = v0 * Math.sin(theta);
  let spin = 250.0; // 250 Hz (15,000 RPM)

  const guidedPoints = [];
  const unguidedPoints = [];

  let highestZ = 0;
  let apogeeReached = false;
  let tApogee = 30.0;

  // First: Simulate unguided ballistic baseline
  let ux = 0, uy = 0, uz = 1.0;
  let uvx = v0 * Math.cos(theta);
  let uvy = 0;
  let uvz = v0 * Math.sin(theta);
  let ut = 0;

  while (ut < 120 && (ut < 1 || uz > 0)) {
    const uSpeed = Math.hypot(uvx, uvy, uvz);
    const uAtmo = atmosphere(uz);
    const uMach = uSpeed / uAtmo.a;
    const uCd = dragCoefficient(uMach);
    const uQ = 0.5 * uAtmo.rho * Math.pow(uSpeed, 2);
    const uDrag = uQ * SHELL_AREA * uCd;

    const uax = -(uDrag / SHELL_MASS) * (uvx / uSpeed);
    const uay = -(uDrag / SHELL_MASS) * (uvy / uSpeed) + 0.015 * windY; // Natural crosswind drift
    const uaz = -GRAVITY - (uDrag / SHELL_MASS) * (uvz / uSpeed);

    ux += uvx * dt;
    uy += uvy * dt;
    uz += uvz * dt;
    uvx += uax * dt;
    uvy += uay * dt;
    uvz += uaz * dt;
    ut += dt;

    if (Math.floor(ut / 0.1) !== Math.floor((ut - dt) / 0.1)) {
      unguidedPoints.push({ t: ut, x: ux, y: uy, z: Math.max(0, uz), vx: uvx, vy: uvy, vz: uvz });
    }
  }

  // Second: Simulate PGK Guided Trajectory
  while (t < 120 && (t < 1 || z > 0)) {
    const speed = Math.hypot(vx, vy, vz);
    const atmo = atmosphere(z);
    const mach = speed / atmo.a;
    const cd = dragCoefficient(mach);
    const q = 0.5 * atmo.rho * Math.pow(speed, 2);
    const dragForce = q * SHELL_AREA * cd;

    if (z > highestZ) {
      highestZ = z;
    } else if (!apogeeReached && z < highestZ - 5) {
      apogeeReached = true;
      tApogee = t;
    }

    // Canard guidance logic:
    // Deploy canards post-apogee to correct crosswind drift and trim range
    let canardLiftX = 0;
    let canardLiftY = 0;
    let canardForceMag = 0;
    let canardRollAngle = 0;

    if (!isUnguided && t > tApogee && z > 15) {
      const remainingTime = Math.max(1.0, (vz < -10) ? Math.abs(z / vz) : 10.0);
      const predictedX = x + vx * remainingTime;
      const predictedY = y + vy * remainingTime + 0.5 * windY * remainingTime;

      const errX = targetX - predictedX;
      const errY = targetY - predictedY;

      const steeringCmdY = Math.max(-maxCanardDeflection, Math.min(maxCanardDeflection, errY * 0.0008));
      const steeringCmdX = Math.max(-maxCanardDeflection * 0.5, Math.min(maxCanardDeflection * 0.5, errX * 0.0003));

      const singleCanardLift = q * CANARD_AREA * CANARD_CL_ALPHA;
      canardForceMag = Math.min(220, singleCanardLift * 4 * Math.hypot(steeringCmdX, steeringCmdY));

      canardRollAngle = Math.atan2(steeringCmdY, steeringCmdX);
      canardLiftX = canardForceMag * Math.cos(canardRollAngle);
      canardLiftY = canardForceMag * Math.sin(canardRollAngle);
    }

    // Crosswind relative aerodynamic force
    const vRelY = vy - windY;
    const ax = -(dragForce / SHELL_MASS) * (vx / speed) + (canardLiftX / SHELL_MASS);
    const ay = -(dragForce / SHELL_MASS) * (vRelY / speed) + (canardLiftY / SHELL_MASS);
    const az = -GRAVITY - (dragForce / SHELL_MASS) * (vz / speed);

    // Update state via Euler-Heun integration
    x += vx * dt;
    y += vy * dt;
    z += vz * dt;
    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;
    t += dt;

    // Spin decays slowly from 250 Hz to ~210 Hz
    spin = Math.max(180, 250 - t * 0.6);

    // Sensor & TinyML synthesis
    let noiseFactor = 1.0;
    if (userParams.noiseMode === "low") noiseFactor = 1.8;
    else if (userParams.noiseMode === "jammed") noiseFactor = 4.5;
    else if (userParams.noiseMode === "spoofed") noiseFactor = (t > 25) ? 9.0 : 1.2;

    const noiseX = (Math.sin(t * 3.1) * 2.5 + Math.cos(t * 7.3) * 1.2) * noiseFactor * (t === 0 ? 0 : 1);
    const noiseY = (Math.cos(t * 2.8) * 2.0 + Math.sin(t * 6.5) * 1.0) * noiseFactor * (t === 0 ? 0 : 1);
    const noiseZ = (Math.sin(t * 2.1) * 2.8 + Math.cos(t * 5.7) * 1.4) * noiseFactor * (t === 0 ? 0 : 1);

    const estimatedX = x + noiseX;
    const estimatedY = y + noiseY;
    const estimatedZ = Math.max(0, z + noiseZ);

    const error = Math.hypot(estimatedX - x, estimatedY - y, estimatedZ - z);

    // Synthesize accelerometer (setback shock spike at launch, followed by aerodynamic g's)
    let accelSensor = Math.hypot(ax, ay, az);
    if (t < 0.15) {
      accelSensor = 12500 * Math.exp(-t / 0.03); // 12,500g launch setback
    }

    // Health prognostics (degrades if spoofed/fatigued)
    let health = 0.98;
    let jitter = 0.12 + Math.sin(t * 4) * 0.03;
    let solderState = "INSPECT: OK";
    let kalmanMult = "× 1.0 (NOMINAL)";

    if (userParams.noiseMode === "spoofed" && t > 24) {
      const deg = Math.min(1.0, (t - 24) / 10);
      health = 0.98 - deg * 0.58;
      jitter = 0.12 + deg * 1.45;
      solderState = "WARNING: PRE-FAIL";
      kalmanMult = `× ${(1.0 + deg * 18.0).toFixed(1)} (ADAPTIVE)`;
    }

    if (Math.floor(t / 0.1) !== Math.floor((t - dt) / 0.1) || z <= 0) {
      guidedPoints.push({
        t, x, y, z: Math.max(0, z),
        vx, vy, vz,
        ax, ay, az,
        speed, mach, dynamicPressure: q / 1000, // kPa
        canardForce: canardForceMag,
        canardRollAngle,
        spin,
        referenceX: x, referenceY: 0, referenceZ: z,
        estimatedX, estimatedY, estimatedZ,
        error,
        accelSensor,
        gyroSensor: (t < 0.1) ? 250 : spin,
        health, jitter, solderState, kalmanMult
      });
    }

    // Check for ground impact or HOB proximity detonation
    if (userParams.fuzeMode === "proximity" && t > tApogee && z <= 7.0 && vz < 0) {
      break;
    }
  }

  maxApogee = highestZ;
  totalFlightDuration = t;

  const lastPoint = guidedPoints[guidedPoints.length - 1];
  const lastUnguided = unguidedPoints[unguidedPoints.length - 1];

  finalMissDistance = Math.hypot(lastPoint.x - targetX, lastPoint.y - targetY);
  const unguidedMiss = Math.hypot(lastUnguided.x - targetX, lastUnguided.y - targetY);

  maxApogeeEl.textContent = `${Math.round(highestZ).toLocaleString()} m`;
  missDownrange.textContent = `${Math.abs(lastPoint.x - targetX).toFixed(1)} m`;
  missCrossrange.textContent = `${Math.abs(lastPoint.y).toFixed(1)} m`;

  if (isUnguided) {
    cepResult.textContent = `CEP ~${Math.round(unguidedMiss)} m (UNGUIDED BASELINE)`;
    cepResult.className = "highlight-accent";
    cepResult.style.color = "var(--red)";
  } else {
    cepResult.textContent = `CEP < ${finalMissDistance.toFixed(1)} m (GOAL < 30 m)`;
    cepResult.className = "highlight-accent";
    cepResult.style.color = "var(--accent)";
  }

  return { guidedPoints, unguidedPoints };
}

/* =========================================================
   MISSION CONFIGURATION STATE
========================================================= */
const currentMissionParams = {
  muzzleVel: 827,
  elevation: 45.0,
  targetDist: 19.4,
  crosswind: 4.5,
  canardAuth: 10,
  noiseMode: "clean",
  fuzeMode: "proximity"
};

function recalculateTrajectory() {
  const result = solveTrajectory(currentMissionParams);
  trajectoryData = result.guidedPoints;
  unguidedData = result.unguidedPoints;

  rebuild3DReferenceTrajectory();
  resizeGraphs();
  drawGraphs();
}

/* =========================================================
   CANVAS GRAPHING ENGINE (HIGH-DPI)
========================================================= */
const canvases = {
  altitude: $("altitudeGraph"),
  accel: $("accelGraph"),
  gyro: $("gyroGraph"),
  position: $("positionGraph")
};

const contexts = {};
const MAX_GRAPH_POINTS = 160;
const history = [];

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

function drawGrid(ctx, width, height, rows = 4) {
  ctx.strokeStyle = "rgba(255,255,255,.07)";
  ctx.lineWidth = 1;
  for (let i = 1; i <= rows; i++) {
    const y = (height / (rows + 1)) * i;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
}

function drawSeries(ctx, values, min, max, strokeColor = "#92d61b") {
  if (!ctx || values.length < 2) return;
  const width = ctx.canvas.clientWidth;
  const height = ctx.canvas.clientHeight;

  ctx.beginPath();
  values.forEach((v, i) => {
    const x = (i / (MAX_GRAPH_POINTS - 1)) * width;
    const norm = (v - min) / (max - min || 1);
    const y = height - Math.max(0, Math.min(1, norm)) * height;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = strokeColor;
  ctx.lineWidth = 1.8;
  ctx.stroke();
}

function drawGraphs() {
  if (!trajectoryData.length) return;

  // 1. Altitude Graph
  if (contexts.altitude) {
    const w = canvases.altitude.clientWidth;
    const h = canvases.altitude.clientHeight;
    contexts.altitude.clearRect(0, 0, w, h);
    drawGrid(contexts.altitude, w, h, 4);

    // Full trajectory outline in faint line
    drawSeries(
      contexts.altitude,
      trajectoryData.map(d => d.z),
      0,
      Math.max(10000, maxApogee * 1.1),
      "rgba(146, 214, 27, 0.25)"
    );

    // Current flight history
    if (history.length > 1) {
      drawSeries(
        contexts.altitude,
        history.map(d => d.z),
        0,
        Math.max(10000, maxApogee * 1.1),
        "#92d61b"
      );
    }
  }

  // 2. Accelerometer Graph
  if (contexts.accel) {
    const w = canvases.accel.clientWidth;
    const h = canvases.accel.clientHeight;
    contexts.accel.clearRect(0, 0, w, h);
    drawGrid(contexts.accel, w, h, 4);
    drawSeries(
      contexts.accel,
      history.map(d => Math.min(150, d.accelSensor)),
      0,
      120,
      "#92d61b"
    );
  }

  // 3. Gyroscope Graph
  if (contexts.gyro) {
    const w = canvases.gyro.clientWidth;
    const h = canvases.gyro.clientHeight;
    contexts.gyro.clearRect(0, 0, w, h);
    drawGrid(contexts.gyro, w, h, 4);
    drawSeries(
      contexts.gyro,
      history.map(d => d.gyroSensor),
      150,
      270,
      "#92d61b"
    );
  }

  // 4. Position Graph
  if (contexts.position) {
    const w = canvases.position.clientWidth;
    const h = canvases.position.clientHeight;
    contexts.position.clearRect(0, 0, w, h);
    drawGrid(contexts.position, w, h, 4);
    const targetX = currentMissionParams.targetDist * 1000;
    drawSeries(
      contexts.position,
      history.map(d => d.x),
      0,
      Math.max(targetX * 1.05, 20000),
      "#92d61b"
    );
  }
}

/* =========================================================
   3D MUNITION DIGITAL TWIN (AFT SPIN + DESPUN CANARDS)
========================================================= */
const twinContainer = $("twinContainer");
const twinScene = new THREE.Scene();
twinScene.background = new THREE.Color(0x040807);

const twinCamera = new THREE.PerspectiveCamera(50, 1, 0.1, 50000);
twinCamera.position.set(-6, 3.8, 8);

const twinRenderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
twinRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
twinRenderer.outputColorSpace = THREE.SRGBColorSpace;
twinContainer.appendChild(twinRenderer.domElement);

// Lighting
twinScene.add(new THREE.AmbientLight(0xffffff, 1.3));
const dirLight = new THREE.DirectionalLight(0xffffff, 2.2);
dirLight.position.set(6, 12, 8);
twinScene.add(dirLight);

const rimLight = new THREE.DirectionalLight(0x92d61b, 0.8);
rimLight.position.set(-8, -4, -6);
twinScene.add(rimLight);

// Master munition group
const twin = new THREE.Group();

/* =========================================================
   10-COMPONENT PGK CAD ARCHITECTURE (SIH26098 / M1156 SPEC)
   MATCHING HIGH-FIDELITY CUTAWAY REFERENCE (media_1789672062323.jpg)
========================================================= */
const COMPONENT_DATA = {
  all: {
    badge: "SYSTEM ARCHITECTURE",
    phase: "ALL FLIGHT PHASES",
    title: "155MM PRECISION GUIDANCE KIT",
    material: "PEEK Radome / 7075-T6 Al / 4340 Steel",
    spec: "MIL-STD-333 / STANAG 4369",
    desc: "Autonomous bolt-on kit that converts unguided 155mm shells into precision strike munitions (CEP < 5m) via roll-steering canards."
  },
  casing: {
    badge: "01 / STRUCTURAL AIRFRAME",
    phase: "LAUNCH TO IMPACT",
    title: "ANODIZED ALUMINUM HOUSING",
    material: "Forged 7075-T6 Aluminum / Hard Anodize",
    spec: "Survives 15,000g Setback & 250 Hz Body Spin",
    desc: "Machined aerodynamic shell that shields internal electronics, absorbs 15,000g launch shock, and seals internal bays."
  },
  nosecap: {
    badge: "02 / ENVIRONMENTAL FAIRING",
    phase: "PRE-LAUNCH & BOOST",
    title: "NOSE CAP & INDUCTIVE SETTER",
    material: "High-Temp PEEK Polymer / Copper Loop",
    spec: "STANAG 4369 (EPIAFS Standard)",
    desc: "RF-transparent PEEK radome with copper coil for contactless pre-launch mission and GPS coordinate upload."
  },
  gpshob: {
    badge: "03 / RF & PROXIMITY SENSING",
    phase: "MIDCOURSE TO TERMINAL",
    title: "GPS ANTENNA & HOB RADAR",
    material: "Taconic RF-60TC Ceramic (εr=6.15)",
    spec: "NavIC L1/L5 & GPS / 24 GHz FMCW Radar",
    desc: "Despun patch antenna tracking NavIC/GPS signals, paired with a 24 GHz FMCW radar triggering airburst at 7m altitude."
  },
  canards: {
    badge: "04 / FLIGHT CONTROL SURFACES",
    phase: "POST-APOGEE GUIDANCE",
    title: "4 FIXED CRUCIFORM CANARDS",
    material: "7075-T6 Aluminum / Titanium Race",
    spec: "Lift: 50–220 N / Roll Slew: >1,200°/s",
    desc: "Four aerodynamic blades roll-steered by the motor to direct lift and guide the shell onto target with <5m accuracy."
  },
  geu: {
    badge: "05 / EMBEDDED COMPUTING BRAIN",
    phase: "CONTINUOUS (100 HZ)",
    title: "3-PCB GEU FLIGHT COMPUTER",
    material: "FR-4 Glass-Epoxy / Polyurethane Potting",
    spec: "15-State ESKF @ 100 Hz / TinyML Diagnostics",
    desc: "Triple-PCB stack running a 100 Hz Kalman filter for trajectory guidance, 8-MEMS sensor fusion, and AI health monitoring."
  },
  geartrain: {
    badge: "06 / STEERING ACTUATION",
    phase: "APOGEE TO TERMINAL",
    title: "MINIATURE MOTOR & GEAR TRAIN",
    material: "Coreless DC Motor / 4140 Hardened Gears",
    spec: "18:1 Spur Reduction / 1.2 N·m Peak Torque",
    desc: "Coreless DC motor with 18:1 reduction gears that applies active torque and braking against 250 Hz shell spin to index canards."
  },
  safearm: {
    badge: "07 / MECHANICAL SAFETY INTERLOCK",
    phase: "BARREL EXIT & ARMING",
    title: "SAFE & ARM (S&A) MECHANISM",
    material: "Stainless Steel / Beryllium Copper Springs",
    spec: "MIL-STD-1316E (>10,000g Shock + >150 Hz Spin)",
    desc: "Mechanical barrier keeping detonator out-of-line until 10,000g setback and 150 Hz spin unlock it past 500m for safe separation."
  },
  booster: {
    badge: "08 / ENERGETIC COUPLING",
    phase: "BURST INITIATION",
    title: "MOFA BOOSTER CANISTER",
    material: "Deep-Drawn Brass C26000 Canister",
    spec: "MOFA Standard Booster Well / MIL-STD-333",
    desc: "Precision-drawn brass canister that relays ignition pulse from the Safe & Arm channel to reliably detonate the main shell filler."
  },
  threadbase: {
    badge: "09 / PROJECTILE INTERFACE",
    phase: "STRUCTURAL MOUNTING",
    title: "THREADED BASE PLUG",
    material: "Forged 4340 Alloy Steel / Bronze Coating",
    spec: "MIL-STD-333 NATO 2-inch 12-UNS-2B Interface",
    desc: "Heavy alloy steel plug with standard NATO 2-inch threads, screwing directly into standard 155mm shell cavities with zero gun mods."
  }
};

// 1. Cutaway Housing (Anodized Aluminum) & View Variants
const compCasing = new THREE.Group();

const casingMat = new THREE.MeshStandardMaterial({
  color: 0x272f33,
  roughness: 0.35,
  metalness: 0.72,
  side: THREE.DoubleSide
});

const cutWallMat = new THREE.MeshStandardMaterial({
  color: 0x9fb0b8,
  roughness: 0.28,
  metalness: 0.85,
  side: THREE.DoubleSide
});

const casingGhostMat = new THREE.MeshStandardMaterial({
  color: 0x3d5059,
  roughness: 0.2,
  metalness: 0.5,
  transparent: true,
  opacity: 0.22,
  side: THREE.DoubleSide
});

// Cutaway sector parameters (leaving ~115 deg cutaway window on top)
const CUT_START = -Math.PI * 0.32;
const CUT_LEN = Math.PI * 1.36;

// Cutaway outer cylindrical shell
const casingCutMesh = new THREE.Mesh(
  new THREE.CylinderGeometry(0.415, 0.415, 2.65, 32, 1, true, CUT_START, CUT_LEN),
  casingMat
);
casingCutMesh.rotation.z = Math.PI / 2;
casingCutMesh.position.x = 0.05;
compCasing.add(casingCutMesh);

// Cutaway inner cylindrical wall
const casingInnerMesh = new THREE.Mesh(
  new THREE.CylinderGeometry(0.375, 0.375, 2.65, 32, 1, true, CUT_START, CUT_LEN),
  casingMat
);
casingInnerMesh.rotation.z = Math.PI / 2;
casingInnerMesh.position.x = 0.05;
compCasing.add(casingInnerMesh);

// Cut wall edge strips (showing wall thickness at cut borders)
for (const ang of [CUT_START, CUT_START + CUT_LEN]) {
  const edgePlane = new THREE.Mesh(new THREE.PlaneGeometry(0.040, 2.65), cutWallMat);
  edgePlane.rotation.z = Math.PI / 2;
  edgePlane.rotation.x = ang;
  const rMid = 0.395;
  edgePlane.position.set(0.05, Math.sin(ang) * rMid, Math.cos(ang) * rMid);
  compCasing.add(edgePlane);
}

// Cutaway forward ogive taper
const ogiveCutMesh = new THREE.Mesh(
  new THREE.CylinderGeometry(0.23, 0.415, 0.55, 28, 1, true, CUT_START, CUT_LEN),
  casingMat
);
ogiveCutMesh.rotation.z = -Math.PI / 2;
ogiveCutMesh.position.x = 1.65;
compCasing.add(ogiveCutMesh);

// Solid shells (for Solid Mode)
const casingSolid = new THREE.Mesh(
  new THREE.CylinderGeometry(0.415, 0.415, 2.65, 32),
  casingMat
);
casingSolid.rotation.z = Math.PI / 2;
casingSolid.position.x = 0.05;
casingSolid.visible = false;
compCasing.add(casingSolid);

const ogiveSolid = new THREE.Mesh(
  new THREE.CylinderGeometry(0.23, 0.415, 0.55, 28),
  casingMat
);
ogiveSolid.rotation.z = -Math.PI / 2;
ogiveSolid.position.x = 1.65;
ogiveSolid.visible = false;
compCasing.add(ogiveSolid);

// Ghost shells (for Ghost Mode)
const casingGhost = new THREE.Mesh(
  new THREE.CylinderGeometry(0.415, 0.415, 2.65, 32),
  casingGhostMat
);
casingGhost.rotation.z = Math.PI / 2;
casingGhost.position.x = 0.05;
casingGhost.visible = false;
compCasing.add(casingGhost);

const ogiveGhost = new THREE.Mesh(
  new THREE.CylinderGeometry(0.23, 0.415, 0.55, 28),
  casingGhostMat
);
ogiveGhost.rotation.z = -Math.PI / 2;
ogiveGhost.position.x = 1.65;
ogiveGhost.visible = false;
compCasing.add(ogiveGhost);

// Internal dividing bulkheads
[-0.75, -0.22, 0.45, 1.25].forEach(bx => {
  const bh = new THREE.Mesh(
    new THREE.CylinderGeometry(0.375, 0.375, 0.035, 24, 1, false, CUT_START, CUT_LEN),
    cutWallMat
  );
  bh.rotation.z = Math.PI / 2;
  bh.position.x = bx;
  compCasing.add(bh);
});
twin.add(compCasing);

// 2. Cover / Nose Cap & Inductive Setter
const compNoseCap = new THREE.Group();
const capMesh = new THREE.Mesh(
  new THREE.ConeGeometry(0.23, 0.52, 24, 1, false, CUT_START, CUT_LEN),
  new THREE.MeshStandardMaterial({ color: 0x1a1f1e, roughness: 0.32, metalness: 0.15, side: THREE.DoubleSide })
);
capMesh.rotation.z = -Math.PI / 2;
capMesh.position.x = 2.18;
compNoseCap.add(capMesh);

const setterInductionCoil = new THREE.Mesh(
  new THREE.TorusGeometry(0.12, 0.016, 8, 24),
  new THREE.MeshStandardMaterial({ color: 0xc87533, roughness: 0.25, metalness: 0.90 })
);
setterInductionCoil.rotation.y = Math.PI / 2;
setterInductionCoil.position.x = 2.36;
compNoseCap.add(setterInductionCoil);
twin.add(compNoseCap);

// 3. GPS Antenna Module & Receiver Electronics
const compGpsHob = new THREE.Group();

// Dark antenna puck dome
const gpsPuck = new THREE.Mesh(
  new THREE.CylinderGeometry(0.18, 0.18, 0.14, 24),
  new THREE.MeshStandardMaterial({ color: 0x1b2022, roughness: 0.4, metalness: 0.4 })
);
gpsPuck.rotation.z = Math.PI / 2;
gpsPuck.position.x = 2.05;
compGpsHob.add(gpsPuck);

// Taconic RF ceramic substrate disk
const gpsSubstrate = new THREE.Mesh(
  new THREE.CylinderGeometry(0.23, 0.23, 0.035, 24),
  new THREE.MeshStandardMaterial({ color: 0x184832, roughness: 0.4, metalness: 0.3 })
);
gpsSubstrate.rotation.z = Math.PI / 2;
gpsSubstrate.position.x = 1.94;
compGpsHob.add(gpsSubstrate);

// Gold perimeter ground ring
const gpsGoldRing = new THREE.Mesh(
  new THREE.TorusGeometry(0.228, 0.008, 6, 24),
  new THREE.MeshStandardMaterial({ color: 0xcfa53e, roughness: 0.3, metalness: 0.9 })
);
gpsGoldRing.rotation.y = Math.PI / 2;
gpsGoldRing.position.x = 1.94;
compGpsHob.add(gpsGoldRing);

// Shielded RF LNA receiver module enclosure
const rfBox = new THREE.Mesh(
  new THREE.BoxGeometry(0.04, 0.12, 0.12),
  new THREE.MeshStandardMaterial({ color: 0xc8d2d6, roughness: 0.2, metalness: 0.88 })
);
rfBox.position.set(1.90, 0, 0);
compGpsHob.add(rfBox);

// FMCW Height-of-burst radar sensor aperture
const hobSensorAperture = new THREE.Mesh(
  new THREE.CylinderGeometry(0.04, 0.04, 0.08, 16),
  new THREE.MeshBasicMaterial({ color: 0x92d61b })
);
hobSensorAperture.rotation.y = Math.PI / 2;
hobSensorAperture.position.set(1.90, -0.16, 0);
compGpsHob.add(hobSensorAperture);
twin.add(compGpsHob);

// 4. Canard Actuation Assembly (CAA) & 4 Fixed Canards
const compCanards = new THREE.Group();

// Titanium rotating drive collar / bearing sleeve
const canardCollar = new THREE.Mesh(
  new THREE.CylinderGeometry(0.378, 0.378, 0.20, 24),
  new THREE.MeshStandardMaterial({ color: 0x485356, roughness: 0.35, metalness: 0.8 })
);
canardCollar.rotation.z = Math.PI / 2;
canardCollar.position.x = 1.48;
compCanards.add(canardCollar);

const canards = [];
const canardFinMat = new THREE.MeshStandardMaterial({
  color: 0x525e62,
  roughness: 0.30,
  metalness: 0.78
});

for (let i = 0; i < 4; i++) {
  const finPivot = new THREE.Group();
  const finAngle = (i * Math.PI) / 2;
  finPivot.rotation.x = finAngle;
  finPivot.position.x = 1.48;

  const finShape = new THREE.Shape();
  finShape.moveTo(-0.16, 0.36);
  finShape.lineTo(0.18, 0.36);
  finShape.lineTo(0.09, 0.94);
  finShape.lineTo(-0.09, 0.94);
  finShape.closePath();

  const finGeom = new THREE.ExtrudeGeometry(finShape, {
    depth: 0.022,
    bevelEnabled: true,
    bevelSegments: 2,
    steps: 1,
    bevelSize: 0.008,
    bevelThickness: 0.008
  });
  finGeom.center();

  const finMesh = new THREE.Mesh(finGeom, canardFinMat);
  finMesh.position.set(0, 0.65, 0);
  finPivot.add(finMesh);

  compCanards.add(finPivot);
  canards.push(finPivot);
}
twin.add(compCanards);

// 5. Guidance Electronic Unit (GEU) / 3 Stacked Glass-Epoxy PCBs & MCUs
const compGeu = new THREE.Group();

const pcbMat = new THREE.MeshStandardMaterial({
  color: 0x165b32,
  roughness: 0.35,
  metalness: 0.2
});
const goldTraceMat = new THREE.MeshStandardMaterial({
  color: 0xcda840,
  roughness: 0.3,
  metalness: 0.88
});
const mcuMat = new THREE.MeshStandardMaterial({
  color: 0x0e1311,
  roughness: 0.22,
  metalness: 0.85
});
const pinMat = new THREE.MeshStandardMaterial({
  color: 0xd6dee1,
  roughness: 0.2,
  metalness: 0.92
});
const smdCapMat = new THREE.MeshStandardMaterial({
  color: 0xc79f48,
  roughness: 0.4,
  metalness: 0.3
});
const crystalMat = new THREE.MeshStandardMaterial({
  color: 0xd0d8db,
  roughness: 0.15,
  metalness: 0.95
});

// Standoff spacers (4 brass pillars)
for (let s = 0; s < 4; s++) {
  const sAngle = (s * Math.PI) / 2 + Math.PI / 4;
  const standoff = new THREE.Mesh(
    new THREE.CylinderGeometry(0.016, 0.016, 0.58, 8),
    goldTraceMat
  );
  standoff.rotation.z = Math.PI / 2;
  standoff.position.set(0.80, Math.cos(sAngle) * 0.26, Math.sin(sAngle) * 0.26);
  compGeu.add(standoff);
}

// 3 Stacked Circular PCBs at X = 1.05, 0.80, 0.55
const pcbPositions = [1.05, 0.80, 0.55];
pcbPositions.forEach((px, idx) => {
  // Circular PCB base
  const pcbDisc = new THREE.Mesh(
    new THREE.CylinderGeometry(0.35, 0.35, 0.022, 28),
    pcbMat
  );
  pcbDisc.rotation.z = Math.PI / 2;
  pcbDisc.position.x = px;
  compGeu.add(pcbDisc);

  // Gold outer ground perimeter trace
  const goldRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.345, 0.007, 6, 28),
    goldTraceMat
  );
  goldRing.rotation.y = Math.PI / 2;
  goldRing.position.x = px;
  compGeu.add(goldRing);

  if (idx === 0) {
    // Board 1: Navigation DSP/FPGA MCU + QFP pins
    const mainMcu = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.13, 0.13), mcuMat);
    mainMcu.position.set(px + 0.018, 0.05, 0);
    compGeu.add(mainMcu);

    // Silver pin lead arrays along edges
    const pinBar1 = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.14, 0.015), pinMat);
    pinBar1.position.set(px + 0.015, 0.05, 0.075);
    compGeu.add(pinBar1);
    const pinBar2 = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.14, 0.015), pinMat);
    pinBar2.position.set(px + 0.015, 0.05, -0.075);
    compGeu.add(pinBar2);

    // Ceramic SMD capacitors
    for (let c = 0; c < 5; c++) {
      const cap = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.025, 0.04), smdCapMat);
      cap.position.set(px + 0.015, -0.10 + c * 0.05, 0.06);
      compGeu.add(cap);
    }
  } else if (idx === 1) {
    // Board 2: Flight Computer MCU 1 & MCU 2
    const mcu1 = new THREE.Mesh(new THREE.BoxGeometry(0.022, 0.09, 0.09), mcuMat);
    mcu1.position.set(px + 0.016, 0.08, 0.06);
    compGeu.add(mcu1);

    const mcu2 = new THREE.Mesh(new THREE.BoxGeometry(0.022, 0.08, 0.08), mcuMat);
    mcu2.position.set(px + 0.016, -0.07, -0.04);
    compGeu.add(mcu2);

    // Quartz crystal can
    const crystal = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.04, 0.07), crystalMat);
    crystal.position.set(px + 0.016, 0.14, -0.08);
    compGeu.add(crystal);
  } else {
    // Board 3: TinyML Hardware Health & 8-MEMS Sensor Array
    const tinyChip = new THREE.Mesh(new THREE.BoxGeometry(0.024, 0.10, 0.10), mcuMat);
    tinyChip.position.set(px + 0.016, 0, 0);
    compGeu.add(tinyChip);

    // 8 MEMS sensor nodes around perimeter
    for (let m = 0; m < 8; m++) {
      const mAng = (m / 8) * Math.PI * 2;
      const memsNode = new THREE.Mesh(
        new THREE.BoxGeometry(0.018, 0.035, 0.035),
        new THREE.MeshStandardMaterial({ color: 0x222b29, roughness: 0.3, metalness: 0.8 })
      );
      memsNode.position.set(px + 0.015, Math.cos(mAng) * 0.22, Math.sin(mAng) * 0.22);
      compGeu.add(memsNode);
    }
  }
});
twin.add(compGeu);

// 6. Canard Actuation Miniature Gear Train & Motor
const compGearTrain = new THREE.Group();

// Dividing Bulkhead Platform
const gearBulkhead = new THREE.Mesh(
  new THREE.CylinderGeometry(0.36, 0.36, 0.030, 24),
  new THREE.MeshStandardMaterial({ color: 0x424e4c, roughness: 0.4, metalness: 0.7 })
);
gearBulkhead.rotation.z = Math.PI / 2;
gearBulkhead.position.x = -0.08;
compGearTrain.add(gearBulkhead);

// Horizontal brushed cylindrical DC Motor
const motorMesh = new THREE.Mesh(
  new THREE.CylinderGeometry(0.095, 0.095, 0.26, 20),
  new THREE.MeshStandardMaterial({ color: 0x909ca4, roughness: 0.28, metalness: 0.84 })
);
motorMesh.rotation.y = Math.PI / 2;
motorMesh.position.set(0.06, 0.11, 0);
compGearTrain.add(motorMesh);

// Motor output shaft
const motorShaft = new THREE.Mesh(
  new THREE.CylinderGeometry(0.018, 0.018, 0.12, 12),
  new THREE.MeshStandardMaterial({ color: 0xd2dbe0, roughness: 0.2, metalness: 0.95 })
);
motorShaft.rotation.y = Math.PI / 2;
motorShaft.position.set(0.06, 0.11, 0.18);
compGearTrain.add(motorShaft);

// Drive Pinion Gear
const pinionGear = new THREE.Mesh(
  new THREE.CylinderGeometry(0.042, 0.042, 0.035, 16),
  new THREE.MeshStandardMaterial({ color: 0x68747a, roughness: 0.3, metalness: 0.82 })
);
pinionGear.position.set(0.06, 0.11, 0.22);
compGearTrain.add(pinionGear);

// Intermediate Compound Reduction Spur Gear
const spurGear = new THREE.Mesh(
  new THREE.CylinderGeometry(0.092, 0.092, 0.030, 24),
  new THREE.MeshStandardMaterial({ color: 0x88959c, roughness: 0.3, metalness: 0.85 })
);
spurGear.position.set(0.06, -0.02, 0.22);
compGearTrain.add(spurGear);

// Driven Sector Gear
const drivenGear = new THREE.Mesh(
  new THREE.CylinderGeometry(0.082, 0.082, 0.028, 20),
  new THREE.MeshStandardMaterial({ color: 0x7b888e, roughness: 0.3, metalness: 0.82 })
);
drivenGear.position.set(0.12, -0.02, 0.14);
compGearTrain.add(drivenGear);

// Dual Machined Aluminum Pillow Blocks / Bearing Supports
[-0.04, 0.16].forEach(px => {
  const bracket = new THREE.Mesh(
    new THREE.BoxGeometry(0.03, 0.18, 0.08),
    new THREE.MeshStandardMaterial({ color: 0x7a878e, roughness: 0.35, metalness: 0.8 })
  );
  bracket.position.set(px, 0.02, 0.22);
  compGearTrain.add(bracket);

  // Hex bolt heads
  const bolt = new THREE.Mesh(
    new THREE.CylinderGeometry(0.012, 0.012, 0.01, 6),
    new THREE.MeshStandardMaterial({ color: 0xb5c1c7, roughness: 0.2, metalness: 0.9 })
  );
  bolt.rotation.x = Math.PI / 2;
  bolt.position.set(px, 0.08, 0.265);
  compGearTrain.add(bolt);
});
twin.add(compGearTrain);

// 7. Safe & Arm (S&A) Mechanism & Detonator Train (MT62 / MIL-STD-1316E)
const compSafeArm = new THREE.Group();

// Titanium base mounting plate
const saBasePlate = new THREE.Mesh(
  new THREE.CylinderGeometry(0.35, 0.35, 0.030, 24),
  new THREE.MeshStandardMaterial({ color: 0x667276, roughness: 0.35, metalness: 0.8 })
);
saBasePlate.rotation.z = Math.PI / 2;
saBasePlate.position.x = -0.64;
compSafeArm.add(saBasePlate);

// Stainless steel rotary barrier disc
const saRotorDisc = new THREE.Mesh(
  new THREE.CylinderGeometry(0.30, 0.30, 0.035, 24),
  new THREE.MeshStandardMaterial({ color: 0x828e94, roughness: 0.28, metalness: 0.88 })
);
saRotorDisc.rotation.z = Math.PI / 2;
saRotorDisc.position.x = -0.52;
compSafeArm.add(saRotorDisc);

// Red Arming Rotor / Centrifugal Slider Block (Safe Out-of-Line position)
const saRedSlider = new THREE.Mesh(
  new THREE.BoxGeometry(0.08, 0.075, 0.20),
  new THREE.MeshStandardMaterial({ color: 0xd83426, roughness: 0.3, metalness: 0.35 })
);
saRedSlider.position.set(-0.47, 0.06, 0.04);
compSafeArm.add(saRedSlider);

// Dual Emerald Green Setback Guide Pins
[-0.08, 0.08].forEach(pz => {
  const pin = new THREE.Mesh(
    new THREE.CylinderGeometry(0.020, 0.020, 0.16, 12),
    new THREE.MeshStandardMaterial({ color: 0x28a745, roughness: 0.3, metalness: 0.55 })
  );
  pin.position.set(-0.54, -0.10, pz);
  compSafeArm.add(pin);
});

// Polished Silver Flight Locking Interlock Pins
const silverPin = new THREE.Mesh(
  new THREE.CylinderGeometry(0.016, 0.016, 0.13, 12),
  new THREE.MeshStandardMaterial({ color: 0xdbe2e5, roughness: 0.2, metalness: 0.92 })
);
silverPin.rotation.x = Math.PI / 2;
silverPin.position.set(-0.54, 0.12, 0.06);
compSafeArm.add(silverPin);

// Red Detonator Train Conduit (Central Safe Channel leading down to booster)
const detonatorTube = new THREE.Mesh(
  new THREE.CylinderGeometry(0.042, 0.042, 0.16, 16),
  new THREE.MeshStandardMaterial({ color: 0xb6291c, roughness: 0.35, metalness: 0.4 })
);
detonatorTube.rotation.z = Math.PI / 2;
detonatorTube.position.set(-0.73, 0, 0);
compSafeArm.add(detonatorTube);
twin.add(compSafeArm);

// 8. MOFA Booster-Cased Booster Charge Pellet
const compBooster = new THREE.Group();

const brassMat = new THREE.MeshStandardMaterial({
  color: 0xd4af37,
  roughness: 0.20,
  metalness: 0.88
});

// Large polished brass cylindrical canister
const boosterBody = new THREE.Mesh(
  new THREE.CylinderGeometry(0.26, 0.26, 0.38, 28),
  brassMat
);
boosterBody.rotation.z = Math.PI / 2;
boosterBody.position.x = -1.02;
compBooster.add(boosterBody);

// Stepped golden neck shoulder
const boosterNeck = new THREE.Mesh(
  new THREE.CylinderGeometry(0.13, 0.13, 0.07, 24),
  brassMat
);
boosterNeck.rotation.z = Math.PI / 2;
boosterNeck.position.x = -0.80;
compBooster.add(boosterNeck);
twin.add(compBooster);

// 9. Threaded Base Plug (NATO 2-inch 12-UNS)
const compThreadedBase = new THREE.Group();

const threadCylinder = new THREE.Mesh(
  new THREE.CylinderGeometry(0.415, 0.415, 0.38, 24),
  new THREE.MeshStandardMaterial({ color: 0x2c3432, roughness: 0.4, metalness: 0.75 })
);
threadCylinder.rotation.z = Math.PI / 2;
threadCylinder.position.x = -1.45;
compThreadedBase.add(threadCylinder);

// 6 precision bronze-coated external screw threads
for (let th = -2; th <= 3; th++) {
  const threadRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.414, 0.012, 8, 24),
    new THREE.MeshStandardMaterial({ color: 0xb68739, roughness: 0.30, metalness: 0.85 })
  );
  threadRing.rotation.y = Math.PI / 2;
  threadRing.position.x = -1.45 + th * 0.05;
  compThreadedBase.add(threadRing);
}

// Spanner wrench engagement flat notches
[-0.20, 0.20].forEach(py => {
  const notch = new THREE.Mesh(
    new THREE.BoxGeometry(0.12, 0.04, 0.04),
    new THREE.MeshStandardMaterial({ color: 0x181e1d })
  );
  notch.position.set(-1.60, py, 0);
  compThreadedBase.add(notch);
});
twin.add(compThreadedBase);

// Full 155mm Artillery Shell Body (Aft Section)
const aftShellGroup = new THREE.Group();
const steelMaterial = new THREE.MeshStandardMaterial({
  color: 0x3d4944,
  roughness: 0.45,
  metalness: 0.65
});

const boatTail = new THREE.Mesh(
  new THREE.CylinderGeometry(0.42, 0.38, 0.6, 24),
  steelMaterial
);
boatTail.rotation.z = Math.PI / 2;
boatTail.position.x = -2.85;
aftShellGroup.add(boatTail);

const mainBody = new THREE.Mesh(
  new THREE.CylinderGeometry(0.42, 0.42, 1.4, 24),
  steelMaterial
);
mainBody.rotation.z = Math.PI / 2;
mainBody.position.x = -1.95;
aftShellGroup.add(mainBody);

const copperBand = new THREE.Mesh(
  new THREE.CylinderGeometry(0.428, 0.428, 0.22, 24),
  new THREE.MeshStandardMaterial({ color: 0xc87533, roughness: 0.3, metalness: 0.85 })
);
copperBand.rotation.z = Math.PI / 2;
copperBand.position.x = -2.45;
aftShellGroup.add(copperBand);

twin.add(aftShellGroup);

// Component registration table for exploded view & inspection
const CAD_COMPONENTS = [
  { id: "casing", group: compCasing, base: 0.0, offset: 0.0 },
  { id: "nosecap", group: compNoseCap, base: 0.0, offset: 2.2 },
  { id: "gpshob", group: compGpsHob, base: 0.0, offset: 1.7 },
  { id: "canards", group: compCanards, base: 0.0, offset: 1.2 },
  { id: "geu", group: compGeu, base: 0.0, offset: 0.6 },
  { id: "geartrain", group: compGearTrain, base: 0.0, offset: 0.1 },
  { id: "safearm", group: compSafeArm, base: 0.0, offset: -0.5 },
  { id: "booster", group: compBooster, base: 0.0, offset: -1.0 },
  { id: "threadbase", group: compThreadedBase, base: 0.0, offset: -1.5 }
];

let targetExploded = 0.0;
let currentExploded = 0.0;
let activeCompId = "all";

// High-performance invisible raycasting hit proxies (1 simple cylinder per component group)
const hitProxies = [];
const hitProxyMat = new THREE.MeshBasicMaterial({
  transparent: true,
  opacity: 0.0,
  depthWrite: false
});

const HIT_PROXY_SPECS = {
  casing:     { x: 0.50, r: 0.46, len: 2.20 },
  nosecap:    { x: 2.25, r: 0.28, len: 0.55 },
  gpshob:     { x: 1.95, r: 0.28, len: 0.40 },
  canards:    { x: 1.48, r: 0.96, len: 0.45 },
  geu:        { x: 0.80, r: 0.42, len: 0.60 },
  geartrain:  { x: 0.25, r: 0.42, len: 0.50 },
  safearm:    { x: -0.40, r: 0.42, len: 0.55 },
  booster:    { x: -0.98, r: 0.44, len: 0.55 },
  threadbase: { x: -1.45, r: 0.46, len: 0.45 }
};

CAD_COMPONENTS.forEach(c => {
  const spec = HIT_PROXY_SPECS[c.id];
  if (!spec) return;
  const geom = new THREE.CylinderGeometry(spec.r, spec.r, spec.len, 12);
  geom.rotateZ(Math.PI / 2);
  const proxy = new THREE.Mesh(geom, hitProxyMat);
  proxy.position.x = spec.x;
  proxy.userData = { compId: c.id };
  c.group.add(proxy);
  hitProxies.push(proxy);
});

// Smooth 60/120 FPS camera orbit & tracking state
let twinOrbitMode = true; // Orbit CAD inspection by default
let twinDragging = false;
let twinPrevX = 0, twinPrevY = 0;
let twinRotYaw = -0.6, twinRotPitch = 0.32;
let targetYaw = -0.6, targetPitch = 0.32;
let twinDistance = 7.5;
let targetDistance = 7.5;
let twinPinchStartDist = 0;
let hasDraggedSignificant = false;
let pointerDownX = 0, pointerDownY = 0;

const currentShellPos = new THREE.Vector3(0, 0, 0);
const currentShellDir = new THREE.Vector3(1, 0, 0);

// High-performance RAF pointer tracking (0 forced reflows during mousemove)
let isPointerInside = false;
let pendingPointer = null;
let containerRect = null;
const twinRaycaster = new THREE.Raycaster();

// Supersonic Mach condensation cone effect
const machConeMat = new THREE.MeshBasicMaterial({
  color: 0x92d61b,
  transparent: true,
  opacity: 0.12,
  wireframe: true
});
const machCone = new THREE.Mesh(
  new THREE.ConeGeometry(1.6, 2.8, 16, 1, true),
  machConeMat
);
machCone.rotation.z = Math.PI / 2;
machCone.position.x = 0.5;
twin.add(machCone);

// Tactical targeting highlight ring for selected 3D component
const highlightRing = new THREE.Mesh(
  new THREE.RingGeometry(0.50, 0.55, 36),
  new THREE.MeshBasicMaterial({
    color: 0x92d61b,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.88
  })
);
highlightRing.rotation.y = Math.PI / 2;
highlightRing.visible = false;
twin.add(highlightRing);

twinScene.add(twin);

// Ground grid in Digital Twin viewport
const twinGrid = new THREE.GridHelper(500, 100, 0x25302c, 0x111917);
twinGrid.position.y = -3;
twinScene.add(twinGrid);

// Trail lines
const twinTrailPoints = [];
const twinTrail = new THREE.Line(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({ color: 0x92d61b, transparent: true, opacity: 0.8 })
);
twinScene.add(twinTrail);

function resizeTwin() {
  const w = twinContainer.clientWidth;
  const h = twinContainer.clientHeight;
  if (!w || !h) return;
  twinCamera.aspect = w / h;
  twinCamera.updateProjectionMatrix();
  twinRenderer.setSize(w, h, false);
  if (twinContainer) containerRect = twinContainer.getBoundingClientRect();
}

function renderTwin() {
  requestAnimationFrame(renderTwin);

  // Smooth exploded separation animation
  currentExploded += (targetExploded - currentExploded) * 0.12;
  CAD_COMPONENTS.forEach(c => {
    c.group.position.x = c.base + c.offset * currentExploded;
  });
  aftShellGroup.position.x = -currentExploded * 2.2;

  // Real-time smooth camera update on EVERY animation frame (60/120 FPS)
  updateTwinCamera(currentShellPos, currentShellDir);

  // Update 3D targeting highlight ring on selected component
  if (activeCompId !== "all") {
    const activeItem = CAD_COMPONENTS.find(c => c.id === activeCompId);
    if (activeItem) {
      highlightRing.visible = true;
      highlightRing.position.x = activeItem.group.position.x;
      const pulse = 1.0 + Math.sin(Date.now() * 0.007) * 0.04;
      highlightRing.scale.set(pulse, pulse, pulse);
    } else {
      highlightRing.visible = false;
    }
  } else {
    highlightRing.visible = false;
  }

  // Fast RAF raycast test on 9 low-poly bounding hit proxies (0.005 ms)
  if (isPointerInside && pendingPointer && !twinDragging) {
    twinRaycaster.setFromCamera(pendingPointer, twinCamera);
    const intersects = twinRaycaster.intersectObjects(hitProxies, false);
    if (intersects.length > 0) {
      twinContainer.style.cursor = "pointer";
      showHoverTooltip(intersects[0].object.userData.compId);
    } else {
      twinContainer.style.cursor = twinOrbitMode ? "grab" : "default";
      hideHoverTooltip();
    }
  }

  twinRenderer.render(twinScene, twinCamera);
}

/* =========================================================
   TOUCH & FREE ORBIT CONTROLS FOR HERO 3D TWIN
========================================================= */
const camModeBtn = $("camModeBtn");
const hudComponent = $("hudComponent");

function updateTwinCamera(scaledPos, direction) {
  if (twinOrbitMode) {
    twinRotYaw += (targetYaw - twinRotYaw) * 0.22;
    twinRotPitch += (targetPitch - twinRotPitch) * 0.22;
    twinDistance += (targetDistance - twinDistance) * 0.22;

    const target = (scaledPos || new THREE.Vector3(0, 0, 0)).clone().add(new THREE.Vector3(0.4, 0, 0));
    const cx = Math.cos(twinRotPitch) * Math.sin(twinRotYaw) * twinDistance;
    const cy = Math.sin(twinRotPitch) * twinDistance;
    const cz = Math.cos(twinRotPitch) * Math.cos(twinRotYaw) * twinDistance;

    twinCamera.position.set(target.x + cx, target.y + cy, target.z + cz);
    twinCamera.lookAt(target);
  } else {
    const cameraOffset = (direction || new THREE.Vector3(1, 0, 0)).clone().multiplyScalar(-6.5);
    cameraOffset.y += 2.8;
    const desiredCamera = (scaledPos || new THREE.Vector3(0, 0, 0)).clone().add(cameraOffset);
    twinCamera.position.lerp(desiredCamera, 0.085);

    const lookTarget = (scaledPos || new THREE.Vector3(0, 0, 0)).clone().add((direction || new THREE.Vector3(1, 0, 0)).clone().multiplyScalar(4.0));
    twinCamera.lookAt(lookTarget);
  }
}

function setTwinOrbitMode(active) {
  twinOrbitMode = active;
  if (camModeBtn) {
    if (twinOrbitMode) {
      camModeBtn.textContent = "CAMERA: FREE ORBIT [TOUCH CHASE]";
      camModeBtn.classList.add("active");
      if (hudComponent) hudComponent.textContent = "FREE ORBIT MODE · DRAG TO ROTATE 3D SHELL · PINCH/WHEEL TO ZOOM";
    } else {
      camModeBtn.textContent = "CAMERA: CHASE [TOUCH TO ORBIT]";
      camModeBtn.classList.remove("active");
      if (hudComponent) hudComponent.textContent = "CHASE CAM ACTIVE · TOUCH OR DRAG ANYWHERE TO INSPECT 3D SHELL";
    }
  }
}

function updateTwinContainerRect() {
  if (twinContainer) containerRect = twinContainer.getBoundingClientRect();
}

function setupTwinControls() {
  if (!twinContainer) return;
  updateTwinContainerRect();

  // Single-pointer drag (mouse or 1-finger touch)
  twinContainer.addEventListener("pointerdown", e => {
    twinDragging = true;
    hasDraggedSignificant = false;
    pointerDownX = e.clientX;
    pointerDownY = e.clientY;
    twinPrevX = e.clientX;
    twinPrevY = e.clientY;
    try { twinContainer.setPointerCapture(e.pointerId); } catch(_) {}
    if (!twinOrbitMode) setTwinOrbitMode(true);
  });

  twinContainer.addEventListener("pointermove", e => {
    if (!twinDragging) return;
    const dx = e.clientX - twinPrevX;
    const dy = e.clientY - twinPrevY;
    twinPrevX = e.clientX;
    twinPrevY = e.clientY;

    if (Math.hypot(e.clientX - pointerDownX, e.clientY - pointerDownY) > 4) {
      hasDraggedSignificant = true;
    }

    targetYaw -= dx * 0.007;
    targetPitch -= dy * 0.007;
    targetPitch = Math.max(-1.4, Math.min(1.4, targetPitch));
  });

  const stopTwinDrag = e => {
    twinDragging = false;
    try { twinContainer.releasePointerCapture(e.pointerId); } catch(_) {}
  };
  twinContainer.addEventListener("pointerup", stopTwinDrag);
  twinContainer.addEventListener("pointercancel", stopTwinDrag);

  // Wheel zoom with smooth targetDistance
  twinContainer.addEventListener("wheel", e => {
    e.preventDefault();
    if (!twinOrbitMode) setTwinOrbitMode(true);
    targetDistance += e.deltaY * 0.006;
    targetDistance = Math.max(2.2, Math.min(20, targetDistance));
  }, { passive: false });

  // Touch pinch-to-zoom (2-finger)
  twinContainer.addEventListener("touchstart", e => {
    if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      twinPinchStartDist = Math.hypot(dx, dy);
    }
  }, { passive: true });

  twinContainer.addEventListener("touchmove", e => {
    if (e.touches.length === 2 && twinPinchStartDist > 0) {
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      const dist = Math.hypot(dx, dy);
      const diff = twinPinchStartDist - dist;
      twinPinchStartDist = dist;

      if (!twinOrbitMode) setTwinOrbitMode(true);
      targetDistance += diff * 0.02;
      targetDistance = Math.max(2.2, Math.min(20, targetDistance));
    }
  }, { passive: true });

  // Camera toggle button
  if (camModeBtn) {
    camModeBtn.addEventListener("click", e => {
      e.stopPropagation();
      setTwinOrbitMode(!twinOrbitMode);
    });
  }
}

/* =========================================================
   COMPONENT INSPECTION & EXPLODED VIEW CONTROLLER
========================================================= */
const viewCutawayBtn = $("viewCutawayBtn");
const explodeToggleBtn = $("explodeToggleBtn");
const viewSolidBtn = $("viewSolidBtn");
const viewGhostBtn = $("viewGhostBtn");
const componentDetailCard = $("componentDetailCard");
const closeDetailCardBtn = $("closeDetailCardBtn");
const compBadge = $("compBadge");
const compPhase = $("compPhase");
const compTitle = $("compTitle");
const compMaterial = $("compMaterial");
const compSpec = $("compSpec");
const compDescription = $("compDescription");
const compHowItWorks = $("compHowItWorks");
const compKeySpecs = $("compKeySpecs");
const hudStatusTag = $("hudStatusTag");

let currentViewMode = "cutaway";

function setViewMode(mode) {
  currentViewMode = mode;

  if (viewCutawayBtn) viewCutawayBtn.classList.toggle("active", mode === "cutaway");
  if (explodeToggleBtn) {
    explodeToggleBtn.classList.toggle("active", mode === "exploded");
    explodeToggleBtn.textContent = mode === "exploded" ? "EXPLODED CAD [ON]" : "EXPLODED CAD [OFF]";
  }
  if (viewSolidBtn) viewSolidBtn.classList.toggle("active", mode === "solid");
  if (viewGhostBtn) viewGhostBtn.classList.toggle("active", mode === "ghost");

  // Shell visibilities
  const isCut = (mode === "cutaway" || mode === "exploded");
  casingCutMesh.visible = isCut;
  casingInnerMesh.visible = isCut;
  ogiveCutMesh.visible = isCut;
  capMesh.visible = isCut;

  casingSolid.visible = (mode === "solid");
  ogiveSolid.visible = (mode === "solid");

  casingGhost.visible = (mode === "ghost");
  ogiveGhost.visible = (mode === "ghost");

  if (mode === "exploded") {
    targetExploded = 1.0;
    if (hudComponent) hudComponent.textContent = "EXPLODED CAD VIEW ACTIVE · HOVER OVER ANY PART FOR BRIEF DESCRIPTION";
  } else {
    targetExploded = 0.0;
    if (hudComponent) hudComponent.textContent = `CAD VIEW: ${mode.toUpperCase()} · HOVER OVER ANY PART FOR BRIEF DESCRIPTION`;
  }
}

function setExplodedMode(active) {
  setViewMode(active ? "exploded" : "cutaway");
}

let currentHoveredCompId = null;

function showHoverTooltip(compId) {
  if (!compId || compId === "all") {
    hideHoverTooltip();
    return;
  }
  if (currentHoveredCompId === compId) return; // Prevent DOM thrashing if already displaying this component

  currentHoveredCompId = compId;
  activeCompId = compId;
  const data = COMPONENT_DATA[compId] || COMPONENT_DATA.all;

  if (compBadge) compBadge.textContent = data.badge;
  if (compPhase) compPhase.textContent = data.phase || "ALL FLIGHT PHASES";
  if (compTitle) compTitle.textContent = data.title;
  if (compMaterial) compMaterial.textContent = data.material;
  if (compSpec) compSpec.textContent = data.spec;
  if (compDescription) compDescription.textContent = data.desc;

  if (componentDetailCard) componentDetailCard.classList.remove("hidden");

  document.querySelectorAll(".comp-nav-btn").forEach(btn => {
    btn.classList.toggle("active", btn.dataset.comp === compId);
  });

  if (hudStatusTag) hudStatusTag.textContent = `INSPECTING: ${data.title}`;
}

function hideHoverTooltip() {
  if (currentHoveredCompId === null) return; // Already hidden, prevent redundant DOM writes
  currentHoveredCompId = null;
  activeCompId = "all";

  if (componentDetailCard) componentDetailCard.classList.add("hidden");

  document.querySelectorAll(".comp-nav-btn").forEach(btn => {
    btn.classList.toggle("active", btn.dataset.comp === "all");
  });
  if (hudStatusTag) hudStatusTag.textContent = "155MM ARTICULATED CAD TWIN";
}

function setupComponentInspection() {
  if (viewCutawayBtn) viewCutawayBtn.addEventListener("click", () => setViewMode("cutaway"));
  if (explodeToggleBtn) {
    explodeToggleBtn.addEventListener("click", e => {
      e.stopPropagation();
      setViewMode(currentViewMode === "exploded" ? "cutaway" : "exploded");
    });
  }
  if (viewSolidBtn) viewSolidBtn.addEventListener("click", () => setViewMode("solid"));
  if (viewGhostBtn) viewGhostBtn.addEventListener("click", () => setViewMode("ghost"));

  // Navbar button hover & click events
  document.querySelectorAll(".comp-nav-btn").forEach(btn => {
    btn.addEventListener("pointerenter", () => {
      showHoverTooltip(btn.dataset.comp);
    });
    btn.addEventListener("pointerleave", () => {
      hideHoverTooltip();
    });
    btn.addEventListener("click", () => {
      const compId = btn.dataset.comp;
      if (compId !== "all") {
        if (targetExploded < 0.3) setExplodedMode(true);
        showHoverTooltip(compId);
      } else {
        setExplodedMode(false);
        hideHoverTooltip();
      }
    });
  });

  // Zero-lag pointer tracking over 3D container (0 forced reflows during mousemove)
  updateTwinContainerRect();
  window.addEventListener("resize", updateTwinContainerRect);

  twinContainer.addEventListener("pointerenter", () => {
    updateTwinContainerRect();
    isPointerInside = true;
  });

  twinContainer.addEventListener("pointerleave", () => {
    isPointerInside = false;
    pendingPointer = null;
    twinContainer.style.cursor = twinOrbitMode ? "grab" : "default";
    hideHoverTooltip();
  });

  twinContainer.addEventListener("pointermove", e => {
    if (twinDragging) return;
    if (!containerRect) updateTwinContainerRect();
    pendingPointer = {
      x: ((e.clientX - containerRect.left) / containerRect.width) * 2 - 1,
      y: -((e.clientY - containerRect.top) / containerRect.height) * 2 + 1
    };
    isPointerInside = true;
  });

  // Clicking on a 3D component expands exploded view if closed
  twinContainer.addEventListener("click", e => {
    if (hasDraggedSignificant) return; // User was orbiting, not clicking!
    if (!containerRect) updateTwinContainerRect();

    const clickPtr = new THREE.Vector2(
      ((e.clientX - containerRect.left) / containerRect.width) * 2 - 1,
      -((e.clientY - containerRect.top) / containerRect.height) * 2 + 1
    );

    twinRaycaster.setFromCamera(clickPtr, twinCamera);
    const intersects = twinRaycaster.intersectObjects(hitProxies, false);

    if (intersects.length > 0) {
      const compId = intersects[0].object.userData.compId;
      if (targetExploded < 0.3) setExplodedMode(true);
      showHoverTooltip(compId);
    }
  });
}

/* =========================================================
   3D NAVIGATION / TRAJECTORY VISUALIZER (ORBITAL INSPECTION)
========================================================= */
const navigation3D = $("navigation3D");
let navScene, navCamera, navRenderer;
let navGuidedLine, navUnguidedLine, navMarker, navTargetRing, navInnerRing;
const navGuidedPoints = [];
const navUnguidedPoints = [];

let navDragging = false;
let navPrevX = 0, navPrevY = 0;
let navRotX = -0.4, navRotY = 0.8;
let navDist = 18;

const NAV_SCALE = { x: 0.00065, z: 0.00065, y: 0.00065 };

function initNavigation3D() {
  if (!navigation3D) return;

  navScene = new THREE.Scene();
  navScene.background = new THREE.Color(0x040807);

  navCamera = new THREE.PerspectiveCamera(
    42,
    navigation3D.clientWidth / navigation3D.clientHeight,
    0.1,
    2000
  );

  navRenderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
  navRenderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
  navRenderer.outputColorSpace = THREE.SRGBColorSpace;
  navigation3D.appendChild(navRenderer.domElement);

  // Coordinate grid
  const grid = new THREE.GridHelper(26, 26, 0x39443f, 0x141d1a);
  navScene.add(grid);

  navScene.add(new THREE.AxesHelper(4));

  // Trajectory Lines
  navGuidedLine = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0x92d61b, linewidth: 2 })
  );
  navScene.add(navGuidedLine);

  navUnguidedLine = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0xe05638, transparent: true, opacity: 0.55 })
  );
  navScene.add(navUnguidedLine);

  // Shell Marker
  navMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.16, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x92d61b })
  );
  navScene.add(navMarker);

  // Target CEP Rings at Impact
  const ringMat = new THREE.MeshBasicMaterial({ color: 0x92d61b, wireframe: true });
  navTargetRing = new THREE.Mesh(new THREE.RingGeometry(0.35, 0.40, 24), ringMat);
  navTargetRing.rotation.x = Math.PI / 2;
  navScene.add(navTargetRing);

  const innerRingMat = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true });
  navInnerRing = new THREE.Mesh(new THREE.RingGeometry(0.08, 0.12, 24), innerRingMat);
  navInnerRing.rotation.x = Math.PI / 2;
  navScene.add(navInnerRing);

  rebuild3DReferenceTrajectory();
  resizeNavigation3D();
  updateNavigationCamera();
  setupNavigationControls();
  renderNavigation3D();
}

function rebuild3DReferenceTrajectory() {
  if (!navScene || !trajectoryData.length) return;

  const targetX = currentMissionParams.targetDist * 1000;
  navTargetRing.position.set(targetX * NAV_SCALE.x, 0.02, 0);
  navInnerRing.position.set(targetX * NAV_SCALE.x, 0.03, 0);

  // Guided path
  navGuidedPoints.length = 0;
  trajectoryData.forEach(d => {
    navGuidedPoints.push(
      new THREE.Vector3(d.x * NAV_SCALE.x, d.z * NAV_SCALE.z, d.y * NAV_SCALE.y)
    );
  });
  navGuidedLine.geometry.dispose();
  navGuidedLine.geometry = new THREE.BufferGeometry().setFromPoints(navGuidedPoints);

  // Unguided path
  navUnguidedPoints.length = 0;
  unguidedData.forEach(d => {
    navUnguidedPoints.push(
      new THREE.Vector3(d.x * NAV_SCALE.x, d.z * NAV_SCALE.z, d.y * NAV_SCALE.y)
    );
  });
  navUnguidedLine.geometry.dispose();
  navUnguidedLine.geometry = new THREE.BufferGeometry().setFromPoints(navUnguidedPoints);
}

function updateNavigationCamera() {
  if (!navCamera) return;
  const targetX = currentMissionParams.targetDist * 1000 * NAV_SCALE.x * 0.5;
  const target = new THREE.Vector3(targetX, 3.0, 0);

  const cx = Math.cos(navRotX) * Math.sin(navRotY) * navDist;
  const cy = Math.sin(navRotX) * navDist;
  const cz = Math.cos(navRotX) * Math.cos(navRotY) * navDist;

  navCamera.position.set(target.x + cx, target.y + cy, target.z + cz);
  navCamera.lookAt(target);
}

function setupNavigationControls() {
  navigation3D.addEventListener("pointerdown", e => {
    navDragging = true;
    navPrevX = e.clientX;
    navPrevY = e.clientY;
    navigation3D.setPointerCapture(e.pointerId);
  });

  navigation3D.addEventListener("pointermove", e => {
    if (!navDragging) return;
    const dx = e.clientX - navPrevX;
    const dy = e.clientY - navPrevY;
    navPrevX = e.clientX;
    navPrevY = e.clientY;

    navRotY -= dx * 0.007;
    navRotX -= dy * 0.007;
    navRotX = Math.max(-1.3, Math.min(1.3, navRotX));
    updateNavigationCamera();
  });

  const endDrag = () => { navDragging = false; };
  navigation3D.addEventListener("pointerup", endDrag);
  navigation3D.addEventListener("pointercancel", endDrag);

  navigation3D.addEventListener("wheel", e => {
    e.preventDefault();
    navDist += e.deltaY * 0.015;
    navDist = Math.max(6, Math.min(36, navDist));
    updateNavigationCamera();
  }, { passive: false });

  // 2-finger pinch zoom on touch screens
  let navPinchDist = 0;
  navigation3D.addEventListener("touchstart", e => {
    if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      navPinchDist = Math.hypot(dx, dy);
    }
  }, { passive: true });

  navigation3D.addEventListener("touchmove", e => {
    if (e.touches.length === 2 && navPinchDist > 0) {
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      const dist = Math.hypot(dx, dy);
      const diff = navPinchDist - dist;
      navPinchDist = dist;

      navDist += diff * 0.03;
      navDist = Math.max(6, Math.min(36, navDist));
      updateNavigationCamera();
    }
  }, { passive: true });
}

function resizeNavigation3D() {
  if (!navRenderer || !navCamera || !navigation3D) return;
  const w = navigation3D.clientWidth;
  const h = navigation3D.clientHeight;
  if (!w || !h) return;
  navCamera.aspect = w / h;
  navCamera.updateProjectionMatrix();
  navRenderer.setSize(w, h, false);
}

let nav3DVisible = true;
if ("IntersectionObserver" in window) {
  const navObserver = new IntersectionObserver(entries => {
    nav3DVisible = entries[0].isIntersecting;
  }, { threshold: 0.05 });
  if (navigation3D) navObserver.observe(navigation3D);
}

function renderNavigation3D() {
  requestAnimationFrame(renderNavigation3D);
  if (nav3DVisible && navRenderer && navScene && navCamera) {
    navRenderer.render(navScene, navCamera);
  }
}

/* =========================================================
   SIMULATION PLAYBACK & INTERPOLATION ENGINE
========================================================= */
let running = false;
let simTime = 0;
let lastTimestamp = null;
const playbackSpeed = 2.0; // 2x playback speed for smooth demonstration

function getInterpolatedState(t) {
  if (!trajectoryData.length) return null;
  if (t <= trajectoryData[0].t) return trajectoryData[0];
  if (t >= trajectoryData[trajectoryData.length - 1].t) {
    return trajectoryData[trajectoryData.length - 1];
  }

  let low = 0, high = trajectoryData.length - 1;
  while (low <= high) {
    const mid = Math.floor((low + high) / 2);
    if (trajectoryData[mid].t < t) low = mid + 1;
    else high = mid - 1;
  }

  const idx0 = Math.max(0, low - 1);
  const idx1 = Math.min(trajectoryData.length - 1, low);
  if (idx0 === idx1) return trajectoryData[idx0];

  const p0 = trajectoryData[idx0];
  const p1 = trajectoryData[idx1];
  const alpha = (t - p0.t) / (p1.t - p0.t);

  return {
    t,
    x: p0.x + (p1.x - p0.x) * alpha,
    y: p0.y + (p1.y - p0.y) * alpha,
    z: p0.z + (p1.z - p0.z) * alpha,
    vx: p0.vx + (p1.vx - p0.vx) * alpha,
    vy: p0.vy + (p1.vy - p0.vy) * alpha,
    vz: p0.vz + (p1.vz - p0.vz) * alpha,
    ax: p0.ax + (p1.ax - p0.ax) * alpha,
    ay: p0.ay + (p1.ay - p0.ay) * alpha,
    az: p0.az + (p1.az - p0.az) * alpha,
    speed: p0.speed + (p1.speed - p0.speed) * alpha,
    mach: p0.mach + (p1.mach - p0.mach) * alpha,
    dynamicPressure: p0.dynamicPressure + (p1.dynamicPressure - p0.dynamicPressure) * alpha,
    canardForce: p0.canardForce + (p1.canardForce - p0.canardForce) * alpha,
    canardRollAngle: p1.canardRollAngle,
    spin: p0.spin + (p1.spin - p0.spin) * alpha,
    estimatedX: p0.estimatedX + (p1.estimatedX - p0.estimatedX) * alpha,
    estimatedY: p0.estimatedY + (p1.estimatedY - p0.estimatedY) * alpha,
    estimatedZ: p0.estimatedZ + (p1.estimatedZ - p0.estimatedZ) * alpha,
    error: p0.error + (p1.error - p0.error) * alpha,
    accelSensor: p0.accelSensor + (p1.accelSensor - p0.accelSensor) * alpha,
    gyroSensor: p0.gyroSensor + (p1.gyroSensor - p0.gyroSensor) * alpha,
    health: p0.health,
    jitter: p0.jitter,
    solderState: p0.solderState,
    kalmanMult: p0.kalmanMult
  };
}

function updateSimulationView(data) {
  if (!data) return;

  // 1. Text telemetry
  altitudeEl.textContent = Math.round(data.z).toLocaleString();
  velocityEl.textContent = Math.round(data.speed).toLocaleString();
  machNumberEl.textContent = `M ${data.mach.toFixed(2)}`;
  accelerationEl.textContent = Math.hypot(data.ax, data.ay, data.az).toFixed(1);
  dynPressureEl.textContent = data.dynamicPressure.toFixed(1);
  canardForceEl.textContent = data.canardForce.toFixed(1);
  flightTimeEl.textContent = data.t.toFixed(1);

  introMach.textContent = `MACH ${data.mach.toFixed(2)}`;
  if (data.canardForce > 5) {
    introCanardStatus.textContent = "ROLL-STEERING ACTIVE";
  } else if (data.t > 5) {
    introCanardStatus.textContent = "ARMED (APOGEE)";
  } else {
    introCanardStatus.textContent = "SETBACK SAFE";
  }

  posXEl.textContent = data.estimatedX.toFixed(1);
  posYEl.textContent = data.estimatedY.toFixed(1);
  posZEl.textContent = data.estimatedZ.toFixed(1);
  errorEl.textContent = data.error.toFixed(2);

  heroPosition.textContent =
    `${Math.round(data.estimatedX).toString().padStart(5, "0")} / ` +
    `${Math.round(data.estimatedY).toString().padStart(5, "0")} / ` +
    `${Math.round(data.estimatedZ).toString().padStart(5, "0")}`;

  nav3DPosition.textContent = heroPosition.textContent;

  accelValueEl.textContent = data.accelSensor.toFixed(1);
  gyroValueEl.textContent = `${data.spin.toFixed(1)} Hz`;
  positionValueEl.textContent = Math.round(data.x).toLocaleString();

  // 2. TinyML Diagnostics
  healthScoreEl.textContent = data.health.toFixed(2);
  healthBarFill.style.width = `${Math.round(data.health * 100)}%`;
  if (data.health < 0.7) {
    healthBarFill.style.backgroundColor = "var(--amber)";
    tinymlBadge.textContent = "DEGRADED";
    tinymlBadge.className = "badge-warning";
  } else {
    healthBarFill.style.backgroundColor = "var(--accent)";
    tinymlBadge.textContent = "NOMINAL";
    tinymlBadge.className = "badge-nominal";
  }

  vibeJitterEl.textContent = `${data.jitter.toFixed(2)} g_rms`;
  solderStateEl.textContent = data.solderState;
  kalmanGainEl.textContent = data.kalmanMult;

  // 3. Update 3D Munition Position & Attitude
  const visualScale = 0.05;
  const scaledPos = new THREE.Vector3(
    data.x * visualScale,
    data.z * visualScale,
    data.y * visualScale
  );

  twin.position.copy(scaledPos);

  const direction = new THREE.Vector3(data.vx, data.vz, data.vy).normalize();
  twin.quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), direction);

  // Aft shell spins at 250 Hz (high axial spin)
  aftShellGroup.rotation.x += data.spin * 0.08;

  // Despun nosecone stays stabilized or commands roll steering angle
  if (data.canardForce > 2) {
    compCanards.rotation.x = data.canardRollAngle;
    canards.forEach(c => {
      c.rotation.z = Math.sin(data.t * 12) * 0.15; // Fin deflection actuation
    });
  } else {
    compCanards.rotation.x = 0;
    canards.forEach(c => { c.rotation.z = 0; });
  }

  // Supersonic Mach Cone visible when supersonic
  machCone.visible = data.mach > 1.0;
  if (machCone.visible) {
    machCone.scale.set(1, 1, 1).multiplyScalar(Math.min(1.8, data.mach * 0.6));
  }

  // Update shell position & direction for the continuous 60/120 FPS camera loop
  currentShellPos.copy(scaledPos);
  currentShellDir.copy(direction);

  // Twin Trail
  twinTrailPoints.push(scaledPos.clone());
  if (twinTrailPoints.length > 180) twinTrailPoints.shift();
  twinTrail.geometry.dispose();
  twinTrail.geometry = new THREE.BufferGeometry().setFromPoints(twinTrailPoints);

  // 4. Update Navigation 3D Marker
  if (navMarker) {
    navMarker.position.set(
      data.x * NAV_SCALE.x,
      data.z * NAV_SCALE.z,
      data.y * NAV_SCALE.y
    );
  }

  // 5. Update Timeline & Fuze State Machine
  updateFuzeTimeline(data.t, data.z, data.x);

  // 6. History for canvas charts
  history.push(data);
  if (history.length > MAX_GRAPH_POINTS) history.shift();
  drawGraphs();

  timeEl.textContent = `00:${data.t.toFixed(1).padStart(4, "0")}`;
}

function updateFuzeTimeline(t, z, x) {
  const setRow = (el, active, passed, statusText) => {
    el.classList.remove("active", "passed");
    if (active) el.classList.add("active");
    if (passed) el.classList.add("passed");
    const statusB = el.querySelector("b");
    if (statusB && statusText) statusB.textContent = statusText;
  };

  // Setback Shock
  setRow(tlSetback, t < 1.0, t >= 1.0, t < 1.0 ? "ARMING" : "ARMED (PASSED)");

  // Centrifugal Spin
  setRow(tlSpin, t >= 1.0 && t < 3.0, t >= 3.0, t >= 3.0 ? "ARMED (>200 Hz)" : "VERIFYING");

  // Safe Distance
  setRow(tlSafeDist, t >= 3.0 && x < 500, x >= 500, x >= 500 ? "PASSED (>500m)" : "ARMING");

  // Apogee
  setRow(tlApogee, t >= 25 && t < 45, t >= 45, t >= 45 ? "STEERING COMPLETE" : (t >= 25 ? "ACTIVE ROLL-STEER" : "STANDBY"));

  // Terminal FMCW
  setRow(tlTerminal, z < 150 && z > 15, z <= 15, z <= 15 ? "TARGET LOCKED" : (z < 150 ? "RADAR ACTIVE" : "STANDBY"));

  // Impact / Detonation
  if (z <= 8.0 && t > 30) {
    setRow(tlImpact, true, false, "BURST @ 7M AGL");
  } else {
    setRow(tlImpact, false, false, "STANDBY");
  }
}

function simulationLoop(timestamp) {
  if (!running) return;

  if (lastTimestamp === null) lastTimestamp = timestamp;
  const dt = (timestamp - lastTimestamp) / 1000;
  lastTimestamp = timestamp;

  simTime += dt * playbackSpeed;

  if (simTime >= totalFlightDuration) {
    simTime = totalFlightDuration;
    const finalState = getInterpolatedState(simTime);
    updateSimulationView(finalState);

    running = false;
    stateEl.textContent = "COMPLETE";
    connectionEl.textContent = "MISSION COMPLETE";
    startBtn.textContent = "RESTART SIMULATION";
    finalSummaryStatus.textContent = "TERMINAL BURST COMPLETE";
    finalSummaryText.textContent = `Successful strike. Final circular error probable (CEP) achieved: ${finalMissDistance.toFixed(1)}m at range ${(currentMissionParams.targetDist).toFixed(1)}km.`;
    return;
  }

  const currentState = getInterpolatedState(simTime);
  updateSimulationView(currentState);

  requestAnimationFrame(simulationLoop);
}

function startOrRestartSimulation() {
  if (simTime >= totalFlightDuration) {
    resetSimulation();
  }

  running = true;
  lastTimestamp = null;
  stateEl.textContent = "RUNNING";
  connectionEl.textContent = "SIMULATION ACTIVE";
  startBtn.textContent = "SIMULATION RUNNING";
  finalSummaryStatus.textContent = "FLIGHT ACTIVE";

  requestAnimationFrame(simulationLoop);
}

function resetSimulation() {
  running = false;
  simTime = 0;
  lastTimestamp = null;
  history.length = 0;
  twinTrailPoints.length = 0;

  twinTrail.geometry.dispose();
  twinTrail.geometry = new THREE.BufferGeometry();

  const initialState = getInterpolatedState(0);
  updateSimulationView(initialState);

  stateEl.textContent = "STANDBY";
  connectionEl.textContent = "SIMULATION READY";
  startBtn.textContent = "START SIMULATION";
  finalSummaryStatus.textContent = "READY";
}

/* =========================================================
   USER INPUTS, PRESETS & DATA INGESTION HANDLERS
========================================================= */

// Sliders and Value Updates
inputMuzzleVel.addEventListener("input", e => {
  valMuzzleVel.textContent = e.target.value;
  currentMissionParams.muzzleVel = parseFloat(e.target.value);
});

inputElevation.addEventListener("input", e => {
  valElevation.textContent = parseFloat(e.target.value).toFixed(1);
  currentMissionParams.elevation = parseFloat(e.target.value);
});

inputTargetDist.addEventListener("input", e => {
  valTargetDist.textContent = parseFloat(e.target.value).toFixed(1);
  currentMissionParams.targetDist = parseFloat(e.target.value);
});

inputCrosswind.addEventListener("input", e => {
  const val = parseFloat(e.target.value);
  valCrosswind.textContent = (val >= 0 ? `+${val.toFixed(1)}` : val.toFixed(1));
  currentMissionParams.crosswind = val;
});

inputCanardAuth.addEventListener("input", e => {
  valCanardAuth.textContent = parseFloat(e.target.value).toFixed(1);
  currentMissionParams.canardAuth = parseFloat(e.target.value);
});

inputNoiseMode.addEventListener("change", e => {
  currentMissionParams.noiseMode = e.target.value;
});

inputFuzeMode.addEventListener("change", e => {
  currentMissionParams.fuzeMode = e.target.value;
  activeFuzeDisplay.textContent = e.target.options[e.target.selectedIndex].text.toUpperCase();
});

applyParamsBtn.addEventListener("click", () => {
  recalculateTrajectory();
  resetSimulation();
  startOrRestartSimulation();
  missionDrawer.classList.add("collapsed");
  togglePlannerBtn.classList.remove("active");
});

// Presets
document.querySelectorAll(".preset-btn").forEach(btn => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".preset-btn").forEach(b => b.classList.remove("active"));
    btn.classList.add("active");

    const preset = btn.dataset.preset;
    if (preset === "nominal") {
      inputMuzzleVel.value = 827; valMuzzleVel.textContent = "827";
      inputElevation.value = 45.0; valElevation.textContent = "45.0";
      inputTargetDist.value = 19.4; valTargetDist.textContent = "19.4";
      inputCrosswind.value = 4.5; valCrosswind.textContent = "+4.5";
      inputCanardAuth.value = 10; valCanardAuth.textContent = "10.0";
      inputNoiseMode.value = "clean";
      inputFuzeMode.value = "proximity";
    } else if (preset === "unguided") {
      inputMuzzleVel.value = 827; valMuzzleVel.textContent = "827";
      inputElevation.value = 45.0; valElevation.textContent = "45.0";
      inputTargetDist.value = 19.4; valTargetDist.textContent = "19.4";
      inputCrosswind.value = 4.5; valCrosswind.textContent = "+4.5";
      inputCanardAuth.value = 0; valCanardAuth.textContent = "0.0";
      inputNoiseMode.value = "clean";
      inputFuzeMode.value = "impact";
    } else if (preset === "crosswind") {
      inputMuzzleVel.value = 827; valMuzzleVel.textContent = "827";
      inputElevation.value = 45.0; valElevation.textContent = "45.0";
      inputTargetDist.value = 19.4; valTargetDist.textContent = "19.4";
      inputCrosswind.value = 18.0; valCrosswind.textContent = "+18.0";
      inputCanardAuth.value = 12; valCanardAuth.textContent = "12.0";
      inputNoiseMode.value = "low";
      inputFuzeMode.value = "proximity";
    } else if (preset === "degraded") {
      inputMuzzleVel.value = 827; valMuzzleVel.textContent = "827";
      inputElevation.value = 45.0; valElevation.textContent = "45.0";
      inputTargetDist.value = 19.4; valTargetDist.textContent = "19.4";
      inputCrosswind.value = 4.5; valCrosswind.textContent = "+4.5";
      inputCanardAuth.value = 8; valCanardAuth.textContent = "8.0";
      inputNoiseMode.value = "spoofed";
      inputFuzeMode.value = "proximity";
    }

    currentMissionParams.muzzleVel = parseFloat(inputMuzzleVel.value);
    currentMissionParams.elevation = parseFloat(inputElevation.value);
    currentMissionParams.targetDist = parseFloat(inputTargetDist.value);
    currentMissionParams.crosswind = parseFloat(inputCrosswind.value);
    currentMissionParams.canardAuth = parseFloat(inputCanardAuth.value);
    currentMissionParams.noiseMode = inputNoiseMode.value;
    currentMissionParams.fuzeMode = inputFuzeMode.value;
    activeFuzeDisplay.textContent = inputFuzeMode.options[inputFuzeMode.selectedIndex].text.toUpperCase();

    recalculateTrajectory();
    resetSimulation();
  });
});

// Quick unguided compare button in Hero section
quickUnguidedBtn.addEventListener("click", () => {
  const isCurrentlyUnguided = currentMissionParams.canardAuth === 0;
  if (isCurrentlyUnguided) {
    currentMissionParams.canardAuth = 10;
    inputCanardAuth.value = 10;
    valCanardAuth.textContent = "10.0";
    quickUnguidedBtn.textContent = "COMPARE UNGUIDED";
  } else {
    currentMissionParams.canardAuth = 0;
    inputCanardAuth.value = 0;
    valCanardAuth.textContent = "0.0";
    quickUnguidedBtn.textContent = "RETURN TO GUIDED";
  }
  recalculateTrajectory();
  resetSimulation();
  startOrRestartSimulation();
});

// Drawer toggle buttons
togglePlannerBtn.addEventListener("click", () => {
  missionDrawer.classList.toggle("collapsed");
  togglePlannerBtn.classList.toggle("active");
});

closeDrawerBtn.addEventListener("click", () => {
  missionDrawer.classList.add("collapsed");
  togglePlannerBtn.classList.remove("active");
});

openParamsBtn.addEventListener("click", () => {
  missionDrawer.classList.remove("collapsed");
  togglePlannerBtn.classList.add("active");
  window.scrollTo({ top: 0, behavior: "smooth" });
});

// Start Simulation Button
startBtn.addEventListener("click", () => {
  if (running) {
    running = false;
    stateEl.textContent = "PAUSED";
    startBtn.textContent = "RESUME SIMULATION";
  } else {
    startOrRestartSimulation();
  }
});

// Data Export (JSON)
exportDataBtn.addEventListener("click", () => {
  const exportPayload = {
    mission: "155mm Precision Guidance Kit Simulation",
    standard: "NATO STANAG 4355 / MIL-STD-1316E",
    parameters: currentMissionParams,
    results: {
      maxApogee,
      totalFlightDuration,
      finalMissDistance,
      cepTargetGoal: "< 30m",
      cepAchieved: `${finalMissDistance.toFixed(2)}m`
    },
    telemetry: trajectoryData
  };

  const blob = new Blob([JSON.stringify(exportPayload, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = `155mm_pgk_telemetry_${Date.now()}.json`;
  a.click();
  URL.revokeObjectURL(url);
});

// File Upload (JSON or CSV)
importFileBtn.addEventListener("click", () => fileInput.click());
fileInput.addEventListener("change", e => {
  const file = e.target.files[0];
  if (!file) return;

  const reader = new FileReader();
  reader.onload = ev => {
    parseAndIngestTelemetryData(ev.target.result);
  };
  reader.readAsText(file);
});

// Ingest / Paste Modal
openPasteModalBtn.addEventListener("click", () => {
  pasteModal.classList.add("open");
});
closePasteModalBtn.addEventListener("click", () => {
  pasteModal.classList.remove("open");
});
cancelPasteBtn.addEventListener("click", () => {
  pasteModal.classList.remove("open");
});

loadSampleDataBtn.addEventListener("click", () => {
  // Prepopulate sample benchmark format
  const sample = trajectoryData.slice(0, 15).map(d => ({
    t: parseFloat(d.t.toFixed(1)),
    x: parseFloat(d.x.toFixed(1)),
    y: parseFloat(d.y.toFixed(1)),
    z: parseFloat(d.z.toFixed(1)),
    vx: parseFloat(d.vx.toFixed(1)),
    vy: parseFloat(d.vy.toFixed(1)),
    vz: parseFloat(d.vz.toFixed(1))
  }));
  customDataInput.value = JSON.stringify(sample, null, 2);
});

submitCustomDataBtn.addEventListener("click", () => {
  const rawText = customDataInput.value.trim();
  if (!rawText) {
    alert("Please paste or load telemetry data first.");
    return;
  }
  parseAndIngestTelemetryData(rawText);
  pasteModal.classList.remove("open");
});

function parseAndIngestTelemetryData(rawText) {
  try {
    let parsedPoints = [];

    // Try parsing as JSON first
    if (rawText.startsWith("[") || rawText.startsWith("{")) {
      const parsed = JSON.parse(rawText);
      const list = Array.isArray(parsed) ? parsed : (parsed.telemetry || parsed.points || []);

      parsedPoints = list.map(item => ({
        t: Number(item.t ?? item.time ?? 0),
        x: Number(item.x ?? item.downrange ?? 0),
        y: Number(item.y ?? item.crossrange ?? 0),
        z: Number(item.z ?? item.altitude ?? 0),
        vx: Number(item.vx ?? 0),
        vy: Number(item.vy ?? 0),
        vz: Number(item.vz ?? 0),
        ax: Number(item.ax ?? 0),
        ay: Number(item.ay ?? 0),
        az: Number(item.az ?? -9.8),
        speed: Math.hypot(item.vx || 0, item.vy || 0, item.vz || 0),
        mach: (Math.hypot(item.vx || 0, item.vy || 0, item.vz || 0) / 340),
        dynamicPressure: 25.0,
        canardForce: 45.0,
        canardRollAngle: 0.1,
        spin: 250,
        referenceX: Number(item.x || 0),
        referenceY: 0,
        referenceZ: Number(item.z || 0),
        estimatedX: Number(item.x || 0),
        estimatedY: Number(item.y || 0),
        estimatedZ: Number(item.z || 0),
        error: 0.5,
        accelSensor: 15.0,
        gyroSensor: 250.0,
        health: 0.98,
        jitter: 0.12,
        solderState: "INSPECT: OK",
        kalmanMult: "× 1.0 (NOMINAL)"
      }));
    } else {
      // Parse as CSV
      const lines = rawText.split(/\r?\n/).map(l => l.trim()).filter(l => l.length > 0);
      if (lines.length < 2) throw new Error("CSV requires header and at least one data row.");

      const header = lines[0].split(",").map(h => h.trim().toLowerCase());
      const tIdx = header.indexOf("t");
      const xIdx = header.indexOf("x");
      const yIdx = header.indexOf("y");
      const zIdx = header.indexOf("z");

      for (let i = 1; i < lines.length; i++) {
        const parts = lines[i].split(",").map(p => parseFloat(p.trim()));
        parsedPoints.push({
          t: parts[tIdx] || (i * 0.1),
          x: parts[xIdx] || 0,
          y: parts[yIdx] || 0,
          z: parts[zIdx] || 0,
          vx: 400, vy: 0, vz: -100,
          ax: 0, ay: 0, az: -9.8,
          speed: 400, mach: 1.2, dynamicPressure: 20, canardForce: 20,
          canardRollAngle: 0, spin: 250,
          referenceX: parts[xIdx] || 0, referenceY: 0, referenceZ: parts[zIdx] || 0,
          estimatedX: parts[xIdx] || 0, estimatedY: parts[yIdx] || 0, estimatedZ: parts[zIdx] || 0,
          error: 0.8, accelSensor: 20, gyroSensor: 250,
          health: 0.98, jitter: 0.12, solderState: "INSPECT: OK", kalmanMult: "× 1.0"
        });
      }
    }

    if (parsedPoints.length < 2) {
      throw new Error("Parsed dataset contains fewer than 2 points.");
    }

    trajectoryData = parsedPoints;
    totalFlightDuration = parsedPoints[parsedPoints.length - 1].t;
    maxApogee = Math.max(...parsedPoints.map(p => p.z));

    rebuild3DReferenceTrajectory();
    resetSimulation();
    startOrRestartSimulation();

    alert(`Successfully ingested ${parsedPoints.length} telemetry points. Duration: ${totalFlightDuration.toFixed(1)}s.`);
  } catch (err) {
    alert("Error parsing custom telemetry: " + err.message);
  }
}

/* =========================================================
   SCROLL POSITION OBSERVER
========================================================= */
const scenes = [...document.querySelectorAll(".scene")];
const sceneObserver = new IntersectionObserver(
  entries => {
    for (const entry of entries) {
      if (!entry.isIntersecting) continue;
      const idx = scenes.indexOf(entry.target) + 1;
      pageNumber.textContent = `${String(idx).padStart(2, "0")} / 05`;
    }
  },
  { threshold: 0.5 }
);
scenes.forEach(s => sceneObserver.observe(s));

/* =========================================================
   INITIALIZATION
========================================================= */
window.addEventListener("resize", () => {
  resizeGraphs();
  resizeTwin();
  resizeNavigation3D();
});

recalculateTrajectory();
resizeGraphs();
resizeTwin();
initNavigation3D();
setupTwinControls();
setupComponentInspection();
resetSimulation();
renderTwin();
