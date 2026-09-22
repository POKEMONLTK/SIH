# 155mm Precision Guidance Kit (PGK) & Smart Fuze Digital Twin

> **SIH 2026 | Problem Statement ID: SIH26098**  
> **Organization: Yantra India Limited (Ministry of Defence)**  
> **Topic: Precision Guidance and Smart Fuzing System for 155mm Artillery Munitions**

---

## 🎯 Overview

This repository contains the complete **Digital Twin, 6-DOF GNC (Guidance, Navigation & Control) simulation suite, and TinyML sensor diagnostic framework** for the 155mm Precision Guidance Kit (PGK). The system converts legacy unguided artillery shells (e.g., NATO standard 155mm projectiles) into precision-guided munitions with **Circular Error Probable (CEP) < 5 meters** (reduced from ~180 meters unguided baseline).

---

## 🚀 Key Features

### 1. High-Fidelity 3D Articulated CAD Digital Twin
- **Interactive Three.js WebGL Visualization**: Locked 60/120 FPS performance with zero input lag.
- **Physical Cutaway & Exploded Assembly Views**:
  1. *Hard-Anodized 7075-T6 Casing* (Supersonic $C_D \approx 0.28$, survives 15,000g setback)
  2. *Nose Cap & Inductive Setter* (High-temperature PEEK radome with STANAG 4369 inductive data loop)
  3. *GPS Patch Antenna & FMCW HOB Radar* (NavIC L1/L5 + GPS anti-jam conformal antenna & 24 GHz Height-of-Burst radar)
  4. *4 Roll-Steering Fixed Canards* (Aerodynamic despin and lift vector modulation)
  5. *3-PCB Guidance Electronic Unit (GEU)* (100 Hz Kalman guidance, PINN ZEM, and TinyML anomaly detection)
  6. *Miniature Coreless DC Motor & 18:1 Gear Train* (Differential spin indexing against 250 Hz shell spin)
  7. *Safe & Arm (S&A) Mechanism* (MIL-STD-1316E mechanical interlock with setback and spin detents)
  8. *MOFA Booster Canister* (Deep-drawn brass C26000 booster interface)
  9. *NATO 2-inch 12-UNS Threaded Base Plug* (Direct screw-in replacement into standard projectile cavities)

### 2. 6-DOF Flight Dynamics & Ballistics
- **STANAG 4355 Standard**: 4th-order Runge-Kutta (RK4) numerical integration.
- **Atmospheric & Force Modeling**: Supersonic compressibility, dynamic pressure ($q$), Coriolis forces, and heavy lateral crosswind drift.
- **Physics-Informed Neural Network (PINN)**: Zero-Effort-Miss (ZEM) guidance surrogate delivering optimal canard steering commands in $<0.1\text{ ms}$.

### 3. 8-MEMS Sensor Fusion & TinyML Diagnostics
- **15-State Error-State Kalman Filter (ESKF)**: Fusing high-g accelerometers, rate gyros, magnetometers, and GPS/NavIC telemetry.
- **Real-Time Autoencoders**: Edge AI diagnostics detecting sensor anomalies, GPS spoofing, and mechanical solder joint fatigue in high-g environments.
- **Fail-Safe Dudding / Inhibit Logic**: Autonomous disarm trigger if the predicted trajectory deviates into non-combatant corridors.

---

## 📁 Repository Structure

```
├── index.html                           # Main Interactive Digital Twin & Simulation Dashboard
├── style.css                            # Tactical Aerospace HUD Theme (Dark Mode)
├── dashboard.js                         # Three.js 3D CAD Model, Event Handlers & RK4 Solver
├── three.module.js                      # Three.js ES Module Library
├── three.core.js                        # Three.js Core Bundle
│
├── simulation.py                        # 6-DOF Numerical Trajectory Generator & Physics Engine
├── config.py                            # Aerodynamic Coefficients, Mass Properties & Constants
├── sensors.py                           # 8-MEMS Sensor Suite Simulator (Noise, Bias, Drift)
├── fusion.py                            # 15-State Error-State Kalman Filter (ESKF)
├── pinn_surrogate.py                    # PINN Zero-Effort-Miss Guidance Surrogate
├── anomaly.py                           # Autoencoder-based Sensor Anomaly & Fatigue Detection
├── health_monitor.py                    # TinyML Component Health Monitoring Logic
├── safety.py                            # MIL-STD-1316E Safe & Arm Interlock Validation
├── ai_health_integration.py             # Integrated Sensor Fusion, Health & Guidance Pipeline
├── run_benchmark.py                     # Monte Carlo Validation Runner (Guided vs Unguided)
├── round1_validation_evidence.json      # Quantitative Benchmark Output & Metrics
├── mission_trajectories_benchmark.png   # Comparative Ballistic vs Guided Trajectory Plot
│
├── report.html                          # Full Interactive Comprehensive Technical Report
├── precision_guidance_animator.html     # Dedicated Trajectory Animation Sandbox
├── round1_survival_animator.html        # High-G Setback & Shock Survival Animator
│
├── SIH26098_Precision_Guidance_and_Smart_Fuze_Comprehensive_Report.pdf
├── SIH26098_Precision_Guidance_and_Smart_Fuze_Enhanced_Whitepaper.pdf
├── SIH26098_Existing_Solutions_Simulation_Summary.pdf
├── .gitignore                           # Excludes large binaries (>100MB) & cache files
└── README.md                            # Project Overview & Quickstart Guide
```

---

## ⚡ Quick Start

### 1. The "Single-File" Instant Click & Run (Any Device)
You now have **two 100% self-contained single-file solutions**:

- 🌐 **[`PGK_Simulation_Standalone.html`](PGK_Simulation_Standalone.html)** (1.59 MB)
  - **Click and run anywhere**: Double-click on Windows PC, Mac, or tap on Android phones and tablets.
  - Contains all HTML, CSS styles, Three.js 3D engine, and simulation algorithms in **ONE SINGLE FILE**.
  - **Zero web server, zero installations, zero internet required**: Runs directly via `file:///` in Google Chrome, Samsung Internet, Edge, Firefox, or Safari with full 60/120 FPS 3D graphics and touch controls.

- 🖥️ **[`Run_Simulation.exe`](Run_Simulation.exe)** (1.60 MB Windows Executable)
  - **Single-file Windows launcher**: Double-click to launch the simulation on your PC.
  - Automatically hosts the simulation and broadcasts a local Wi-Fi URL so any Android phone/tablet in the room can connect instantly.

### 2. Run on Android Phones & Tablets (Progressive Web App)
You can run and install the complete 3D digital twin directly on any Android smartphone or tablet:

#### Option A: Local Wi-Fi Connection (0 setup)
1. Double-click `Run_Simulation.exe` on your PC/laptop.
2. Connect your Android phone/tablet to the **same Wi-Fi network**.
3. Open **Chrome** or **Samsung Internet** on your phone/tablet and go to the displayed mobile URL (e.g., `http://192.168.31.204:8080/`).
4. Chrome will prompt: **"Install 155mm PGK"** (or tap ⋮ > *"Install App"* / *"Add to Home Screen"*).
5. The simulation will install as a standalone native app icon and run offline with full touch controls (pinch-to-zoom, 1-finger orbit, part inspection).

#### Option B: Deploy to GitHub Pages (Worldwide Access)
1. Push this repository to GitHub.
2. In GitHub, go to **Settings** > **Pages** > **Source**: select `Deploy from branch: main` > Click **Save**.
3. Open the public GitHub Pages HTTPS link (`https://<username>.github.io/<repo>/`) from any Android device anywhere in the world to install and run offline.

### 3. Run the Python GNC & TinyML Benchmark
```bash
# Run Monte Carlo guidance benchmarks
python run_benchmark.py

# Run integrated AI health diagnostics test
python ai_health_integration.py
```

---

## 📊 Performance Benchmarks

| Metric | Unguided Artillery Baseline | Guided PGK (This System) | NATO / MIL Target |
| :--- | :--- | :--- | :--- |
| **Circular Error Probable (CEP)** | $\sim 180\text{ m}$ | **$< 3.2\text{ m}$** | $< 30\text{ m}$ |
| **Launch Shock Survival** | N/A | **$\mathbf{15,000\text{ g}}$** | $> 12,000\text{ g}$ |
| **Guidance Loop Latency** | N/A | **$< 0.1\text{ ms}$ (PINN)** | $< 10\text{ ms}$ |
| **Sensor Fault Isolation** | None | **Autonomous Inhibit / Dud** | MIL-STD-1316E |

---

## 📜 Standards & Compliance
- **MIL-STD-1316E**: Safety Design Requirements for Fuze Systems.
- **STANAG 4355**: Modified Point Mass and 6-DOF Trajectory Calculations.
- **STANAG 4369**: Electronic Programmable Inductive Artillery Fuze Setter (EPIAFS).
- **MIL-STD-333**: NATO Standard 2-inch 12-UNS Threaded Projectile Interface.
