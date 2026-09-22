"""
SIH26098 Digital Twin Simulation Configuration
Low-Cost Precision Guidance and Smart Digital Twin for 155mm Research Demonstrator
"""

import math

# ==========================================
# 1. Projectile & Ballistics Constants (155mm M107 / M795)
# ==========================================
CALIBER = 0.155          # Projectile diameter (meters)
MASS = 43.5              # Mass (kg)
REF_AREA = math.pi * (CALIBER / 2.0) ** 2  # Reference cross-sectional area (m^2)
TWIST_RATE = 20.0        # Rifling twist: 1 turn in 20 calibers

# Muzzle launch conditions (Nominal Charge 7 / L52 Howitzer)
NOMINAL_MUZZLE_VELOCITY = 827.0  # m/s (~Mach 2.43)
LAUNCH_ELEVATION_DEG = 45.0      # Quadrant elevation (degrees)
LAUNCH_AZIMUTH_DEG = 0.0         # Azimuth heading (degrees)
INITIAL_SPIN_HZ = NOMINAL_MUZZLE_VELOCITY / (TWIST_RATE * CALIBER) # ~266.8 Hz

# Target coordinates (Nominal distance ~19.4 km for 45 deg launch)
TARGET_X = 19400.0       # Downrange target distance (m)
TARGET_Y = 0.0           # Crossrange target position (m)
TARGET_Z = 0.0           # Altitude (m)

# ==========================================
# 2. Atmospheric & Environmental Parameters
# ==========================================
GRAVITY = 9.80665        # m/s^2
RHO_0 = 1.225            # Sea-level atmospheric density (kg/m^3)
SCALE_HEIGHT = 8500.0    # Barometric scale height (meters)
SPEED_OF_SOUND_0 = 340.3 # Sea-level speed of sound (m/s)
EARTH_OMEGA = 7.292115e-5 # Earth rotation angular velocity (rad/s)
LATITUDE_DEG = 28.0      # Reference launch latitude (e.g. Pokhran / Northern India)

# ==========================================
# 3. Aerodynamics & Drag Model
# ==========================================
# Drag coefficient CD as a function of Mach number
CD_SUBSONIC = 0.22
CD_TRANSONIC_PEAK = 0.44
CD_SUPERSONIC_ASYMPTOTE = 0.26

# Deployable Aerodynamic 1D Airbrake / Canard parameters
AIRBRAKE_CD_INCREMENT = 0.24  # Additional drag when airbrake opens (CD: 0.22 -> 0.46)
CANARD_LIFT_MAX_N = 220.0     # Maximum net lift force (Newtons) at q ~ 40 kPa
APOGEE_CANARD_DEPLOY_SEC = 22.0

# ==========================================
# 4. Multi-Sensor Simulation Parameters
# ==========================================
# Virtual Multi-MEMS IMU Array (8 consumer sensors fused)
IMU_SAMPLE_RATE_HZ = 100.0    # 100 Hz dead-reckoning
IMU_DT = 1.0 / IMU_SAMPLE_RATE_HZ
ACCEL_NOISE_STD = 0.15        # Accelerometer noise (m/s^2) - reduced by sqrt(8)
GYRO_NOISE_STD = 0.008        # Gyroscope noise (rad/s)
ACCEL_BIAS_DRIFT = 0.02       # Accelerometer bias random walk (m/s^2 / sqrt(s))
GYRO_BIAS_DRIFT = 0.001       # Gyro bias random walk (rad/s / sqrt(s))

# Multi-Constellation GNSS (NavIC L5 + Commercial GNSS)
GNSS_SAMPLE_RATE_HZ = 10.0    # 10 Hz fixes
GNSS_DT = 1.0 / GNSS_SAMPLE_RATE_HZ
GNSS_POS_NOISE_STD = 1.2      # Position measurement noise std (meters)
GNSS_VEL_NOISE_STD = 0.15     # Velocity measurement noise std (m/s)
GNSS_LOCK_DELAY_SEC = 2.5     # Time post-muzzle-exit to acquire lock (seconds)

# ==========================================
# 5. Cyber-Resilience & Anomaly Gating Thresholds
# ==========================================
CHI_SQUARE_GATE_THRESHOLD = 11.34  # Critical value for 3 DOF at alpha = 0.01
CONFIDENCE_DECAY_RATE = 0.85       # Confidence score reduction multiplier upon fault
MIN_CONFIDENCE_THRESHOLD = 25.0    # Confidence % below which Safe Dud is triggered
MAX_ALLOWED_MISS_METERS = 150.0    # Collateral damage boundary

# ==========================================
# 6. Safety State Machine States
# ==========================================
STATE_SAFE = "SAFE"
STATE_INITIALIZING = "INITIALIZING"
STATE_TRACKING = "TRACKING"
STATE_INHIBITED_DUD = "INHIBITED_DUD"
STATE_TERMINAL_BURST = "PROXIMITY_BURST"
