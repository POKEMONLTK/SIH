"""
SIH26098 Projectile Flight Dynamics & Trajectory Simulation Engine
Implements NATO STANAG 4355 Modified Point Mass (MPM) Dynamics with RK4 Integration
"""

import math
import numpy as np
from config import (
    CALIBER, MASS, REF_AREA, GRAVITY, RHO_0, SCALE_HEIGHT, SPEED_OF_SOUND_0,
    EARTH_OMEGA, LATITUDE_DEG, CD_SUBSONIC, CD_TRANSONIC_PEAK, CD_SUPERSONIC_ASYMPTOTE,
    AIRBRAKE_CD_INCREMENT, CANARD_LIFT_MAX_N
)

class ProjectileSimulator:
    def __init__(self, elevation_deg=45.0, azimuth_deg=0.0, muzzle_velocity=827.0, crosswind_ms=0.0):
        self.elevation_rad = math.radians(elevation_deg)
        self.azimuth_rad = math.radians(azimuth_deg)
        self.muzzle_velocity = muzzle_velocity
        self.crosswind_ms = crosswind_ms
        
        # Initial velocity vector
        vx0 = muzzle_velocity * math.cos(self.elevation_rad) * math.cos(self.azimuth_rad)
        vy0 = muzzle_velocity * math.cos(self.elevation_rad) * math.sin(self.azimuth_rad)
        vz0 = muzzle_velocity * math.sin(self.elevation_rad)
        
        # State vector: [x, y, z, vx, vy, vz, spin_rad_s]
        p0 = (2.0 * math.pi * muzzle_velocity) / (20.0 * CALIBER)  # ~1676 rad/s
        self.state = np.array([0.0, 0.0, 1.0, vx0, vy0, vz0, p0], dtype=np.float64)
        self.time = 0.0
        
        # Control flags
        self.airbrake_active = False
        self.canard_lift_vector = np.zeros(3, dtype=np.float64)  # [Fx, Fy, Fz]
        self.trajectory_history = []
        self.apogee_reached = False
        self.impact_reached = False
        
        # Pre-compute Coriolis angular velocity vector for local latitude
        phi = math.radians(LATITUDE_DEG)
        # In local ENU (East=y, North=x, Up=z):
        self.omega_earth = EARTH_OMEGA * np.array([math.cos(phi), 0.0, math.sin(phi)])

    @staticmethod
    def get_atmospheric_density(altitude_m):
        """Standard barometric density lapse"""
        alt = max(0.0, altitude_m)
        return RHO_0 * math.exp(-alt / SCALE_HEIGHT)

    @staticmethod
    def get_speed_of_sound(altitude_m):
        """Speed of sound with standard temperature lapse rate"""
        T0 = 288.15  # K
        lapse_rate = 0.0065  # K/m
        T = max(216.65, T0 - lapse_rate * max(0.0, altitude_m))
        return math.sqrt(1.4 * 287.05 * T)

    @classmethod
    def get_drag_coefficient(cls, mach):
        """Mach-dependent drag curve with steep transonic wave drag spike"""
        if mach < 0.8:
            return CD_SUBSONIC
        elif 0.8 <= mach <= 1.15:
            # Transonic rise to peak
            t = (mach - 0.8) / (1.15 - 0.8)
            return CD_SUBSONIC + (CD_TRANSONIC_PEAK - CD_SUBSONIC) * (3.0 * t**2 - 2.0 * t**3)
        else:
            # Supersonic exponential decay
            return CD_SUPERSONIC_ASYMPTOTE + (CD_TRANSONIC_PEAK - CD_SUPERSONIC_ASYMPTOTE) * math.exp(-0.4 * (mach - 1.15))

    def compute_derivatives(self, state, t):
        """
        Computes d(state)/dt = [vx, vy, vz, ax, ay, az, dspin/dt]
        Implements NATO STANAG 4355 modified point mass dynamics.
        """
        x, y, z, vx, vy, vz, spin = state
        vel = np.array([vx, vy, vz])
        
        # Relative airspeed vector accounting for crosswind
        v_rel = vel - np.array([0.0, self.crosswind_ms, 0.0])
        v_mag = np.linalg.norm(v_rel)
        if v_mag < 1e-4:
            v_mag = 1e-4

        rho = self.get_atmospheric_density(z)
        sos = self.get_speed_of_sound(z)
        mach = v_mag / sos

        # Total Drag Coefficient (including deployable airbrake if active)
        cd = self.get_drag_coefficient(mach)
        if self.airbrake_active:
            cd += AIRBRAKE_CD_INCREMENT

        # Aerodynamic Drag Force vector: F_drag = -0.5 * rho * S * CD * v_rel * |v_rel|
        f_drag = -0.5 * rho * REF_AREA * cd * v_mag * v_rel
        a_drag = f_drag / MASS

        # Gravity: a_g = [0, 0, -g]
        a_grav = np.array([0.0, 0.0, -GRAVITY])

        # Coriolis Acceleration: a_coriolis = -2 * (Omega_earth x vel)
        a_coriolis = -2.0 * np.cross(self.omega_earth, vel)

        # Guidance Canard Lift Force
        a_canard = self.canard_lift_vector / MASS

        # Total Acceleration
        a_total = a_drag + a_grav + a_coriolis + a_canard

        # Spin decay: dspin/dt = -k * rho * v * spin
        k_spin = 0.00015
        dspin_dt = -k_spin * (rho / RHO_0) * v_mag * spin

        return np.array([vx, vy, vz, a_total[0], a_total[1], a_total[2], dspin_dt])

    def step_rk4(self, dt):
        """4th-order Runge-Kutta numerical integration step"""
        if self.impact_reached:
            return self.state

        s = self.state
        t = self.time

        k1 = self.compute_derivatives(s, t)
        k2 = self.compute_derivatives(s + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = self.compute_derivatives(s + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = self.compute_derivatives(s + dt * k3, t + dt)

        self.state = s + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        self.time += dt

        # Check apogee
        if self.state[5] <= 0.0 and not self.apogee_reached:
            self.apogee_reached = True

        # Check ground impact
        if self.state[2] <= 0.0:
            self.state[2] = 0.0
            self.impact_reached = True

        # Record trajectory snapshot
        self.trajectory_history.append({
            'time': self.time,
            'x': float(self.state[0]),
            'y': float(self.state[1]),
            'z': float(self.state[2]),
            'vx': float(self.state[3]),
            'vy': float(self.state[4]),
            'vz': float(self.state[5]),
            'spin_hz': float(self.state[6] / (2.0 * math.pi)),
            'mach': float(np.linalg.norm(self.state[3:6]) / self.get_speed_of_sound(self.state[2]))
        })

        return self.state
