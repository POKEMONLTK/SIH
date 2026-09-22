"""
SIH26098 Synthetic Sensor Simulator Layer
Implements Multi-MEMS Array (Noise Averaged) and Multi-Constellation GNSS with Spoofing Injection
"""

import math
import numpy as np
from config import (
    ACCEL_NOISE_STD, GYRO_NOISE_STD, ACCEL_BIAS_DRIFT, GYRO_BIAS_DRIFT,
    GNSS_POS_NOISE_STD, GNSS_VEL_NOISE_STD, GNSS_LOCK_DELAY_SEC
)

class MultiMemsImuSimulator:
    """
    Simulates a virtual array of N=8 consumer MEMS sensors.
    Averaging uncorrelated white noise reduces random walk by 1/sqrt(N).
    Derives axial spin rate from centrifugal radial acceleration a_r = omega^2 * r.
    """
    def __init__(self, num_sensors=8, radial_offset_m=0.045):
        self.num_sensors = num_sensors
        self.radial_offset_m = radial_offset_m  # 45mm distance from projectile centerline
        
        # Initial turn-on biases
        self.accel_bias = np.random.normal(0.0, 0.05, 3)
        self.gyro_bias = np.random.normal(0.0, 0.005, 3)
        
        # Effective noise std reduced by array size
        self.eff_accel_noise = ACCEL_NOISE_STD / math.sqrt(num_sensors)
        self.eff_gyro_noise = GYRO_NOISE_STD / math.sqrt(num_sensors)

    def measure(self, true_accel, true_vel, true_spin_rad_s, dt):
        """
        Produces synthetic IMU reading [ax, ay, az, gx, gy, gz] with noise and bias drift.
        """
        # Bias random walk
        self.accel_bias += np.random.normal(0.0, ACCEL_BIAS_DRIFT * math.sqrt(dt), 3)
        self.gyro_bias += np.random.normal(0.0, GYRO_BIAS_DRIFT * math.sqrt(dt), 3)

        # Accelerometer measurement
        accel_noise = np.random.normal(0.0, self.eff_accel_noise, 3)
        meas_accel = true_accel + self.accel_bias + accel_noise

        # Radial acceleration from spin: a_r = omega^2 * r
        a_radial = (true_spin_rad_s ** 2) * self.radial_offset_m
        meas_radial = a_radial + np.random.normal(0.0, self.eff_accel_noise)

        # Derived spin rate from radial channel
        derived_spin = math.sqrt(max(0.0, meas_radial) / self.radial_offset_m)

        # Gyroscope measurement (gx, gy, gz)
        gyro_noise = np.random.normal(0.0, self.eff_gyro_noise, 3)
        meas_gyro = np.array([0.0, 0.0, derived_spin]) + self.gyro_bias + gyro_noise

        return {
            'accel': meas_accel,
            'gyro': meas_gyro,
            'derived_spin_hz': derived_spin / (2.0 * math.pi)
        }


class GnssReceiverSimulator:
    """
    Simulates Multi-Constellation GNSS (NavIC L1/L5 + GPS).
    Supports satellite lock delay, signal loss, and adversarial spoofing attacks.
    """
    def __init__(self):
        self.pos_noise_std = GNSS_POS_NOISE_STD
        self.vel_noise_std = GNSS_VEL_NOISE_STD
        self.spoofing_active = False
        self.spoof_offset = np.zeros(3, dtype=np.float64)
        self.satellites_locked = 12

    def set_spoofing(self, active=True, offset_xyz=np.array([200.0, -150.0, 50.0])):
        """Injects artificial coordinate displacement simulating an electronic attack"""
        self.spoofing_active = active
        self.spoof_offset = np.array(offset_xyz, dtype=np.float64) if active else np.zeros(3)

    def measure(self, true_pos, true_vel, current_time):
        """
        Returns GNSS observation or None if still in launch blackout / signal loss.
        """
        if current_time < GNSS_LOCK_DELAY_SEC:
            return None  # Muzzle exit ionization / acquisition delay

        pos_noise = np.random.normal(0.0, self.pos_noise_std, 3)
        vel_noise = np.random.normal(0.0, self.vel_noise_std, 3)

        meas_pos = true_pos + pos_noise
        meas_vel = true_vel + vel_noise

        if self.spoofing_active:
            meas_pos += self.spoof_offset
            # Spoofing introduces synthetic drift to velocity
            meas_vel += 0.05 * self.spoof_offset

        return {
            'pos': meas_pos,
            'vel': meas_vel,
            'satellites': self.satellites_locked,
            'is_spoofed': self.spoofing_active
        }
