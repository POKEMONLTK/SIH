"""
SIH26098 Navigation & Sensor Fusion Engine
Implements Error-State Extended Kalman Filter (ESKF) fusing High-Rate IMU with Low-Rate GNSS
"""

import numpy as np
from config import GRAVITY

class ErrorStateKalmanFilter:
    """
    15-State Error-State Kalman Filter:
    - Position error (3): [dx, dy, dz]
    - Velocity error (3): [dvx, dvy, dvz]
    - Attitude error (3): [dtheta_x, dtheta_y, dtheta_z]
    - Accelerometer bias (3): [dba_x, dba_y, dba_z]
    - Gyroscope bias (3): [dbg_x, dbg_y, dbg_z]
    """
    def __init__(self, init_pos=np.array([0., 0., 1.]), init_vel=np.array([600., 0., 570.])):
        # Nominal state vector: pos (3), vel (3)
        self.pos = np.array(init_pos, dtype=np.float64)
        self.vel = np.array(init_vel, dtype=np.float64)
        
        # State error covariance P (15x15)
        self.P = np.eye(15, dtype=np.float64) * 0.1
        self.P[0:3, 0:3] *= 5.0    # Pos uncertainty
        self.P[3:6, 3:6] *= 1.0    # Vel uncertainty
        self.P[6:9, 6:9] *= 0.01   # Attitude uncertainty
        self.P[9:12, 9:12] *= 0.05 # Accel bias
        self.P[12:15, 12:15] *= 0.005 # Gyro bias

        # Process noise covariance Q (15x15)
        self.Q = np.eye(15, dtype=np.float64) * 1e-4
        self.Q[0:3, 0:3] *= 0.01
        self.Q[3:6, 3:6] *= 0.05
        self.Q[6:9, 6:9] *= 1e-4
        self.Q[9:12, 9:12] *= 1e-5
        self.Q[12:15, 12:15] *= 1e-6

        # Measurement noise covariance R for GNSS (6x6 for pos and vel)
        self.R_gnss = np.eye(6, dtype=np.float64)
        self.R_gnss[0:3, 0:3] *= (1.2 ** 2)  # 1.2m pos std
        self.R_gnss[3:6, 3:6] *= (0.15 ** 2) # 0.15m/s vel std

        # Measurement matrix H (6x15)
        self.H_gnss = np.zeros((6, 15), dtype=np.float64)
        self.H_gnss[0:3, 0:3] = np.eye(3)
        self.H_gnss[3:6, 3:6] = np.eye(3)

    def predict_imu(self, meas_accel, dt):
        """
        High-rate dead-reckoning step (100 Hz):
        Propagates nominal position and velocity using strapdown accelerometer data.
        """
        # Remove gravity: a_inertial = meas_accel + [0, 0, -g]
        a_inertial = meas_accel + np.array([0.0, 0.0, -GRAVITY])

        # State propagation
        self.pos += self.vel * dt + 0.5 * a_inertial * (dt ** 2)
        self.vel += a_inertial * dt

        # Error state transition matrix F (15x15)
        F = np.eye(15, dtype=np.float64)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 9:12] = -np.eye(3) * dt

        # Covariance propagation: P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q * dt
        return self.pos, self.vel

    def update_gnss(self, gnss_meas, gate_threshold=11.34):
        """
        Low-rate GNSS measurement update (10 Hz):
        Calculates innovation vector, innovation covariance, and Chi-square NIS.
        Rejects measurement if NIS exceeds gate_threshold.
        """
        z = np.hstack([gnss_meas['pos'], gnss_meas['vel']])
        z_pred = np.hstack([self.pos, self.vel])

        # Innovation vector: y = z - z_pred
        y = z - z_pred

        # Innovation covariance: S = H * P * H^T + R
        S = self.H_gnss @ self.P @ self.H_gnss.T + self.R_gnss

        # Normalized Innovation Squared (NIS): gamma = y^T * S^-1 * y
        S_inv = np.linalg.inv(S)
        nis = float(y.T @ S_inv @ y)

        # Chi-Square Gating
        is_valid = nis <= gate_threshold

        if is_valid:
            # Kalman gain: K = P * H^T * S^-1
            K = self.P @ self.H_gnss.T @ S_inv

            # Error state correction
            delta_x = K @ y

            # Correct nominal state
            self.pos += delta_x[0:3]
            self.vel += delta_x[3:6]

            # Update covariance: P = (I - K * H) * P
            I_KH = np.eye(15) - K @ self.H_gnss
            self.P = I_KH @ self.P @ I_KH.T + K @ self.R_gnss @ K.T
        
        return {
            'accepted': is_valid,
            'nis': nis,
            'residual_pos_norm': float(np.linalg.norm(y[0:3])),
            'residual_vel_norm': float(np.linalg.norm(y[3:6]))
        }
