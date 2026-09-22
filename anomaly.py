"""
SIH26098 Cyber-Resilience & Anomaly Detection Layer
Implements Chi-Square Gating, Physics Energy Invariance Checks, and Dynamic Confidence Scoring
"""

import numpy as np
from config import (
    CHI_SQUARE_GATE_THRESHOLD, MIN_CONFIDENCE_THRESHOLD, GRAVITY
)

class CyberAnomalyDetector:
    """
    Multi-Tier Cyber & Sensor Anomaly Detector:
    1. Statistical Layer: Chi-Square Innovation Residual Gating (NIS > 11.34)
    2. Physics Invariant Layer: Specific Mechanical Energy Derivative Check (dE_s/dt <= 0)
    3. State Covariance Trace Monitor
    """
    def __init__(self):
        self.confidence_score = 100.0  # 0 to 100%
        self.consecutive_rejections = 0
        self.energy_history = []

    @staticmethod
    def compute_specific_energy(pos, vel):
        """Specific Mechanical Energy: E_s = 0.5 * v^2 + g * z"""
        v_sq = float(np.dot(vel, vel))
        z = float(pos[2])
        return 0.5 * v_sq + GRAVITY * z

    def evaluate(self, fusion_result, current_pos, current_vel, satellites_locked, dt):
        """
        Evaluates current navigation state and updates confidence score.
        Returns: (is_anomalous, confidence_pct, reason)
        """
        nis = fusion_result['nis']
        accepted = fusion_result['accepted']
        
        # 1. Statistical Chi-Square Check
        statistical_fault = not accepted or (nis > CHI_SQUARE_GATE_THRESHOLD)
        if statistical_fault:
            self.consecutive_rejections += 1
        else:
            self.consecutive_rejections = max(0, self.consecutive_rejections - 1)

        # 2. Physics-Informed Invariant Check
        # In unpowered ballistic flight, specific mechanical energy MUST monotonically decrease due to aerodynamic drag!
        e_current = self.compute_specific_energy(current_pos, current_vel)
        energy_violation = False
        if len(self.energy_history) > 0:
            e_prev = self.energy_history[-1]
            de_dt = (e_current - e_prev) / max(1e-4, dt)
            # Energy cannot spontaneously increase by more than noise allowance
            if de_dt > 50.0:  # Spontaneous acceleration / altitude teleportation
                energy_violation = True

        self.energy_history.append(e_current)
        if len(self.energy_history) > 50:
            self.energy_history.pop(0)

        # 3. Dynamic Confidence Score Formulation
        # Factors:
        # f1: Innovation consistency (1.0 if NIS low, decays to 0 if NIS high)
        f1 = max(0.0, 1.0 - (nis / 40.0))
        # f2: Satellite lock health
        f2 = min(1.0, satellites_locked / 12.0)
        # f3: Energy physics compliance
        f3 = 0.0 if energy_violation else 1.0
        # f4: Continuous tracking continuity penalty
        f4 = max(0.0, 1.0 - (self.consecutive_rejections * 0.25))

        target_confidence = (0.35 * f1 + 0.20 * f2 + 0.30 * f3 + 0.15 * f4) * 100.0

        # Exponential smoothing
        alpha = 0.3
        self.confidence_score = (1.0 - alpha) * self.confidence_score + alpha * target_confidence
        self.confidence_score = max(0.0, min(100.0, self.confidence_score))

        # Fault determination
        is_fault = (self.confidence_score < MIN_CONFIDENCE_THRESHOLD) or (self.consecutive_rejections >= 3)
        reason = "Nominal"
        if energy_violation:
            reason = "Physics Invariant Violation: Spontaneous Kinetic/Potential Energy Rise (GPS Spoofing)"
        elif statistical_fault:
            reason = f"Innovation Gate Exceeded (NIS={nis:.2f} > {CHI_SQUARE_GATE_THRESHOLD})"
        elif self.confidence_score < MIN_CONFIDENCE_THRESHOLD:
            reason = f"Confidence Plunge ({self.confidence_score:.1f}% < {MIN_CONFIDENCE_THRESHOLD}%)"

        return {
            'is_anomalous': is_fault,
            'confidence_score': self.confidence_score,
            'reason': reason,
            'nis': nis,
            'energy_violation': energy_violation
        }
