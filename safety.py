"""
SIH26098 Safety State Machine Layer
Implements Multi-Environment Interlocks, Autonomous Dudding, and Proximity Fuzing
"""

from config import (
    STATE_SAFE, STATE_INITIALIZING, STATE_TRACKING,
    STATE_INHIBITED_DUD, STATE_TERMINAL_BURST, MAX_ALLOWED_MISS_METERS
)

class SafetyStateMachine:
    """
    Simulation Safety FSM:
    - SAFE: Stored / Pre-launch
    - INITIALIZING: Launch setback & spin verification window (0 to 2.5s)
    - TRACKING: Active guidance phase
    - INHIBITED_DUD: Fail-safe dud state (spoofing detected or predicted miss > boundary)
    - PROXIMITY_BURST: Terminal airburst triggered at 7m AGL
    """
    def __init__(self):
        self.current_state = STATE_SAFE
        self.setback_verified = False
        self.spin_verified = False
        self.guidance_inhibited = False
        self.event_log = []

    def update(self, current_time, altitude_m, predicted_miss_m, anomaly_info):
        """
        Evaluates state transitions based on time, altitude, predicted miss, and anomaly status.
        """
        # 1. State: SAFE -> INITIALIZING (Gun Firing)
        if self.current_state == STATE_SAFE and current_time > 0.05:
            self.current_state = STATE_INITIALIZING
            self.setback_verified = True
            self.event_log.append((current_time, "INTERLOCK 1: Launch setback acceleration pulse verified"))

        # 2. State: INITIALIZING -> TRACKING (Spin and Muzzle exit verified)
        if self.current_state == STATE_INITIALIZING and current_time >= 2.5:
            self.current_state = STATE_TRACKING
            self.spin_verified = True
            self.event_log.append((current_time, "INTERLOCK 2: Centrifugal rifling spin sustained. Transitions to TRACKING"))

        # 3. Fail-Safe Auto-Inhibit Check (Collateral Damage / Spoofing Defense)
        if self.current_state == STATE_TRACKING:
            should_inhibit = False
            inhibit_reason = ""

            if anomaly_info['is_anomalous']:
                should_inhibit = True
                inhibit_reason = f"Cyber Anomaly Triggered ({anomaly_info['reason']})"
            elif altitude_m <= 150.0 and predicted_miss_m > MAX_ALLOWED_MISS_METERS:
                should_inhibit = True
                inhibit_reason = f"Terminal Miss ({predicted_miss_m:.1f}m) exceeds safety corridor ({MAX_ALLOWED_MISS_METERS}m)"

            if should_inhibit:
                self.current_state = STATE_INHIBITED_DUD
                self.guidance_inhibited = True
                self.event_log.append((current_time, f"SAFETY ABORT: Transitioned to INHIBITED_DUD. Reason: {inhibit_reason}"))

        # 4. Terminal Proximity Trigger (7m AGL above target)
        if self.current_state == STATE_TRACKING and altitude_m <= 7.0:
            self.current_state = STATE_TERMINAL_BURST
            self.event_log.append((current_time, "TERMINAL FUZE: FMCW Proximity Radar triggered Airburst at 7m AGL"))

        return {
            'state': self.current_state,
            'canards_enabled': (self.current_state == STATE_TRACKING),
            'guidance_inhibited': self.guidance_inhibited,
            'log': self.event_log
        }
