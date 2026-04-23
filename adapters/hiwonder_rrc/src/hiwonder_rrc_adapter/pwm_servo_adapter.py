"""
HiwonderRRCPWMServoAdapter — one PWM servo on a RRC PWM channel.

Consumes ServoCommand (position in rad, duration in seconds). Maps
[angle_min_rad, angle_max_rad] → [pulse_min_us, pulse_max_us] linearly.
PWM servos have no position feedback; ServoState echoes the last command.
"""

from typing import Any, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    LEVEL_ERROR,
    LEVEL_WARN,
)

from .rrc import HiwonderRRC


class HiwonderRRCPWMServoAdapter(HardwareAdapter):
    driver_name = "hiwonder_rrc_pwm_servo"
    command_types = ["ServoCommand"]
    state_types = ["ServoState"]
    requires = ["hiwonder_rrc_board"]

    def configure(self) -> None:
        self.board_id = str(self.params.get("board", ""))
        self.channel = int(self.params["channel"])
        self.pulse_min_us = int(self.params.get("pulse_min_us", 500))
        self.pulse_max_us = int(self.params.get("pulse_max_us", 2500))
        self.angle_min_rad = float(self.params.get("angle_min_rad", -1.5708))
        self.angle_max_rad = float(self.params.get("angle_max_rad", 1.5708))
        self.default_duration_ms = int(self.params.get("default_duration_ms", 20))
        self.state_hz = float(self.params.get("state_hz", 10.0))

        self._rrc: Optional[HiwonderRRC] = None
        self._state_handle: Any = None

        self._last_position_rad: float = 0.0
        self._torque_enabled: bool = True

    def start(self) -> None:
        rrc = self.resources.get("hiwonder_rrc_board")
        if rrc is None:
            self.report_status(
                LEVEL_ERROR,
                "board_missing",
                f"No hiwonder_rrc_board resource for '{self.board_id}'",
            )
            return
        self._rrc = rrc

        if self.state_hz > 0:
            self._state_handle = self.schedule_poll(self.state_hz, self._publish_state)

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type != "ServoCommand":
            return
        if self._rrc is None or not self._torque_enabled:
            return

        position_rad = float(data.get("position", 0.0))
        duration_s = float(data.get("duration", 0.0))
        duration_ms = int(duration_s * 1000) if duration_s > 0 else self.default_duration_ms

        # torque_enable on a PWM servo means "drive the output". When false, we
        # stop updating pulses — true hardware torque-off isn't available.
        self._torque_enabled = bool(data.get("torque_enable", True))

        pulse_us = self._rad_to_pulse(position_rad)
        try:
            self._rrc.set_pwm_servo(duration_ms, [(self.channel, pulse_us)])
        except Exception as exc:
            self.report_status(LEVEL_WARN, "command_write_failed", str(exc))
            return

        self._last_position_rad = self._clamp_position(position_rad)

    # --- Mapping ---

    def _rad_to_pulse(self, rad: float) -> int:
        rad = self._clamp_position(rad)
        span_rad = self.angle_max_rad - self.angle_min_rad
        if span_rad == 0:
            return (self.pulse_min_us + self.pulse_max_us) // 2
        t = (rad - self.angle_min_rad) / span_rad
        pulse = self.pulse_min_us + t * (self.pulse_max_us - self.pulse_min_us)
        lo = min(self.pulse_min_us, self.pulse_max_us)
        hi = max(self.pulse_min_us, self.pulse_max_us)
        return int(max(lo, min(hi, pulse)))

    def _clamp_position(self, rad: float) -> float:
        lo = min(self.angle_min_rad, self.angle_max_rad)
        hi = max(self.angle_min_rad, self.angle_max_rad)
        return max(lo, min(hi, rad))

    # --- Periodic ---

    def _publish_state(self) -> None:
        self.publish_state("ServoState", {
            "servo_id": self.target_id,
            "position": self._last_position_rad,
            "temperature": 0.0,
            "voltage": 0.0,
            "torque_enabled": self._torque_enabled,
            "connected": self._rrc is not None,
        })

    # --- Lifecycle ---

    def shutdown(self) -> None:
        if self._state_handle is not None:
            try:
                self._state_handle.cancel()
            except Exception:
                pass
            self._state_handle = None
        self._rrc = None
