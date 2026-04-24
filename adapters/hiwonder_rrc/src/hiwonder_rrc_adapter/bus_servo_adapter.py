"""
HiwonderRRCBusServoAdapter — one LX-series bus servo on the RRC bus.

Consumes ServoCommand (position in rad, duration in seconds). Maps
[angle_min_rad, angle_max_rad] → [raw_min, raw_max] linearly. Polls the
servo's position back from the board at state_hz and publishes ServoState.

Note: read_bus_servo_position() on the driver is synchronous (sends a
read request, waits on the queue). Calling it off the state poll is safe
because the driver's _send has a write lock shared with other adapters.
"""

from typing import Any, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    LEVEL_ERROR,
    LEVEL_WARN,
)

from .rrc import HiwonderRRC


class HiwonderRRCBusServoAdapter(HardwareAdapter):
    driver_name = "hiwonder_rrc_bus_servo"
    command_types = ["ServoCommand"]
    state_types = ["ServoState"]
    requires = ["hiwonder_rrc_board"]

    def configure(self) -> None:
        self.servo_id = int(self.params["servo_id"])
        self.raw_min = int(self.params.get("raw_min", 0))
        self.raw_max = int(self.params.get("raw_max", 1000))
        self.angle_min_rad = float(self.params.get("angle_min_rad", -2.0944))
        self.angle_max_rad = float(self.params.get("angle_max_rad", 2.0944))
        self.default_duration_ms = int(self.params.get("default_duration_ms", 200))
        self.state_hz = float(self.params.get("state_hz", 5.0))

        self._rrc: Optional[HiwonderRRC] = None
        self._state_handle: Any = None

        self._last_commanded_rad: float = 0.0
        self._torque_enabled: bool = True
        self._connected: bool = False

    def start(self) -> None:
        rrc = self.resources.get("hiwonder_rrc_board")
        if rrc is None:
            self.report_status(
                LEVEL_ERROR,
                "board_missing",
                f"No hiwonder_rrc_board resource for '{self.target_id}'",
            )
            return
        self._rrc = rrc

        if self.state_hz > 0:
            self._state_handle = self.schedule_poll(self.state_hz, self._poll_state)

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type != "ServoCommand":
            return
        if self._rrc is None:
            return

        self._torque_enabled = bool(data.get("torque_enable", True))
        if not self._torque_enabled:
            # No native torque-off on this protocol path — skip writes.
            return

        position_rad = float(data.get("position", 0.0))
        duration_s = float(data.get("duration", 0.0))
        duration_ms = int(duration_s * 1000) if duration_s > 0 else self.default_duration_ms
        duration_ms = max(0, min(30000, duration_ms))

        raw = self._rad_to_raw(position_rad)
        try:
            self._rrc.set_bus_servo_position(self.servo_id, raw, duration_ms)
        except Exception as exc:
            self.report_status(LEVEL_WARN, "command_write_failed", str(exc))
            return

        self._last_commanded_rad = self._clamp_position(position_rad)

    # --- Mapping ---

    def _rad_to_raw(self, rad: float) -> int:
        rad = self._clamp_position(rad)
        span_rad = self.angle_max_rad - self.angle_min_rad
        if span_rad == 0:
            return (self.raw_min + self.raw_max) // 2
        t = (rad - self.angle_min_rad) / span_rad
        raw = self.raw_min + t * (self.raw_max - self.raw_min)
        lo = min(self.raw_min, self.raw_max)
        hi = max(self.raw_min, self.raw_max)
        return int(max(lo, min(hi, raw)))

    def _raw_to_rad(self, raw: int) -> float:
        span_raw = self.raw_max - self.raw_min
        if span_raw == 0:
            return 0.5 * (self.angle_min_rad + self.angle_max_rad)
        t = (raw - self.raw_min) / span_raw
        return self.angle_min_rad + t * (self.angle_max_rad - self.angle_min_rad)

    def _clamp_position(self, rad: float) -> float:
        lo = min(self.angle_min_rad, self.angle_max_rad)
        hi = max(self.angle_min_rad, self.angle_max_rad)
        return max(lo, min(hi, rad))

    # --- Periodic ---

    def _poll_state(self) -> None:
        rrc = self._rrc
        if rrc is None:
            return

        position_rad = self._last_commanded_rad
        connected = False
        try:
            raw = rrc.read_bus_servo_position(self.servo_id)
        except Exception as exc:
            self.log(f"read_bus_servo_position failed: {exc}")
            raw = None

        if raw is not None:
            position_rad = self._raw_to_rad(int(raw))
            connected = True

        self._connected = connected

        self.publish_state("ServoState", {
            "servo_id": self.target_id,
            "position": position_rad,
            "temperature": 0.0,
            "voltage": 0.0,
            "torque_enabled": self._torque_enabled,
            "connected": connected,
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
