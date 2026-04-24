"""
HiwonderRRCMotorAdapter — one logical motor on an RRC motor channel.

Consumes MotorCommand (SPEED rad/s, DUTY [-1,1], IDLE) and HardwareLifecycle.
Publishes MotorState. The RRC firmware does not report encoder feedback,
so measured_value mirrors the last commanded value.
"""

import math
import time
from typing import Any, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    LEVEL_ERROR,
    LEVEL_WARN,
    LIFECYCLE_DISABLE,
    LIFECYCLE_ENABLE,
    LIFECYCLE_ESTOP,
    MOTOR_MODE_DUTY,
    MOTOR_MODE_IDLE,
    MOTOR_MODE_SPEED,
)

from .rrc import HiwonderRRC


_TWO_PI = 2.0 * math.pi


class HiwonderRRCMotorAdapter(HardwareAdapter):
    driver_name = "hiwonder_rrc_motor"
    command_types = ["MotorCommand", "HardwareLifecycle"]
    state_types = ["MotorState"]
    requires = ["hiwonder_rrc_board"]

    def configure(self) -> None:
        self.port = int(self.params["port"])
        self.gear_ratio = float(self.params.get("gear_ratio", 1.0))
        self.invert = bool(self.params.get("invert", False))
        self.state_hz = float(self.params.get("state_hz", 10.0))

        self._rrc: Optional[HiwonderRRC] = None
        self._state_handle: Any = None

        self._enabled: bool = True
        self._mode: int = MOTOR_MODE_IDLE
        self._commanded_value: float = 0.0
        self._last_cmd_time: float = 0.0
        self._watchdog_timeout: float = 0.0

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

        # Ensure channel starts stopped.
        self._write_speed_rps(0.0)

        if self.state_hz > 0:
            self._state_handle = self.schedule_poll(self.state_hz, self._publish_state)

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type == "MotorCommand":
            self._handle_motor_command(data)
        elif command_type == "HardwareLifecycle":
            self._handle_lifecycle(data)

    # --- Command handlers ---

    def _handle_motor_command(self, data: dict) -> None:
        if self._rrc is None:
            return
        if not self._enabled:
            self._write_speed_rps(0.0)
            return

        mode = int(data.get("mode", MOTOR_MODE_IDLE))
        value = float(data.get("value", 0.0))
        timeout_s = float(data.get("timeout_s", 0.0))

        try:
            if mode == MOTOR_MODE_SPEED:
                rps = (value / _TWO_PI) * self.gear_ratio
                self._write_speed_rps(-rps if self.invert else rps)
            elif mode == MOTOR_MODE_DUTY:
                duty_pct = max(-1.0, min(1.0, value)) * 100.0
                self._write_duty_pct(-duty_pct if self.invert else duty_pct)
            elif mode == MOTOR_MODE_IDLE:
                self._write_speed_rps(0.0)
                value = 0.0
            else:
                self.report_status(LEVEL_WARN, "unknown_mode", f"MotorCommand.mode={mode}")
                return
        except Exception as exc:
            self.report_status(LEVEL_WARN, "command_write_failed", str(exc))
            return

        self._mode = mode
        self._commanded_value = value
        self._last_cmd_time = time.monotonic()
        self._watchdog_timeout = timeout_s

    def _handle_lifecycle(self, data: dict) -> None:
        request = int(data.get("request", -1))
        if request == LIFECYCLE_ENABLE:
            self._enabled = True
        elif request in (LIFECYCLE_DISABLE, LIFECYCLE_ESTOP):
            self._enabled = False
            self._safe_stop()

    # --- Low-level writes ---

    def _write_speed_rps(self, rps: float) -> None:
        if self._rrc is None:
            return
        self._rrc.set_motor_speed([(self.port, float(rps))])

    def _write_duty_pct(self, duty: float) -> None:
        if self._rrc is None:
            return
        self._rrc.set_motor_duty([(self.port, float(duty))])

    def _safe_stop(self) -> None:
        try:
            self._write_speed_rps(0.0)
        except Exception as exc:
            self.log(f"safe stop failed: {exc}")
        self._mode = MOTOR_MODE_IDLE
        self._commanded_value = 0.0

    # --- Periodic ---

    def _publish_state(self) -> None:
        # Watchdog: if a timeout was set on the last command and it has elapsed, stop.
        if (
            self._watchdog_timeout > 0.0
            and self._mode != MOTOR_MODE_IDLE
            and (time.monotonic() - self._last_cmd_time) > self._watchdog_timeout
        ):
            self._safe_stop()
            self._watchdog_timeout = 0.0
            self.report_status(LEVEL_WARN, "watchdog_timeout", "MotorCommand watchdog fired — stopped")

        self.publish_state("MotorState", {
            "motor_id": self.target_id,
            "mode": self._mode,
            "commanded_value": self._commanded_value,
            "measured_value": self._commanded_value,
            "current": 0.0,
            "connected": self._rrc is not None,
        })

    # --- Lifecycle ---

    def estop(self) -> None:
        self._enabled = False
        self._safe_stop()

    def shutdown(self) -> None:
        if self._state_handle is not None:
            try:
                self._state_handle.cancel()
            except Exception:
                pass
            self._state_handle = None

        try:
            self._safe_stop()
        except Exception:
            pass
        self._rrc = None
