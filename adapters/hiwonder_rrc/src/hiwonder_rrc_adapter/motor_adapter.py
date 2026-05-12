"""
HiwonderRRCMotorAdapter — one logical motor on an RRC motor channel.

Consumes MotorCommand (SPEED rad/s, DUTY [-1,1], IDLE) and HardwareLifecycle.
Publishes MotorState.

The RRC firmware's PID-speed mode (sub-cmd 0x00/0x01) needs encoder
feedback to behave, and encoder hardware is currently broken — any
nonzero speed setpoint saturates to full PWM. This adapter therefore
maps both SPEED and DUTY commands onto raw-PWM (sub-cmd 0x04) so motion
is at least predictable open-loop. SPEED uses `max_speed_rad_s` as the
linear full-scale point. Once encoders are fixed, switch SPEED back to
`set_motor_speed`.

Direction inversion is handled upstream by the MotorController node
(`reverse` parameter), so MotorCommand values arrive with the desired
sign already applied. `max_speed_rad_s` is similarly fed in from the
controlling node's `max_speed` parameter via hardware.json — it is not
a user-editable adapter field.

Motor-state telemetry (SYS sub-cmd 0x05, 20 Hz) is decoded by the driver
and surfaced here as MotorState.measured_value. counter and rps will
read 0 until the encoder hardware is fixed.
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
_PWM_FULL_SCALE = 1000


class HiwonderRRCMotorAdapter(HardwareAdapter):
    driver_name = "hiwonder_rrc_motor"
    command_types = ["MotorCommand", "HardwareLifecycle"]
    state_types = ["MotorState"]
    requires = ["hiwonder_rrc_board"]

    def configure(self) -> None:
        self.port = int(self.params["port"])
        self.gear_ratio = float(self.params.get("gear_ratio", 1.0))
        self.state_hz = float(self.params.get("state_hz", 10.0))
        self.debug_log = bool(self.params.get("debug_log", False))
        # SPEED rad/s at which raw PWM saturates to ±1000. Used to map
        # MOTOR_MODE_SPEED into open-loop PWM while the on-board PID is
        # unusable (encoder feedback broken in current firmware). Sourced
        # from the controlling MotorController node's `max_speed` parameter
        # (serialized into hardware.json by the studio); defaults to 5.0
        # rad/s when the studio hasn't injected a value yet.
        self.max_speed_rad_s = float(self.params.get("max_speed_rad_s", 5.0))

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
        self._write_active_stop()

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
            self._write_active_stop()
            return

        mode = int(data.get("mode", MOTOR_MODE_IDLE))
        value = float(data.get("value", 0.0))
        timeout_s = float(data.get("timeout_s", 0.0))

        try:
            if mode == MOTOR_MODE_SPEED:
                if value == 0.0:
                    self._write_active_stop()
                else:
                    self._write_pwm(self._rad_s_to_pwm(value))
            elif mode == MOTOR_MODE_DUTY:
                duty = max(-1.0, min(1.0, value))
                self._write_pwm(int(round(duty * _PWM_FULL_SCALE)))
            elif mode == MOTOR_MODE_IDLE:
                self._write_active_stop()
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

    def _rad_s_to_pwm(self, rad_s: float) -> int:
        if self.max_speed_rad_s <= 0.0:
            return 0
        scaled = (rad_s / self.max_speed_rad_s) * _PWM_FULL_SCALE
        return max(-_PWM_FULL_SCALE, min(_PWM_FULL_SCALE, int(round(scaled))))

    def _write_pwm(self, pwm: int) -> None:
        if self._rrc is None:
            return
        if self.debug_log:
            self.log(f"port={self.port} write pwm={pwm:+d}")
        self._rrc.set_motor_pwm(self.port, pwm)

    def _write_active_stop(self) -> None:
        # Bitmask stop sub-command (0x03) — chosen over per-channel stop
        # because at least one shipping firmware variant only slows the
        # motor in response to the per-channel form. set_motor_speed(0)
        # is also unusable as a halt — firmware treats it as "no command".
        if self._rrc is None:
            return
        if self.debug_log:
            self.log(f"port={self.port} stop")
        self._rrc.stop_motor(self.port)

    def _safe_stop(self) -> None:
        try:
            self._write_active_stop()
        except Exception as exc:
            self.log(f"safe stop failed: {exc}")
        self._mode = MOTOR_MODE_IDLE
        self._commanded_value = 0.0

    # --- Periodic ---

    def _measured_value(self) -> float:
        """Current motor speed (rad/s on the output shaft) from telemetry,
        or the commanded value if no telemetry is available yet."""
        if self._rrc is None:
            return 0.0
        sample = self._rrc.get_motor_state(self.port)
        if sample is None:
            return self._commanded_value
        _counter, rps, _age = sample
        rad_s_motor = rps * _TWO_PI
        rad_s_output = rad_s_motor / self.gear_ratio if self.gear_ratio else rad_s_motor
        return rad_s_output

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
            "measured_value": self._measured_value(),
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
