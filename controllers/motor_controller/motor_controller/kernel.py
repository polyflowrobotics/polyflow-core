from common.polyflow_kernel import PolyflowKernel


# Mirrors polyflow_msgs/MotorCommand constants; duplicated here so the kernel
# stays Pyodide-portable and free of ROS imports.
_MODE_SPEED = 0
_MODE_DUTY = 1
_MODE_IDLE = 2

_MODE_NAMES = {
    "speed": _MODE_SPEED,
    "duty":  _MODE_DUTY,
    "idle":  _MODE_IDLE,
}


class MotorControllerKernel(PolyflowKernel):
    """
    Portable logic for a motor controller.

    Receives a scalar command on "command" and emits a MotorCommand-shaped
    dict for the polyflow-os hardware daemon to route to the appropriate
    board driver.

    Input pins (as dicts):
        command — {"data": <float>}

    Output pins (as dicts):
        hw_motor_command — MotorCommand fields
            {motor_id, mode, value, timeout_s}

    Parameters:
        motor_id:   User-defined string ID (e.g., "left_drive").
        max_speed:  Clamp for SPEED mode. Ignored in DUTY mode (clamps to [-1, 1]).
        mode:       "speed" (default) or "duty".
        timeout_s:  Watchdog seconds applied to every command; 0 = adapter default.
    """

    def setup(self):
        self.motor_id = str(self.get_param("motor_id", "motor_0"))
        self.max_speed = float(self.get_param("max_speed", 1.0))

        mode_str = str(self.get_param("mode", "speed")).lower().strip()
        self.mode = _MODE_NAMES.get(mode_str, _MODE_SPEED)

        self.timeout_s = float(self.get_param("timeout_s", 0.0))

        self._current_value = 0.0

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id != "command":
            return

        raw = float(data.get("data", 0.0))
        if self.mode == _MODE_DUTY:
            self._current_value = max(-1.0, min(1.0, raw))
        else:
            self._current_value = max(-self.max_speed, min(self.max_speed, raw))

        self.emit("hw_motor_command", {
            "motor_id": self.motor_id,
            "mode": self.mode,
            "value": self._current_value,
            "timeout_s": self.timeout_s,
        })
