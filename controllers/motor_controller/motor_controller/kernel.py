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

    Receives a scalar command on "command", clamps/reverses it, and emits
    a MotorCommand-shaped dict on the internal "hw_command" channel for
    the node to type up and publish on the PRP hardware command topic.

    Input pins (as dicts):
        command — {"data": <float>}

    Output pins:
        state — measured motor value as a raw float scalar

    Internal kernel→node channel (not a graph pin):
        hw_command — MotorCommand fields
            {motor_id, mode, value, timeout_s}

    Parameters:
        motor_id:   User-defined string ID (e.g., "left_drive").
        max_speed:  Full-scale speed in rad/s, bound to the joint's
                    max_velocity. In SPEED mode the normalized [-1, 1] command
                    is scaled by this — the motor controller is the single place
                    speed is applied. Upstream nodes (gamepad, differential
                    drive) only pass normalized values. Ignored in DUTY mode.
        mode:       "speed" (default) or "duty".
        reverse:    Negate command before scaling; for motors mounted
                    mirrored from the convention (e.g. left-side drive
                    wheels whose joint axis points opposite the right side).
        timeout_s:  Watchdog seconds applied to every command; 0 = adapter default.
    """

    def setup(self):
        self.motor_id = str(self.get_param("motor_id", "motor_0"))
        self.max_speed = float(self.get_param("max_speed", 1.0))

        mode_str = str(self.get_param("mode", "speed")).lower().strip()
        self.mode = _MODE_NAMES.get(mode_str, _MODE_SPEED)

        self.reverse = bool(self.get_param("reverse", False))
        self.timeout_s = float(self.get_param("timeout_s", 0.0))

        self._current_value = 0.0

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id != "command":
            return

        raw = float(data.get("data", 0.0))
        if self.reverse:
            raw = -raw
        # Command is a normalized [-1, 1] setpoint from upstream. DUTY emits it
        # as-is (normalized duty); SPEED scales it by max_speed (the joint's
        # max_velocity) so this node is the single place speed is applied.
        norm = max(-1.0, min(1.0, raw))
        if self.mode == _MODE_DUTY:
            self._current_value = norm
        else:
            self._current_value = norm * self.max_speed

        self.emit("hw_command", {
            "motor_id": self.motor_id,
            "mode": self.mode,
            "value": self._current_value,
            "timeout_s": self.timeout_s,
        })

    # Suppress noise around zero. Physics-sim drift and real encoder
    # quantization both produce sub-threshold values when the motor is
    # commanded to stop — pinning those to 0 makes charts and downstream
    # consumers see a clean rest state.
    _STATE_DEADBAND = 0.05

    def update_state(self, data: dict) -> None:
        """Called by the node when a MotorState message for this motor arrives.

        Re-emits the measured value as a raw float on the graph "state" pin so
        consumers (e.g. a studio chart widget) get a clean scalar without
        hardware-bridge fields leaking into the graph.
        """
        if data.get("motor_id") and data.get("motor_id") != self.motor_id:
            return

        value = float(data.get("measured_value", 0.0) or 0.0)
        if abs(value) < self._STATE_DEADBAND:
            value = 0.0
        self.emit("state", value)
