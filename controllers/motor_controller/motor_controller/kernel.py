from common.polyflow_kernel import PolyflowKernel


class MotorControllerKernel(PolyflowKernel):
    """
    Portable logic for a generic motor controller.

    Receives velocity or position commands on "command" and emits motor state
    on "state". The actual hardware connection is managed by the host wrapper.

    Parameters:
        motor_id:       Identifier for the motor being controlled.
        control_mode:   "velocity" or "position" (default: "velocity").
        max_speed:      Maximum allowable speed value (default: 1.0).
    """

    def setup(self):
        self.motor_id = self.get_param("motor_id", "motor_0")
        self.control_mode = self.get_param("control_mode", "velocity")
        self.max_speed = float(self.get_param("max_speed", 1.0))
        self._current_command = 0.0
        self._connected = False

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "command":
            self._current_command = max(-self.max_speed, min(self.max_speed, data["data"]))

            self.emit("state", {
                "motor_id": self.motor_id,
                "control_mode": self.control_mode,
                "command": self._current_command,
                "connected": self._connected,
            })
