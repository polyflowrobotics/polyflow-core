from common.polyflow_kernel import PolyflowKernel


class MotorControllerKernel(PolyflowKernel):
    """
    Portable logic for a motor controller.

    Receives speed commands on "command" and emits a hardware motor command
    for the OS system manager to route to the appropriate board driver via PRP.

    Input pins (as dicts):
        command — {"data": <speed_float>}

    Output pins (as dicts):
        hw_motor_command — {"motor_id": int, "speed": float}

    Parameters:
        motor_id:   Motor port ID (1-indexed).
        max_speed:  Maximum allowable speed value (default: 1.0).
    """

    def setup(self):
        self.motor_id = int(self.get_param("motor_id", 1))
        self.max_speed = float(self.get_param("max_speed", 1.0))
        self._current_speed = 0.0

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "command":
            self._current_speed = max(-self.max_speed, min(self.max_speed, data.get("data", 0.0)))
            self.emit("hw_motor_command", {
                "motor_id": self.motor_id,
                "speed": self._current_speed,
            })
