from common.polyflow_kernel import PolyflowKernel


class DifferentialDriveKernel(PolyflowKernel):
    """
    Portable kinematics logic for a 4-wheel skid-steer differential drive.

    Converts (linear, angular) velocity commands into per-wheel speed commands.
    All inputs and outputs are plain dicts — no ROS or hardware dependencies.

    Input pins (as dicts):
        cmd_vel         — {"linear": {"x": ...}, "angular": {"z": ...}}
        mode            — {"mode": "teleop"|"automated"|"stopped"}

    Output pins (as dicts):
        front_left_motor, rear_left_motor, front_right_motor, rear_right_motor
                        — {"data": <speed_float>}

    Parameters:
        wheel_radius:       Wheel radius in meters (default: 0.05).
        wheel_separation:   Distance between left and right wheels in meters (default: 0.3).
        max_wheel_speed:    Maximum wheel speed in m/s (default: 1.0).
    """

    def setup(self):
        self.wheel_radius = float(self.get_param("wheel_radius", 0.05))
        self.wheel_separation = float(self.get_param("wheel_separation", 0.3))
        self.max_wheel_speed = float(self.get_param("max_wheel_speed", 1.0))

        self._current_mode = "stopped"

    def _clamp(self, value: float) -> float:
        return max(-self.max_wheel_speed, min(self.max_wheel_speed, value))

    def _emit_motor_commands(self, left_speed: float, right_speed: float):
        left_speed = self._clamp(left_speed)
        right_speed = self._clamp(right_speed)

        self.emit("front_left_motor", {"data": left_speed})
        self.emit("rear_left_motor", {"data": left_speed})
        self.emit("front_right_motor", {"data": right_speed})
        self.emit("rear_right_motor", {"data": right_speed})

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "mode":
            self._current_mode = data["mode"]
            if self._current_mode == "stopped":
                self._emit_motor_commands(0.0, 0.0)

        elif pin_id == "cmd_vel":
            if self._current_mode == "stopped":
                self._emit_motor_commands(0.0, 0.0)
                return

            linear = data["linear"]["x"]
            angular = data["angular"]["z"]

            # Unicycle to differential drive kinematics
            left_speed = linear - (angular * self.wheel_separation / 2.0)
            right_speed = linear + (angular * self.wheel_separation / 2.0)

            self._emit_motor_commands(left_speed, right_speed)
