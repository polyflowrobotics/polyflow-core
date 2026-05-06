import time

from common.polyflow_kernel import PolyflowKernel


_ZERO_TWIST = {
    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
}


class DifferentialDriveKernel(PolyflowKernel):
    """
    Portable kinematics logic for a 4-wheel skid-steer differential drive.

    Converts (linear, angular) velocity commands into per-wheel speed commands.
    All inputs and outputs are plain dicts — no ROS or hardware dependencies.

    Two cmd_vel inputs with priority:
        cmd_vel_teleop      — operator/gamepad input. Wins whenever a message
                              has arrived in the last `teleop_timeout_s`.
        cmd_vel_automated   — autonomous planner input. Used when teleop is
                              stale (or has never published).

    Input pins (as dicts):
        cmd_vel_teleop      — {"linear": {"x": ...}, "angular": {"z": ...}}
        cmd_vel_automated   — {"linear": {"x": ...}, "angular": {"z": ...}}

    Output pins (as dicts):
        front_left_motor, rear_left_motor, front_right_motor, rear_right_motor
                            — {"data": <speed_float>}

    Parameters:
        wheel_radius:       Wheel radius in meters (default: 0.05).
        wheel_separation:   Distance between left and right wheels in meters (default: 0.3).
        max_wheel_speed:    Maximum wheel speed in m/s (default: 1.0).
        teleop_timeout_s:   Seconds without a teleop message before we fall
                            back to automated (default: 1.0).
    """

    def setup(self):
        self.wheel_radius = float(self.get_param("wheel_radius", 0.05))
        self.wheel_separation = float(self.get_param("wheel_separation", 0.3))
        self.max_wheel_speed = float(self.get_param("max_wheel_speed", 1.0))
        self.teleop_timeout_s = float(self.get_param("teleop_timeout_s", 1.0))

        self._last_teleop_cmd = _ZERO_TWIST
        self._last_teleop_ts = 0.0
        self._last_automated_cmd = _ZERO_TWIST

    def _clamp(self, value: float) -> float:
        return max(-self.max_wheel_speed, min(self.max_wheel_speed, value))

    def _emit_motor_commands(self, left_speed: float, right_speed: float):
        left_speed = self._clamp(left_speed)
        right_speed = self._clamp(right_speed)
        left_rad_s = left_speed / self.wheel_radius
        right_rad_s = right_speed / self.wheel_radius

        self.emit("front_left_motor", {"data": left_rad_s})
        self.emit("rear_left_motor", {"data": left_rad_s})
        self.emit("front_right_motor", {"data": right_rad_s})
        self.emit("rear_right_motor", {"data": right_rad_s})

    def _select_command(self) -> dict:
        teleop_age = time.monotonic() - self._last_teleop_ts
        if self._last_teleop_ts > 0.0 and teleop_age < self.teleop_timeout_s:
            return self._last_teleop_cmd
        return self._last_automated_cmd

    def _emit_from_command(self, cmd: dict):
        linear = cmd["linear"]["x"]
        angular = cmd["angular"]["z"]
        left_speed = linear - (angular * self.wheel_separation / 2.0)
        right_speed = linear + (angular * self.wheel_separation / 2.0)
        self._emit_motor_commands(left_speed, right_speed)

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "cmd_vel_teleop":
            self._last_teleop_cmd = data
            self._last_teleop_ts = time.monotonic()
        elif pin_id == "cmd_vel_automated":
            self._last_automated_cmd = data
        else:
            return

        self._emit_from_command(self._select_command())
