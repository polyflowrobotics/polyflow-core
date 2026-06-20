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
        front_left_motor, back_left_motor, front_right_motor, back_right_motor
                            — {"data": <normalized [-1, 1]>}

    Inputs and outputs are all normalized to [-1, 1]. This node only does
    skid-steer kinematics — it does not apply speed. The downstream motor
    controllers scale the normalized command by their max_speed (the joint's
    max_velocity), so speed lives in exactly one place.

    Parameters:
        teleop_timeout_s:   Seconds without a teleop message before we fall
                            back to automated (default: 1.0).
    """

    def setup(self):
        self.teleop_timeout_s = float(self.get_param("teleop_timeout_s", 1.0))

        self._last_teleop_cmd = _ZERO_TWIST
        self._last_teleop_ts = 0.0
        self._last_automated_cmd = _ZERO_TWIST

    @staticmethod
    def _clamp(value: float) -> float:
        return max(-1.0, min(1.0, value))

    def _emit_motor_commands(self, left: float, right: float):
        left = self._clamp(left)
        right = self._clamp(right)

        self.emit("front_left_motor", {"data": left})
        self.emit("back_left_motor", {"data": left})
        self.emit("front_right_motor", {"data": right})
        self.emit("back_right_motor", {"data": right})

    def _select_command(self) -> dict:
        teleop_age = time.monotonic() - self._last_teleop_ts
        if self._last_teleop_ts > 0.0 and teleop_age < self.teleop_timeout_s:
            return self._last_teleop_cmd
        return self._last_automated_cmd

    def _emit_from_command(self, cmd: dict):
        # Normalized skid-steer: linear.x and angular.z are in [-1, 1]. Inverts
        # the gamepad's tank encoding (linear.x = (l+r)/2, angular.z = (r-l)/2)
        # back to per-side normalized commands.
        linear = cmd["linear"]["x"]
        angular = cmd["angular"]["z"]
        left = linear - angular
        right = linear + angular
        self._emit_motor_commands(left, right)

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
