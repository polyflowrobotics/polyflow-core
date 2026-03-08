import math

from common.polyflow_kernel import PolyflowKernel


class DifferentialDriveKernel(PolyflowKernel):
    """
    Portable kinematics logic for a 4-wheel skid-steer differential drive.

    Converts (linear, angular) velocity commands into per-wheel speed commands
    and computes odometry from encoder feedback. All inputs and outputs are
    plain dicts — no ROS or hardware dependencies.

    Input pins (as dicts):
        cmd_vel         — {"linear": {"x": ...}, "angular": {"z": ...}}
        mode            — {"mode": "teleop"|"automated"|"stopped"}
        encoder_left    — {"position_rad": ...}
        encoder_right   — {"position_rad": ...}

    Output pins (as dicts):
        front_left_motor, rear_left_motor, front_right_motor, rear_right_motor
                        — {"data": <speed_float>}
        odometry        — {"x": ..., "y": ..., "theta": ..., "qz": ..., "qw": ...}

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

        # Odometry state
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_left_pos = None
        self._last_right_pos = None
        self._last_left_pos_pending = None

    def _clamp(self, value: float) -> float:
        return max(-self.max_wheel_speed, min(self.max_wheel_speed, value))

    def _emit_motor_commands(self, left_speed: float, right_speed: float):
        left_speed = self._clamp(left_speed)
        right_speed = self._clamp(right_speed)

        self.emit("front_left_motor", {"data": left_speed})
        self.emit("rear_left_motor", {"data": left_speed})
        self.emit("front_right_motor", {"data": right_speed})
        self.emit("rear_right_motor", {"data": right_speed})

    def _update_odometry(self, left_pos: float, right_pos: float):
        if self._last_left_pos is None or self._last_right_pos is None:
            self._last_left_pos = left_pos
            self._last_right_pos = right_pos
            return

        dl = (left_pos - self._last_left_pos) * self.wheel_radius
        dr = (right_pos - self._last_right_pos) * self.wheel_radius
        self._last_left_pos = left_pos
        self._last_right_pos = right_pos

        # Differential drive forward kinematics
        d_center = (dl + dr) / 2.0
        d_theta = (dr - dl) / self.wheel_separation

        self._x += d_center * math.cos(self._theta + d_theta / 2.0)
        self._y += d_center * math.sin(self._theta + d_theta / 2.0)
        self._theta += d_theta

        self.emit("odometry", {
            "header": {
                "frame_id": "odom",
            },
            "child_frame_id": "base_link",
            "pose": {"pose": {
                "position": {"x": self._x, "y": self._y, "z": 0.0},
                "orientation": {
                    "x": 0.0, "y": 0.0,
                    "z": math.sin(self._theta / 2.0),
                    "w": math.cos(self._theta / 2.0),
                },
            }},
        })

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

        elif pin_id == "encoder_left":
            self._last_left_pos_pending = data["position_rad"]

        elif pin_id == "encoder_right":
            if self._last_left_pos_pending is not None:
                self._update_odometry(self._last_left_pos_pending, data["position_rad"])
