import json
import math
from typing import Dict, Optional

from common.polyflow_kernel import PolyflowKernel


class ODriveS1Kernel(PolyflowKernel):
    """
    Portable trajectory-processing logic for an ODrive S1 controller.

    Parses incoming trajectory JSON, validates the target joint, clamps/quantizes
    values, applies exponential smoothing, and emits a processed hardware command
    dict. The actual hardware I/O (CAN, USB) is handled by the host wrapper.

    Emits on "hw_command":
        {"joint_id": str, "mode": str, "value": float, "scaled_value": float}
    """

    def setup(self):
        self.joint_id = self.get_param("joint")
        self.control_mode = "position"

        # Gear ratio: motor_turns = output_turns * gear_ratio
        gear_ratio = float(self.get_param("gear_ratio", 1.0))

        units = str(self.get_param("units", "radians")).lower().strip()
        if units in ("turn", "turns"):
            default_cmd_pos_scale = 1.0 * gear_ratio
            default_cmd_vel_scale = 1.0 * gear_ratio
            default_state_pos_scale = 1.0 / gear_ratio
            default_state_vel_scale = 1.0 / gear_ratio
        else:
            default_cmd_pos_scale = (1.0 / float(math.tau)) * gear_ratio
            default_cmd_vel_scale = (1.0 / float(math.tau)) * gear_ratio
            default_state_pos_scale = float(math.tau) / gear_ratio
            default_state_vel_scale = float(math.tau) / gear_ratio

        self.cmd_position_scale = float(self.get_param("cmd_position_scale", default_cmd_pos_scale))
        self.cmd_velocity_scale = float(self.get_param("cmd_velocity_scale", default_cmd_vel_scale))
        self.state_position_scale = float(self.get_param("state_position_scale", default_state_pos_scale))
        self.state_velocity_scale = float(self.get_param("state_velocity_scale", default_state_vel_scale))

        self.lower_position = self.get_param(["limit.lower_position", "lower_position"])
        self.upper_position = self.get_param(["limit.upper_position", "upper_position"])
        self.position_step = self.get_param(["limit.position_step", "position_step"])
        self.max_effort = self.get_param(["limit.max_effort", "max_effort"])
        self.effort_step = self.get_param(["limit.effort_step", "effort_step"])
        self.max_velocity = self.get_param(["limit.max_velocity", "max_velocity"])
        self.velocity_step = self.get_param(["limit.velocity_step", "velocity_step"])

        # Smoothing parameters
        self.smoothing_alpha = float(self.get_param("smoothing_alpha", 0.0))
        self._last_commanded_position: Dict[str, float] = {}
        self._last_commanded_velocity: Dict[str, float] = {}

    @staticmethod
    def _clamp(value: float, min_v: Optional[float], max_v: Optional[float]) -> float:
        if min_v is not None:
            value = max(value, float(min_v))
        if max_v is not None:
            value = min(value, float(max_v))
        return value

    @staticmethod
    def _quantize(value: float, step: Optional[float]) -> float:
        if step is None or step == 0:
            return value
        return round(value / float(step)) * float(step)

    def process_input(self, pin_id: str, data: dict):
        if pin_id == "mode":
            # std_msgs/String format: {"data": "position"}
            raw = data.get("data") if isinstance(data, dict) else None
            if isinstance(raw, str) and raw.strip() in ("position", "velocity", "torque"):
                self.control_mode = raw.strip()
                self.emit("state", {"control_mode": self.control_mode})
            return

        if not self.should_run(trigger_info={'pin_id': pin_id}):
            return

        if pin_id != "trajectory":
            return

        # The trajectory pin carries a JSON string inside a String msg
        raw_json = data.get("data")
        if not raw_json:
            return

        try:
            trajectory = json.loads(raw_json)
        except (json.JSONDecodeError, TypeError):
            self.log("Trajectory payload must be a JSON string")
            return

        if not isinstance(trajectory, dict):
            self.log("Trajectory payload must be an object")
            return

        # Check if the command is for the joint this node controls
        joint_id = trajectory.get("jointId") or trajectory.get("joint_id")
        if joint_id != self.joint_id:
            return

        point_data = trajectory.get("point") or trajectory.get("points", [{}])
        if isinstance(point_data, list):
            point_data = point_data[-1] if point_data else {}

        if not isinstance(point_data, dict):
            self.log("Trajectory point must be an object")
            return

        position = None
        velocity = None
        effort = None
        if point_data.get("positions"): position = point_data["positions"][0]
        if point_data.get("velocities"): velocity = point_data["velocities"][0]
        if point_data.get("effort"): effort = point_data["effort"][0]
        elif point_data.get("efforts"): effort = point_data["efforts"][0]

        self._compute_command(joint_id, position, velocity, effort)

    def _compute_command(
        self, joint_id: str, position: Optional[float], velocity: Optional[float], effort: Optional[float]
    ) -> None:
        """Clamp, quantize, smooth, scale, and emit a hardware command dict."""
        if self.control_mode == "velocity" and velocity is not None:
            vel_cmd = float(velocity)
            vel_cmd = self._clamp(vel_cmd, None, self.max_velocity)
            vel_cmd = self._quantize(vel_cmd, self.velocity_step)

            if self.smoothing_alpha > 0 and joint_id in self._last_commanded_velocity:
                vel_cmd = self.smoothing_alpha * self._last_commanded_velocity[joint_id] + (1 - self.smoothing_alpha) * vel_cmd
            self._last_commanded_velocity[joint_id] = vel_cmd

            self.emit("hw_command", {
                "joint_id": joint_id,
                "mode": "velocity",
                "value": vel_cmd,
                "scaled_value": vel_cmd * float(self.cmd_velocity_scale),
            })

        elif self.control_mode == "position" and position is not None:
            pos_cmd = float(position)
            pos_cmd = self._clamp(pos_cmd, self.lower_position, self.upper_position)
            pos_cmd = self._quantize(pos_cmd, self.position_step)

            if self.smoothing_alpha > 0 and joint_id in self._last_commanded_position:
                pos_cmd = self.smoothing_alpha * self._last_commanded_position[joint_id] + (1 - self.smoothing_alpha) * pos_cmd
            self._last_commanded_position[joint_id] = pos_cmd

            self.emit("hw_command", {
                "joint_id": joint_id,
                "mode": "position",
                "value": pos_cmd,
                "scaled_value": pos_cmd * float(self.cmd_position_scale),
            })

        elif self.control_mode == "torque" and effort is not None:
            eff_cmd = float(effort)
            eff_cmd = self._clamp(eff_cmd, None, self.max_effort)
            eff_cmd = self._quantize(eff_cmd, self.effort_step)

            self.emit("hw_command", {
                "joint_id": joint_id,
                "mode": "torque",
                "value": eff_cmd,
                "scaled_value": eff_cmd,
            })
