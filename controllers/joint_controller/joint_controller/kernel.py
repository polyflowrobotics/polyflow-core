from typing import Dict, Optional

from common.polyflow_kernel import PolyflowKernel


# Mirrors polyflow_msgs/JointCommand constants; duplicated here so the kernel
# stays Pyodide-portable and free of ROS imports.
_MODE_POSITION = 0
_MODE_VELOCITY = 1
_MODE_TORQUE = 2

_MODE_NAMES = {
    "position": _MODE_POSITION,
    "velocity": _MODE_VELOCITY,
    "torque": _MODE_TORQUE,
}


class JointControllerKernel(PolyflowKernel):
    """
    Hardware-agnostic joint controller kernel.

    Consumes trajectory input from the graph, clamps / quantizes / smooths
    it against user-level joint limits (SI units), and emits a
    JointCommand-shaped dict for the hardware adapter to consume over PRP.

    Also forwards hardware state feedback: when the node receives a
    JointState on the PRP state topic, it calls update_state(data), which
    caches the latest values and emits them on the "state" pin for graph
    consumers.

    Input pins (as dicts):
        trajectory — JointTrajectoryPoint fields: {name[], positions[], velocities[], effort[]}
        mode       — {"data": "position" | "velocity" | "torque"}

    Output pins (as dicts):
        joint_command — JointCommand fields: {joint_id, mode, position, velocity,
                        torque, has_position, has_velocity, has_torque}
        state         — sensor_msgs/JointState fields (single-joint arrays)

    Parameters:
        joint_id:         Logical joint id; must match a target in hardware.yaml.
        default_mode:     "position" (default), "velocity", or "torque".
        min_angle, max_angle, position_step: SI radians.
        max_effort, effort_step:             SI Nm.
        max_velocity, velocity_step:         SI rad/s.
        smoothing_alpha:  Exponential smoothing factor [0, 1).
    """

    def setup(self):
        self.joint_id = self.get_param("joint_id")

        default_mode = str(self.get_param("default_mode", "position")).lower().strip()
        self.control_mode = _MODE_NAMES.get(default_mode, _MODE_POSITION)

        # Joint limits (all SI: rad, rad/s, Nm)
        self.lower_position = self.get_param("min_angle")
        self.upper_position = self.get_param("max_angle")
        self.position_step = self.get_param("position_step")
        self.max_effort = self.get_param("max_effort")
        self.effort_step = self.get_param("effort_step")
        self.max_velocity = self.get_param("max_velocity")
        self.velocity_step = self.get_param("velocity_step")

        # Smoothing state (keyed by joint_id for safety if reused)
        self.smoothing_alpha = float(self.get_param("smoothing_alpha", 0.0))
        self._last_commanded_position: Dict[str, float] = {}
        self._last_commanded_velocity: Dict[str, float] = {}

        # Cached state from PRP JointState feedback
        self._state_position: float = 0.0
        self._state_velocity: float = 0.0
        self._state_effort: float = 0.0
        self._state_connected: bool = False

    # --- Command path ---

    def process_input(self, pin_id: str, data: dict):
        if pin_id == "mode":
            raw = data.get("data") if isinstance(data, dict) else None
            if isinstance(raw, str) and raw.strip() in _MODE_NAMES:
                self.control_mode = _MODE_NAMES[raw.strip()]
                self.log(f"Control mode -> {raw.strip()} for joint {self.joint_id}")
            return

        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id != "trajectory":
            return

        names = data.get("name", [])
        positions = data.get("positions", [])
        if not names or not positions:
            return

        try:
            idx = list(names).index(self.joint_id)
        except ValueError:
            return  # not this joint's command

        position = float(positions[idx])
        velocity: Optional[float] = None
        effort: Optional[float] = None

        velocities = data.get("velocities", [])
        efforts = data.get("effort", data.get("efforts", []))
        if velocities and len(velocities) > idx:
            velocity = float(velocities[idx])
        if efforts and len(efforts) > idx:
            effort = float(efforts[idx])

        self._emit_command(position, velocity, effort)

    @staticmethod
    def _clamp(value, min_v, max_v):
        if min_v is not None:
            value = max(value, float(min_v))
        if max_v is not None:
            value = min(value, float(max_v))
        return value

    @staticmethod
    def _quantize(value, step):
        if step is None or step == 0:
            return value
        return round(value / float(step)) * float(step)

    def _emit_command(
        self,
        position: Optional[float],
        velocity: Optional[float],
        effort: Optional[float],
    ) -> None:
        cmd = {
            "joint_id": self.joint_id,
            "mode": self.control_mode,
            "position": 0.0,
            "velocity": 0.0,
            "torque": 0.0,
            "has_position": False,
            "has_velocity": False,
            "has_torque": False,
        }

        if self.control_mode == _MODE_POSITION and position is not None:
            p = self._clamp(position, self.lower_position, self.upper_position)
            p = self._quantize(p, self.position_step)
            if self.smoothing_alpha > 0 and self.joint_id in self._last_commanded_position:
                prev = self._last_commanded_position[self.joint_id]
                p = self.smoothing_alpha * prev + (1 - self.smoothing_alpha) * p
            self._last_commanded_position[self.joint_id] = p
            cmd["position"] = p
            cmd["has_position"] = True

        elif self.control_mode == _MODE_VELOCITY and velocity is not None:
            v_max = self.max_velocity
            v_min = -float(v_max) if v_max is not None else None
            v = self._clamp(velocity, v_min, v_max)
            v = self._quantize(v, self.velocity_step)
            if self.smoothing_alpha > 0 and self.joint_id in self._last_commanded_velocity:
                prev = self._last_commanded_velocity[self.joint_id]
                v = self.smoothing_alpha * prev + (1 - self.smoothing_alpha) * v
            self._last_commanded_velocity[self.joint_id] = v
            cmd["velocity"] = v
            cmd["has_velocity"] = True

        elif self.control_mode == _MODE_TORQUE and effort is not None:
            e = self._clamp(effort, None, self.max_effort)
            e = self._quantize(e, self.effort_step)
            cmd["torque"] = e
            cmd["has_torque"] = True

        if cmd["has_position"] or cmd["has_velocity"] or cmd["has_torque"]:
            self.emit("joint_command", cmd)

    # --- State feedback path ---

    def update_state(self, data: dict) -> None:
        """Called by the node when a JointState message for this joint arrives."""
        if data.get("joint_id") and data.get("joint_id") != self.joint_id:
            return

        self._state_position = float(data.get("position", 0.0) or 0.0)
        self._state_velocity = float(data.get("velocity", 0.0) or 0.0)
        self._state_effort = float(data.get("effort", 0.0) or 0.0)
        self._state_connected = bool(data.get("connected", False))

        # Forward to the graph as a sensor_msgs/JointState-shaped single-joint array.
        self.emit("state", {
            "name": [self.joint_id],
            "position": [self._state_position],
            "velocity": [self._state_velocity],
            "effort": [self._state_effort],
        })
