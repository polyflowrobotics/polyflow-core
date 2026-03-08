import json

from common.polyflow_kernel import PolyflowKernel


# Supported commands and their (linear.x, angular.z) direction multipliers
COMMAND_MAP = {
    "forward":    (1.0,  0.0),
    "backward":  (-1.0,  0.0),
    "turn_left":  (0.0,  1.0),
    "turn_right": (0.0, -1.0),
    "stop":       (0.0,  0.0),
}


class MissionRunnerKernel(PolyflowKernel):
    """
    Portable mission parsing and command generation logic.

    Parses and validates mission steps, converts command names into
    (linear, angular) velocity dicts. The async execution timing is
    handled by the host wrapper.

    Parameters:
        mission:        JSON array of {command, duration} steps.
        linear_speed:   Linear speed in m/s for forward/backward (default: 0.5).
        angular_speed:  Angular speed in rad/s for turns (default: 1.0).
    """

    def setup(self):
        self.linear_speed = float(self.get_param("linear_speed", 0.5))
        self.angular_speed = float(self.get_param("angular_speed", 1.0))
        self._mission_raw = self.get_param("mission", "[]")

    def make_cmd_vel(self, command: str) -> dict:
        """Build a cmd_vel dict for a given command name."""
        linear_dir, angular_dir = COMMAND_MAP.get(command, (0.0, 0.0))
        return {
            "linear": {"x": linear_dir * self.linear_speed, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_dir * self.angular_speed},
        }

    def emit_status(self, text: str):
        self.emit("status", {"data": text})
        self.log(text)

    def emit_cmd_vel(self, command: str):
        self.emit("cmd_vel", self.make_cmd_vel(command))

    def parse_mission(self, raw=None) -> list:
        """Parse and validate a mission. Accepts a JSON string or a list."""
        if raw is None:
            raw = self._mission_raw

        if isinstance(raw, str):
            try:
                steps = json.loads(raw)
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid mission JSON: {e}")
        elif isinstance(raw, list):
            steps = raw
        else:
            raise ValueError("Mission must be a JSON string or a list")

        if not isinstance(steps, list):
            raise ValueError("Mission must be a JSON array")

        for i, step in enumerate(steps):
            if not isinstance(step, dict):
                raise ValueError(f"Step {i} must be an object")
            cmd = step.get("command")
            if cmd not in COMMAND_MAP:
                raise ValueError(
                    f"Step {i}: unknown command '{cmd}'. "
                    f"Valid commands: {list(COMMAND_MAP.keys())}"
                )
            if "duration" not in step:
                raise ValueError(f"Step {i}: missing 'duration'")

        return steps

    def process_input(self, pin_id: str, data: dict):
        pass
