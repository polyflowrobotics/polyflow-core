import asyncio
import json

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from common.polyflow_node import PolyflowNode


# Supported commands and their (linear.x, angular.z) Twist values
COMMAND_MAP = {
    "forward":    (1.0,  0.0),
    "backward":  (-1.0,  0.0),
    "turn_left":  (0.0,  1.0),
    "turn_right": (0.0, -1.0),
    "stop":       (0.0,  0.0),
}


class MissionRunnerNode(PolyflowNode):
    """
    Mission Runner orchestrator node.

    Executes a predefined mission — an ordered list of timed commands —
    publishing Twist messages for each step. Designed to feed into the
    Differential Drive node via cmd_vel.

    The mission is configured as a node parameter (JSON array):
        [
            {"command": "forward",    "duration": 2.0},
            {"command": "turn_left",  "duration": 1.0},
            {"command": "forward",    "duration": 3.0},
            {"command": "stop",       "duration": 0.0}
        ]

    Supported commands: forward, backward, stop, turn_right, turn_left.

    Output pins:
        cmd_vel   (Twist)  — velocity command for each step
        status    (String) — current mission execution status

    Parameters:
        mission:        JSON array of {command, duration} steps.
        linear_speed:   Linear speed in m/s for forward/backward (default: 0.5).
        angular_speed:  Angular speed in rad/s for turns (default: 1.0).
    """

    def __init__(self):
        super().__init__()

        self.linear_speed = float(self.get_param("linear_speed", 0.5))
        self.angular_speed = float(self.get_param("angular_speed", 1.0))

        self._mission_raw = self.get_param("mission", "[]")
        self._mission = None

        self.register_output_pin("cmd_vel", Twist)
        self.register_output_pin("status", String)

        self.get_logger().info(
            f"MissionRunner | linear_speed={self.linear_speed} | "
            f"angular_speed={self.angular_speed}"
        )

    def _make_twist(self, command: str) -> Twist:
        """Build a Twist message for a given command name."""
        linear_dir, angular_dir = COMMAND_MAP.get(command, (0.0, 0.0))
        twist = Twist()
        twist.linear.x = linear_dir * self.linear_speed
        twist.angular.z = angular_dir * self.angular_speed
        return twist

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.publish_to_pin("status", msg)
        self.log(text)

    def _parse_mission(self, raw) -> list:
        """Parse and validate a mission. Accepts a JSON string or a list."""
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

    async def _execute_mission(self, steps: list):
        """Run through each mission step sequentially."""
        self._publish_status(f"Mission started: {len(steps)} steps")

        for i, step in enumerate(steps):
            if not self.should_run():
                self._publish_status("Mission paused")
                while not self.should_run():
                    await asyncio.sleep(0.1)
                self._publish_status("Mission resumed")

            command = step["command"]
            duration = float(step["duration"])

            self._publish_status(f"Step {i + 1}/{len(steps)}: {command} for {duration}s")

            twist = self._make_twist(command)
            self.publish_to_pin("cmd_vel", twist)

            if duration > 0:
                await asyncio.sleep(duration)

        self.publish_to_pin("cmd_vel", self._make_twist("stop"))
        self._publish_status("Mission complete")

    async def run_async(self):
        self.get_logger().info("MissionRunner ready")

        # Publish stop on startup
        self.publish_to_pin("cmd_vel", self._make_twist("stop"))

        # Parse and execute the mission from parameters
        try:
            steps = self._parse_mission(self._mission_raw)
        except ValueError as e:
            self._publish_status(f"Mission rejected: {e}")
            return

        if not steps:
            self._publish_status("No mission configured")
            return

        await self._execute_mission(steps)

        # Keep node alive after mission completes
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    MissionRunnerNode.main(args)
