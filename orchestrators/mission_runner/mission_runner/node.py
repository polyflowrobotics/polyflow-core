import asyncio

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from common.polyflow_node import PolyflowNode
from mission_runner.kernel import MissionRunnerKernel


class MissionRunnerNode(PolyflowNode):
    """ROS wrapper for MissionRunnerKernel. Handles async mission execution timing."""

    kernel_class = MissionRunnerKernel

    def __init__(self):
        super().__init__()

        self.register_output_pin("cmd_vel", Twist)
        self.register_output_pin("status", String)

        self.get_logger().info(
            f"MissionRunner | linear_speed={self.kernel.linear_speed} | "
            f"angular_speed={self.kernel.angular_speed}"
        )

    async def _execute_mission(self, steps: list):
        """Run through each mission step sequentially."""
        self.kernel.emit_status(f"Mission started: {len(steps)} steps")

        for i, step in enumerate(steps):
            if not self.should_run():
                self.kernel.emit_status("Mission paused")
                while not self.should_run():
                    await asyncio.sleep(0.1)
                self.kernel.emit_status("Mission resumed")

            command = step["command"]
            duration = float(step["duration"])

            self.kernel.emit_status(f"Step {i + 1}/{len(steps)}: {command} for {duration}s")
            self.kernel.emit_cmd_vel(command)

            if duration > 0:
                await asyncio.sleep(duration)

        self.kernel.emit_cmd_vel("stop")
        self.kernel.emit_status("Mission complete")

    async def run_async(self):
        self.get_logger().info("MissionRunner ready")

        # Publish stop on startup
        self.kernel.emit_cmd_vel("stop")

        # Parse and execute the mission from parameters
        try:
            steps = self.kernel.parse_mission()
        except ValueError as e:
            self.kernel.emit_status(f"Mission rejected: {e}")
            return

        if not steps:
            self.kernel.emit_status("No mission configured")
            return

        await self._execute_mission(steps)

        # Keep node alive after mission completes
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    MissionRunnerNode.main(args)
