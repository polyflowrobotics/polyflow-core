import asyncio

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from polyflow_msgs.msg import ModeState
from common.polyflow_node import PolyflowNode
from differential_drive.kernel import DifferentialDriveKernel


class DifferentialDriveNode(PolyflowNode):
    """ROS wrapper for DifferentialDriveKernel."""

    kernel_class = DifferentialDriveKernel

    def __init__(self):
        super().__init__()

        # Register input pins
        self.register_input_pin("cmd_vel", Twist)
        self.register_input_pin("mode", ModeState)

        # Register output pins
        self.register_output_pin("front_left_motor", Float64)
        self.register_output_pin("rear_left_motor", Float64)
        self.register_output_pin("front_right_motor", Float64)
        self.register_output_pin("rear_right_motor", Float64)

        self.get_logger().info(
            f"DifferentialDrive | wheel_radius={self.kernel.wheel_radius} | "
            f"wheel_separation={self.kernel.wheel_separation} | max_wheel_speed={self.kernel.max_wheel_speed}"
        )

    async def run_async(self):
        self.get_logger().info("DifferentialDrive ready")

        # Publish zero commands on startup (rover starts stopped)
        self.kernel._emit_motor_commands(0.0, 0.0)

        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    DifferentialDriveNode.main(args)
