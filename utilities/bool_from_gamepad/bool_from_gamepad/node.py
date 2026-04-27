import asyncio

from polyflow_msgs.msg import GamepadButtons
from std_msgs.msg import Bool
from common.polyflow_node import PolyflowNode
from bool_from_gamepad.kernel import BoolFromGamepadKernel


class BoolFromGamepadNode(PolyflowNode):
    """ROS wrapper for BoolFromGamepadKernel."""

    kernel_class = BoolFromGamepadKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("buttons", GamepadButtons)
        self.register_output_pin("value", Bool)

        self.get_logger().info(
            f"BoolFromGamepad | button={self.kernel.button} | invert={self.kernel.invert}"
        )

    async def run_async(self):
        self.get_logger().info("BoolFromGamepad ready")
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    BoolFromGamepadNode.main(args)
