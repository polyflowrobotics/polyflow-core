import asyncio

from geometry_msgs.msg import Twist
from polyflow_msgs.msg import GamepadAxes, GamepadButtons
from common.polyflow_node import PolyflowNode
from gamepad.kernel import GamepadKernel


class GamepadNode(PolyflowNode):
    """ROS wrapper for GamepadKernel."""

    kernel_class = GamepadKernel

    def __init__(self):
        super().__init__()

        self.register_output_pin("axes", GamepadAxes)
        self.register_output_pin("buttons", GamepadButtons)
        self.register_output_pin("cmd_vel", Twist)

        self.get_logger().info(
            f"Gamepad device={self.kernel.device_index} | poll_rate={self.kernel.poll_rate_hz}Hz | deadzone={self.kernel.deadzone}"
        )

    async def run_async(self):
        self.get_logger().info("Gamepad starting up...")

        # TODO: Initialize gamepad connection here
        self.kernel._connected = True
        self.get_logger().info("Gamepad ready")

        period = 1.0 / self.kernel.poll_rate_hz
        while True:
            if self.should_run():
                # TODO: Read actual gamepad state and pass values
                self.kernel.emit_gamepad_state()

            await asyncio.sleep(period)


def main(args=None):
    GamepadNode.main(args)
