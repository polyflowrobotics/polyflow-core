import asyncio

from std_msgs.msg import Float64
from polyflow_msgs.msg import MotorState
from common.polyflow_node import PolyflowNode
from motor_controller.kernel import MotorControllerKernel


class MotorControllerNode(PolyflowNode):
    """
    ROS wrapper for MotorControllerKernel.

    Handles pin registration, hardware lifecycle, and ROS transport.
    """

    kernel_class = MotorControllerKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("command", Float64)
        self.register_output_pin("state", MotorState)

        self.get_logger().info(
            f"MotorController '{self.kernel.motor_id}' | mode={self.kernel.control_mode} | max_speed={self.kernel.max_speed}"
        )

    async def run_async(self):
        self.get_logger().info(f"MotorController '{self.kernel.motor_id}' starting up...")

        # TODO: Initialize hardware connection here
        self.kernel._connected = True
        self.get_logger().info(f"MotorController '{self.kernel.motor_id}' ready")

        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    MotorControllerNode.main(args)
