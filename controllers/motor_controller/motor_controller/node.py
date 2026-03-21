import asyncio
import json

from std_msgs.msg import Float64, String
from common.polyflow_node import PolyflowNode
from motor_controller.kernel import MotorControllerKernel


class MotorControllerNode(PolyflowNode):
    """
    ROS wrapper for MotorControllerKernel.

    Receives speed commands from the node graph and publishes PRP hardware
    commands for the OS system manager to route to the board driver.
    """

    kernel_class = MotorControllerKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("command", Float64)

        # PRP hardware command output (picked up by the OS system manager)
        self._prp_publisher = self.create_publisher(
            String, "/prp/hardware/motor/set_speed", 10
        )

        # Intercept kernel hw_motor_command emissions
        original_emit = self.kernel.on_emit
        def on_kernel_emit(pin_id, data):
            if pin_id == "hw_motor_command":
                self._publish_prp_command(data)
            elif original_emit:
                original_emit(pin_id, data)
        self.kernel.on_emit = on_kernel_emit

        self.get_logger().info(
            f"MotorController | motor_id={self.kernel.motor_id} | max_speed={self.kernel.max_speed}"
        )

    def _publish_prp_command(self, data: dict):
        """Publish a PRP hardware motor command."""
        msg = String()
        msg.data = json.dumps(data)
        self._prp_publisher.publish(msg)

    async def run_async(self):
        self.get_logger().info(f"MotorController motor_id={self.kernel.motor_id} ready")
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    MotorControllerNode.main(args)
