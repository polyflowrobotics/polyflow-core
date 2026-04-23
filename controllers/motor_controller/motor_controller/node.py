import asyncio

from std_msgs.msg import Float64
from polyflow_msgs.msg import MotorCommand

from common.polyflow_node import PolyflowNode, _dict_to_ros_msg
from motor_controller.kernel import MotorControllerKernel


class MotorControllerNode(PolyflowNode):
    """
    ROS wrapper for MotorControllerKernel.

    Converts the kernel's hw_motor_command dicts into typed MotorCommand
    messages and publishes them on the PRP hardware command topic for the
    polyflow-os daemon to route to the appropriate board driver.
    """

    kernel_class = MotorControllerKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("command", Float64)

        self._motor_cmd_publisher = self.create_publisher(
            MotorCommand, "/prp/hardware/cmd/motor_command", 10
        )

        original_emit = self.kernel.on_emit
        def on_kernel_emit(pin_id, data):
            if pin_id == "hw_motor_command":
                self._publish_motor_command(data)
            elif original_emit:
                original_emit(pin_id, data)
        self.kernel.on_emit = on_kernel_emit

        self.get_logger().info(
            f"MotorController | motor_id={self.kernel.motor_id} "
            f"| mode={self.kernel.mode} | max_speed={self.kernel.max_speed}"
        )

    def _publish_motor_command(self, data: dict):
        msg = _dict_to_ros_msg(data, MotorCommand)
        msg.stamp = self.get_clock().now().to_msg()
        self._motor_cmd_publisher.publish(msg)

    async def run_async(self):
        self.get_logger().info(f"MotorController motor_id={self.kernel.motor_id} ready")
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    MotorControllerNode.main(args)
