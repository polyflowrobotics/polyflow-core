import asyncio
from typing import Optional

from std_msgs.msg import Float64
from polyflow_msgs.msg import MotorCommand, MotorState

from common.polyflow_node import (
    PolyflowNode,
    _dict_to_ros_msg,
    _ros_msg_to_dict,
    ros_hardware_topic_token,
)
from motor_controller.kernel import MotorControllerKernel


class MotorControllerNode(PolyflowNode):
    """
    ROS wrapper for MotorControllerKernel.

    Intercepts the kernel's "hw_command" emission, types it up as a
    MotorCommand, and publishes it on the PRP hardware command topic.
    Subscribes to the matching PRP MotorState topic and forwards feedback
    into the kernel, which re-emits a scalar on the graph "state" pin.
    """

    kernel_class = MotorControllerKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("command", Float64)
        self.register_output_pin("state", Float64)

        self._motor_cmd_publisher = self.create_publisher(
            MotorCommand, "/prp/hardware/cmd/motor_command", 10
        )

        motor_id = self.kernel.motor_id
        self._state_subscription: Optional[object] = None
        if motor_id:
            topic = f"/prp/hardware/state/{ros_hardware_topic_token(motor_id)}/motor_state"
            self._state_subscription = self.create_subscription(
                MotorState,
                topic,
                self._on_prp_motor_state,
                10,
            )
            self.get_logger().info(f"Subscribed to PRP state topic: {topic}")
        else:
            self.get_logger().warning(
                "No motor_id configured; PRP state subscription skipped"
            )

        original_emit = self.kernel.on_emit
        def on_kernel_emit(pin_id, data):
            if pin_id == "hw_command":
                self._publish_motor_command(data)
            elif original_emit:
                original_emit(pin_id, data)
        self.kernel.on_emit = on_kernel_emit

        self.get_logger().info(
            f"MotorController | motor_id={self.kernel.motor_id} "
            f"| mode={self.kernel.mode} | max_speed={self.kernel.max_speed}"
        )

    def _publish_motor_command(self, data: dict) -> None:
        msg = _dict_to_ros_msg(data, MotorCommand)
        msg.stamp = self.get_clock().now().to_msg()
        self._motor_cmd_publisher.publish(msg)
        self._trace_pin("OUT", "hw_command", msg)

    def _on_prp_motor_state(self, msg: MotorState) -> None:
        """Forward hardware feedback to the kernel (which re-emits on the graph state pin)."""
        self.kernel.update_state(_ros_msg_to_dict(msg))

    async def run_async(self):
        self.get_logger().info(f"MotorController motor_id={self.kernel.motor_id} ready")
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    MotorControllerNode.main(args)
