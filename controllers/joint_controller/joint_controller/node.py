"""
JointControllerNode — ROS wrapper for JointControllerKernel.

Consumes a trajectory_msgs/JointTrajectoryPoint (and optional mode
changes) from the graph, emits a typed polyflow_msgs/JointCommand on
the PRP hardware command topic, and subscribes to the corresponding
JointState on the PRP hardware state topic — forwarding the feedback
back into the graph as sensor_msgs/JointState.
"""

import asyncio
from typing import Optional

from std_msgs.msg import String
from sensor_msgs.msg import JointState as ROSJointState
from trajectory_msgs.msg import JointTrajectoryPoint
from polyflow_msgs.msg import JointCommand, JointState

from common.polyflow_node import PolyflowNode, _dict_to_ros_msg, _ros_msg_to_dict
from joint_controller.kernel import JointControllerKernel


class JointControllerNode(PolyflowNode):
    """ROS wrapper for JointControllerKernel — no hardware I/O."""

    kernel_class = JointControllerKernel

    def __init__(self):
        super().__init__()

        # Graph-facing pins
        self.register_input_pin("trajectory", JointTrajectoryPoint)
        self.register_input_pin("mode", String)
        self.register_output_pin("state", ROSJointState)

        # PRP hardware command: publish typed JointCommand for adapters to consume.
        self._cmd_publisher = self.create_publisher(
            JointCommand, "/prp/hardware/cmd/joint_command", 10
        )

        # PRP hardware state: subscribe to this joint's feedback from its adapter.
        joint_id = self.kernel.joint_id
        self._state_subscription: Optional[object] = None
        if joint_id:
            topic = f"/prp/hardware/state/{joint_id}/joint_state"
            self._state_subscription = self.create_subscription(
                JointState,
                topic,
                self._on_prp_joint_state,
                10,
            )
            self.get_logger().info(f"Subscribed to PRP state topic: {topic}")
        else:
            self.get_logger().warning(
                "No joint_id configured; PRP state subscription skipped"
            )

        # Intercept kernel emissions so we can convert joint_command dicts
        # into typed messages and route them to the PRP publisher.
        original_emit = self.kernel.on_emit

        def on_kernel_emit(pin_id, data):
            if pin_id == "joint_command":
                self._publish_joint_command(data)
                return
            if original_emit:
                original_emit(pin_id, data)

        self.kernel.on_emit = on_kernel_emit

        self.get_logger().info(
            f"JointController | joint_id={self.kernel.joint_id} | "
            f"mode={self.kernel.control_mode} | smoothing={self.kernel.smoothing_alpha}"
        )

    def _publish_joint_command(self, data: dict) -> None:
        msg = _dict_to_ros_msg(data, JointCommand)
        msg.stamp = self.get_clock().now().to_msg()
        self._cmd_publisher.publish(msg)

    def _on_prp_joint_state(self, msg: JointState) -> None:
        """Forward hardware feedback to the kernel (which re-emits on the graph state pin)."""
        self.kernel.update_state(_ros_msg_to_dict(msg))

    async def run_async(self):
        self.get_logger().info(
            f"JointController joint_id={self.kernel.joint_id} ready"
        )
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    """Entry point registered as joint_controller_node."""
    JointControllerNode.main(args=args)


if __name__ == "__main__":
    main()
