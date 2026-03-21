import asyncio

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from common.polyflow_node import PolyflowNode
from inverse_kinematics.kernel import InverseKinematicsKernel


class InverseKinematicsNode(PolyflowNode):
    """ROS wrapper for InverseKinematicsKernel."""

    kernel_class = InverseKinematicsKernel

    def __init__(self):
        super().__init__()

        # Register input pins
        self.register_input_pin("target_pose", Pose)
        self.register_input_pin("joint_states", JointState)

        # Register output pins
        self.register_output_pin("joint_commands", JointTrajectoryPoint)

        self.get_logger().info(
            f"InverseKinematics | {len(self.kernel._chain)} joints | "
            f"max_iter={self.kernel.max_iterations} | tol={self.kernel.tolerance}"
        )

    async def run_async(self):
        self.get_logger().info("InverseKinematics ready")
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    InverseKinematicsNode.main(args)
