import asyncio

from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
from common.polyflow_node import PolyflowNode
from twist_to_pose.kernel import TwistToPoseKernel


class TwistToPoseNode(PolyflowNode):
    """ROS wrapper for TwistToPoseKernel."""

    kernel_class = TwistToPoseKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("cmd_vel", Twist)
        self.register_input_pin("seed_pose", Pose)
        self.register_input_pin("enable", Bool)
        self.register_output_pin("target_pose", Pose)

        self.get_logger().info(
            f"TwistToPose | rate={self.kernel.rate_hz}Hz | frame={self.kernel.frame} | "
            f"require_seed={self.kernel.require_seed} | require_enable={self.kernel.require_enable}"
        )

    async def run_async(self):
        self.get_logger().info("TwistToPose ready")
        period = 1.0 / max(self.kernel.rate_hz, 1e-3)
        while True:
            self.kernel.poll()
            await asyncio.sleep(period)


def main(args=None):
    TwistToPoseNode.main(args)
