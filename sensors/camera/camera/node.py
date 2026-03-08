import asyncio

from std_msgs.msg import Empty
from polyflow_msgs.msg import CameraFrame
from common.polyflow_node import PolyflowNode
from camera.kernel import CameraKernel


class CameraNode(PolyflowNode):
    """ROS wrapper for CameraKernel."""

    kernel_class = CameraKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("capture", Empty)
        self.register_output_pin("frame", CameraFrame)

        self.get_logger().info(
            f"Camera '{self.kernel.camera_id}' | device={self.kernel.device_index} | "
            f"{self.kernel.width}x{self.kernel.height}@{self.kernel.fps}fps"
        )

    async def run_async(self):
        self.get_logger().info(f"Camera '{self.kernel.camera_id}' starting up...")

        # TODO: Initialize camera capture here
        self.kernel._connected = True
        self.get_logger().info(f"Camera '{self.kernel.camera_id}' ready")

        period = 1.0 / self.kernel.fps
        while True:
            if self.should_run():
                # TODO: Read actual frame from hardware and pass bytes
                self.kernel.emit_frame()
            await asyncio.sleep(period)


def main(args=None):
    CameraNode.main(args)
