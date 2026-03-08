import asyncio

from std_msgs.msg import Empty
from polyflow_msgs.msg import CameraFrame
from common.polyflow_node import PolyflowNode
from common.polyflow_kernel import PolyflowKernel


class CameraKernel(PolyflowKernel):
    """
    Portable logic for a camera sensor node.

    Handles capture triggers and emits frame metadata. The actual frame
    capture (OpenCV, etc.) is handled by the host wrapper which should
    populate the frame data before or after the kernel emits.

    Parameters:
        camera_id:      Identifier for this camera.
        device_index:   Camera device index (default: 0).
        width:          Frame width in pixels (default: 640).
        height:         Frame height in pixels (default: 480).
        fps:            Capture frame rate (default: 30).
    """

    def setup(self):
        self.camera_id = self.get_param("camera_id", "camera_0")
        self.device_index = int(self.get_param("device_index", 0))
        self.width = int(self.get_param("width", 640))
        self.height = int(self.get_param("height", 480))
        self.fps = float(self.get_param("fps", 30.0))
        self._connected = False

    def emit_frame(self, frame_data: list = None):
        """Emit a frame. The host wrapper provides actual pixel data."""
        self.emit("frame", {
            "camera_id": self.camera_id,
            "width": self.width,
            "height": self.height,
            "encoding": "jpeg",
            "data": frame_data or [],
            "connected": self._connected,
        })

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "capture":
            self.emit_frame()


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
