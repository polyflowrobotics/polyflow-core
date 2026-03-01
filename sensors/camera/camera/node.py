import asyncio

from std_msgs.msg import Empty
from polyflow_msgs.msg import CameraFrame
from common.polyflow_node import PolyflowNode


class CameraNode(PolyflowNode):
    """
    Camera sensor node.

    Captures frames from a camera device at a configurable rate and publishes
    them on the "frame" output pin (CameraFrame). Supports on-demand capture
    via the "capture" input pin (Empty).

    Parameters (via POLYFLOW_PARAMETERS):
        camera_id:          Identifier for this camera.
        device_index:       Camera device index (default: 0).
        width:              Frame width in pixels (default: 640).
        height:             Frame height in pixels (default: 480).
        fps:                Capture frame rate (default: 30).
    """

    def __init__(self):
        super().__init__()

        self.camera_id = self.get_param("camera_id", "camera_0")
        self.device_index = int(self.get_param("device_index", 0))
        self.width = int(self.get_param("width", 640))
        self.height = int(self.get_param("height", 480))
        self.fps = float(self.get_param("fps", 30.0))

        self._capture = None
        self._connected = False

        self.register_input_pin("capture", Empty)
        self.register_output_pin("frame", CameraFrame)

        self.get_logger().info(
            f"Camera '{self.camera_id}' | device={self.device_index} | {self.width}x{self.height}@{self.fps}fps"
        )

    def process_input(self, pin_id: str, data):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "capture":
            self._capture_and_publish()

    def _capture_and_publish(self):
        """Capture a single frame and publish it."""
        # TODO: Read frame from actual camera hardware here
        # Example with OpenCV:
        #   ret, frame = self._capture.read()
        #   if ret:
        #       _, encoded = cv2.imencode('.jpg', frame)
        #       frame_bytes = encoded.tobytes()

        msg = CameraFrame()
        msg.camera_id = self.camera_id
        msg.width = self.width
        msg.height = self.height
        msg.encoding = "jpeg"
        msg.data = bytes()  # TODO: actual frame bytes
        msg.connected = self._connected
        self.publish_to_pin("frame", msg)

    async def run_async(self):
        self.get_logger().info(f"Camera '{self.camera_id}' starting up...")

        # TODO: Initialize camera capture here
        # Example with OpenCV:
        #   import cv2
        #   self._capture = cv2.VideoCapture(self.device_index)
        #   self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        #   self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        #   self._capture.set(cv2.CAP_PROP_FPS, self.fps)
        self._connected = True
        self.get_logger().info(f"Camera '{self.camera_id}' ready")

        period = 1.0 / self.fps
        while True:
            if self.should_run():
                self._capture_and_publish()
            await asyncio.sleep(period)


def main(args=None):
    CameraNode.main(args)
