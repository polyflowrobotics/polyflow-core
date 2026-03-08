import asyncio

from std_msgs.msg import Empty
from polyflow_msgs.msg import EncoderState
from common.polyflow_node import PolyflowNode
from optical_encoder.kernel import OpticalEncoderKernel


class OpticalEncoderNode(PolyflowNode):
    """ROS wrapper for OpticalEncoderKernel."""

    kernel_class = OpticalEncoderKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("reset", Empty)
        self.register_output_pin("encoder_state", EncoderState)

        self.get_logger().info(
            f"OpticalEncoder '{self.kernel.encoder_id}' | ticks_per_rev={self.kernel.ticks_per_rev} | rate={self.kernel.publish_rate_hz}Hz"
        )

    async def run_async(self):
        self.get_logger().info(f"OpticalEncoder '{self.kernel.encoder_id}' starting up...")

        # TODO: Initialize hardware connection (e.g. GPIO, SPI, I2C) here
        self.kernel._connected = True
        self.get_logger().info(f"OpticalEncoder '{self.kernel.encoder_id}' ready")

        period = 1.0 / self.kernel.publish_rate_hz
        while True:
            if self.should_run():
                # TODO: Read actual encoder ticks from hardware here
                self.kernel.emit_state()

            await asyncio.sleep(period)


def main(args=None):
    OpticalEncoderNode.main(args)
