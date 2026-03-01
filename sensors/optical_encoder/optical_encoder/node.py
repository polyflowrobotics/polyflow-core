import asyncio

from std_msgs.msg import Empty
from polyflow_msgs.msg import EncoderState
from common.polyflow_node import PolyflowNode


class OpticalEncoderNode(PolyflowNode):
    """
    Optical encoder sensor node.

    Reads encoder ticks from hardware at a configurable rate and publishes
    position/velocity estimates on the "encoder_state" output pin (EncoderState).
    Accepts a reset trigger on the "reset" input pin (Empty).

    Parameters (via POLYFLOW_PARAMETERS):
        encoder_id:         Identifier for this encoder.
        ticks_per_rev:      Encoder resolution in ticks per revolution (default: 1024).
        publish_rate_hz:    How often to publish state (default: 50).
    """

    def __init__(self):
        super().__init__()

        self.encoder_id = self.get_param("encoder_id", "encoder_0")
        self.ticks_per_rev = int(self.get_param("ticks_per_rev", 1024))
        self.publish_rate_hz = float(self.get_param("publish_rate_hz", 50.0))

        self._ticks = 0
        self._position_rad = 0.0
        self._velocity_rad_s = 0.0
        self._connected = False

        self.register_input_pin("reset", Empty)
        self.register_output_pin("encoder_state", EncoderState)

        self.get_logger().info(
            f"OpticalEncoder '{self.encoder_id}' | ticks_per_rev={self.ticks_per_rev} | rate={self.publish_rate_hz}Hz"
        )

    def process_input(self, pin_id: str, data):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "reset":
            self._ticks = 0
            self._position_rad = 0.0
            self._velocity_rad_s = 0.0
            self.get_logger().info(f"Encoder '{self.encoder_id}' reset")

    def _build_encoder_state(self) -> EncoderState:
        msg = EncoderState()
        msg.encoder_id = self.encoder_id
        msg.ticks = self._ticks
        msg.position_rad = self._position_rad
        msg.velocity_rad_s = self._velocity_rad_s
        msg.connected = self._connected
        return msg

    async def run_async(self):
        self.get_logger().info(f"OpticalEncoder '{self.encoder_id}' starting up...")

        # TODO: Initialize hardware connection (e.g. GPIO, SPI, I2C) here
        self._connected = True
        self.get_logger().info(f"OpticalEncoder '{self.encoder_id}' ready")

        period = 1.0 / self.publish_rate_hz
        while True:
            if self.should_run():
                # TODO: Read actual encoder ticks from hardware here
                self.publish_to_pin("encoder_state", self._build_encoder_state())

            await asyncio.sleep(period)


def main(args=None):
    OpticalEncoderNode.main(args)
