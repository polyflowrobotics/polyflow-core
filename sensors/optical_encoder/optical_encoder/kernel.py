from common.polyflow_kernel import PolyflowKernel


class OpticalEncoderKernel(PolyflowKernel):
    """
    Portable logic for an optical encoder sensor.

    Maintains tick/position/velocity state and handles reset commands.
    The host wrapper reads actual hardware ticks and updates the kernel
    state before calling emit_state().

    Parameters:
        encoder_id:         Identifier for this encoder.
        ticks_per_rev:      Encoder resolution in ticks per revolution (default: 1024).
        publish_rate_hz:    How often to publish state (default: 50).
    """

    def setup(self):
        self.encoder_id = self.get_param("encoder_id", "encoder_0")
        self.ticks_per_rev = int(self.get_param("ticks_per_rev", 1024))
        self.publish_rate_hz = float(self.get_param("publish_rate_hz", 50.0))
        self._ticks = 0
        self._position_rad = 0.0
        self._velocity_rad_s = 0.0
        self._connected = False

    def emit_state(self):
        self.emit("encoder_state", {
            "encoder_id": self.encoder_id,
            "ticks": self._ticks,
            "position_rad": self._position_rad,
            "velocity_rad_s": self._velocity_rad_s,
            "connected": self._connected,
        })

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "reset":
            self._ticks = 0
            self._position_rad = 0.0
            self._velocity_rad_s = 0.0
            self.log(f"Encoder '{self.encoder_id}' reset")
