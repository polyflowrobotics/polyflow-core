from typing import Optional

from common.polyflow_kernel import PolyflowKernel


class BoolFromGamepadKernel(PolyflowKernel):
    """
    Extracts a single button's pressed state from a GamepadButtons struct
    and republishes it as a std_msgs/Bool.

    Bridges the typed gap between Gamepad.buttons (a struct of named bools)
    and any Bool-input pin (e.g. the `enable` pin on TwistToPose).

    Parameters:
        button:               Which button to forward (e.g. "a", "lb").
        invert:               Negate the value before publishing.
        emit_only_on_change:  Edge-triggered output (default true) avoids
                              re-publishing the same value every input frame.
    """

    def setup(self):
        self.button: str = str(self.get_param("button", "a"))
        self.invert: bool = bool(self.get_param("invert", False))
        self.emit_only_on_change: bool = bool(self.get_param("emit_only_on_change", True))
        self._last_value: Optional[bool] = None

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return
        if pin_id != "buttons":
            return

        pressed = bool(data.get(self.button, False))
        value = (not pressed) if self.invert else pressed

        if self.emit_only_on_change and value == self._last_value:
            return

        self._last_value = value
        self.emit("value", {"data": value})
