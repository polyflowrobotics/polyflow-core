import asyncio

from polyflow_msgs.msg import ModeCommand, ModeState
from common.polyflow_node import PolyflowNode


class ModeSwitcherNode(PolyflowNode):
    """
    Mode switcher utility node.

    Manages the rover's operating mode: teleop, automated, or stopped.
    Receives mode change requests on the "set_mode" input pin (ModeCommand)
    and broadcasts the current mode on the "mode" output pin (ModeState).

    Parameters (via POLYFLOW_PARAMETERS):
        default_mode:   Starting mode (default: "stopped").
        allowed_modes:  List of valid modes (default: ["teleop", "automated", "stopped"]).
    """

    VALID_MODES = {"teleop", "automated", "stopped"}

    def __init__(self):
        super().__init__()

        self.allowed_modes = set(self.get_param("allowed_modes", list(self.VALID_MODES)))
        self._current_mode = self.get_param("default_mode", "stopped")

        if self._current_mode not in self.allowed_modes:
            self.get_logger().warn(
                f"Default mode '{self._current_mode}' not in allowed modes, falling back to 'stopped'"
            )
            self._current_mode = "stopped"

        self.register_input_pin("set_mode", ModeCommand)
        self.register_output_pin("mode", ModeState)

        self.get_logger().info(
            f"ModeSwitcher | mode={self._current_mode} | allowed={sorted(self.allowed_modes)}"
        )

    def _broadcast_mode(self):
        msg = ModeState()
        msg.mode = self._current_mode
        self.publish_to_pin("mode", msg)

    def process_input(self, pin_id: str, data):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "set_mode":
            requested = data.mode

            if requested not in self.allowed_modes:
                self.get_logger().warn(f"Rejected invalid mode '{requested}'")
                return

            if requested != self._current_mode:
                prev = self._current_mode
                self._current_mode = requested
                self.get_logger().info(f"Mode changed: {prev} -> {self._current_mode}")
                self._broadcast_mode()

    async def run_async(self):
        self.get_logger().info("ModeSwitcher ready")
        self._broadcast_mode()

        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    ModeSwitcherNode.main(args)
