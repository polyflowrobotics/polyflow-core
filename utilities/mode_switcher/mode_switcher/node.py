import asyncio

from polyflow_msgs.msg import ModeCommand, ModeState
from common.polyflow_node import PolyflowNode
from common.polyflow_kernel import PolyflowKernel


class ModeSwitcherKernel(PolyflowKernel):
    """
    Portable logic for a mode switcher.

    Manages the rover's operating mode: teleop, automated, or stopped.
    Receives mode change requests on "set_mode" and emits the current mode
    on "mode".

    Parameters:
        default_mode:   Starting mode (default: "stopped").
        allowed_modes:  List of valid modes (default: ["teleop", "automated", "stopped"]).
    """

    VALID_MODES = {"teleop", "automated", "stopped"}

    def setup(self):
        self.allowed_modes = set(self.get_param("allowed_modes", list(self.VALID_MODES)))
        self._current_mode = self.get_param("default_mode", "stopped")

        if self._current_mode not in self.allowed_modes:
            self.log(f"Default mode '{self._current_mode}' not in allowed modes, falling back to 'stopped'")
            self._current_mode = "stopped"

    def broadcast_mode(self):
        self.emit("mode", {"mode": self._current_mode})

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "set_mode":
            requested = data["mode"]

            if requested not in self.allowed_modes:
                self.log(f"Rejected invalid mode '{requested}'")
                return

            if requested != self._current_mode:
                self._current_mode = requested
                self.broadcast_mode()


class ModeSwitcherNode(PolyflowNode):
    """ROS wrapper for ModeSwitcherKernel."""

    kernel_class = ModeSwitcherKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("set_mode", ModeCommand)
        self.register_output_pin("mode", ModeState)

        self.get_logger().info(
            f"ModeSwitcher | mode={self.kernel._current_mode} | allowed={sorted(self.kernel.allowed_modes)}"
        )

    async def run_async(self):
        self.get_logger().info("ModeSwitcher ready")
        self.kernel.broadcast_mode()

        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    ModeSwitcherNode.main(args)
