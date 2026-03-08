import asyncio

from std_msgs.msg import Float64
from polyflow_msgs.msg import MotorState
from common.polyflow_node import PolyflowNode
from common.polyflow_kernel import PolyflowKernel


class MotorControllerKernel(PolyflowKernel):
    """
    Portable logic for a generic motor controller.

    Receives velocity or position commands on "command" and emits motor state
    on "state". The actual hardware connection is managed by the host wrapper.

    Parameters:
        motor_id:       Identifier for the motor being controlled.
        control_mode:   "velocity" or "position" (default: "velocity").
        max_speed:      Maximum allowable speed value (default: 1.0).
    """

    def setup(self):
        self.motor_id = self.get_param("motor_id", "motor_0")
        self.control_mode = self.get_param("control_mode", "velocity")
        self.max_speed = float(self.get_param("max_speed", 1.0))
        self._current_command = 0.0
        self._connected = False

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "command":
            self._current_command = max(-self.max_speed, min(self.max_speed, data["data"]))

            self.emit("state", {
                "motor_id": self.motor_id,
                "control_mode": self.control_mode,
                "command": self._current_command,
                "connected": self._connected,
            })


class MotorControllerNode(PolyflowNode):
    """
    ROS wrapper for MotorControllerKernel.

    Handles pin registration, hardware lifecycle, and ROS transport.
    """

    kernel_class = MotorControllerKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("command", Float64)
        self.register_output_pin("state", MotorState)

        self.get_logger().info(
            f"MotorController '{self.kernel.motor_id}' | mode={self.kernel.control_mode} | max_speed={self.kernel.max_speed}"
        )

    async def run_async(self):
        self.get_logger().info(f"MotorController '{self.kernel.motor_id}' starting up...")

        # TODO: Initialize hardware connection here
        self.kernel._connected = True
        self.get_logger().info(f"MotorController '{self.kernel.motor_id}' ready")

        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    MotorControllerNode.main(args)
