import asyncio

from std_msgs.msg import Float64
from polyflow_msgs.msg import MotorState
from common.polyflow_node import PolyflowNode


class MotorControllerNode(PolyflowNode):
    """
    Generic motor controller node.

    Receives velocity or position commands on the "command" input pin (Float64)
    and publishes motor state on the "state" output pin (MotorState).

    Parameters (via POLYFLOW_PARAMETERS):
        motor_id:       Identifier for the motor being controlled.
        control_mode:   "velocity" or "position" (default: "velocity").
        max_speed:      Maximum allowable speed value (default: 1.0).
    """

    def __init__(self):
        super().__init__()

        self.motor_id = self.get_param("motor_id", "motor_0")
        self.control_mode = self.get_param("control_mode", "velocity")
        self.max_speed = float(self.get_param("max_speed", 1.0))

        self._current_command = 0.0
        self._connected = False

        self.register_input_pin("command", Float64)
        self.register_output_pin("state", MotorState)

        self.get_logger().info(
            f"MotorController '{self.motor_id}' | mode={self.control_mode} | max_speed={self.max_speed}"
        )

    def process_input(self, pin_id: str, data):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return

        if pin_id == "command":
            self._current_command = max(-self.max_speed, min(self.max_speed, data.data))
            self.get_logger().debug(f"Command received: {self._current_command}")

            # TODO: Send command to actual motor hardware here

            state = MotorState()
            state.motor_id = self.motor_id
            state.control_mode = self.control_mode
            state.command = self._current_command
            state.connected = self._connected
            self.publish_to_pin("state", state)

    async def run_async(self):
        self.get_logger().info(f"MotorController '{self.motor_id}' starting up...")

        # TODO: Initialize hardware connection here
        self._connected = True
        self.get_logger().info(f"MotorController '{self.motor_id}' ready")

        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    MotorControllerNode.main(args)
