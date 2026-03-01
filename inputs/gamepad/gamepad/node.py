import asyncio

from geometry_msgs.msg import Twist
from polyflow_msgs.msg import GamepadAxes, GamepadButtons
from common.polyflow_node import PolyflowNode


class GamepadNode(PolyflowNode):
    """
    Gamepad teleop input node.

    Polls a connected gamepad/joystick at a configurable rate and publishes
    axis state on the "axes" output pin (GamepadAxes) and button state on
    the "buttons" output pin (GamepadButtons).

    Parameters (via POLYFLOW_PARAMETERS):
        device_index:       Gamepad device index (default: 0).
        poll_rate_hz:       How often to poll the device (default: 60).
        deadzone:           Axis deadzone threshold (default: 0.05).
        max_linear_speed:   Max forward/backward speed in m/s for cmd_vel (default: 1.0).
        max_angular_speed:  Max turning rate in rad/s for cmd_vel (default: 2.0).
    """

    def __init__(self):
        super().__init__()

        self.device_index = int(self.get_param("device_index", 0))
        self.poll_rate_hz = float(self.get_param("poll_rate_hz", 60.0))
        self.deadzone = float(self.get_param("deadzone", 0.05))
        self.max_linear_speed = float(self.get_param("max_linear_speed", 1.0))
        self.max_angular_speed = float(self.get_param("max_angular_speed", 2.0))

        self._connected = False

        self.register_output_pin("axes", GamepadAxes)
        self.register_output_pin("buttons", GamepadButtons)
        self.register_output_pin("cmd_vel", Twist)

        self.get_logger().info(
            f"Gamepad device={self.device_index} | poll_rate={self.poll_rate_hz}Hz | deadzone={self.deadzone}"
        )

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def process_input(self, pin_id: str, data):
        pass

    async def run_async(self):
        self.get_logger().info("Gamepad starting up...")

        # TODO: Initialize gamepad connection here
        # Example with pygame:
        #   import pygame
        #   pygame.joystick.init()
        #   joystick = pygame.joystick.Joystick(self.device_index)
        #   joystick.init()
        self._connected = True
        self.get_logger().info("Gamepad ready")

        period = 1.0 / self.poll_rate_hz
        while True:
            if self.should_run():
                # TODO: Read actual gamepad state here

                axes = GamepadAxes()
                axes.left_x = 0.0
                axes.left_y = 0.0
                axes.right_x = 0.0
                axes.right_y = 0.0
                axes.connected = self._connected
                self.publish_to_pin("axes", axes)

                buttons = GamepadButtons()
                buttons.a = False
                buttons.b = False
                buttons.x = False
                buttons.y = False
                buttons.lb = False
                buttons.rb = False
                buttons.start = False
                buttons.select = False
                buttons.connected = self._connected
                self.publish_to_pin("buttons", buttons)

                # Publish Twist for direct connection to differential drive
                twist = Twist()
                twist.linear.x = axes.left_y * self.max_linear_speed
                twist.angular.z = axes.right_x * self.max_angular_speed
                self.publish_to_pin("cmd_vel", twist)

            await asyncio.sleep(period)


def main(args=None):
    GamepadNode.main(args)
