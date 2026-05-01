import asyncio
import json
import time
from typing import Optional, Tuple

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from polyflow_msgs.msg import GamepadAxes, GamepadButtons
from common.polyflow_node import PolyflowNode
from gamepad.kernel import GamepadKernel


# Topic Studio publishes onto when relaying operator gamepad state over WebRTC.
RELAY_TOPIC = "/prp/io/gamepad"

# An input source is considered fresh only if updated within this many seconds.
# Tuned for teleop: short enough that local-hardware fallback feels responsive
# when the operator drops, long enough to ride out brief network jitter.
SOURCE_FRESHNESS_S = 0.2


class GamepadNode(PolyflowNode):
    """ROS wrapper for GamepadKernel.

    Reads gamepad state from two parallel sources:
      - Studio relay over /prp/io/gamepad (operator driving remotely from the browser).
      - Local hardware (USB/Bluetooth controller plugged into the robot).

    On each tick at poll_rate_hz, whichever source most recently produced fresh
    data drives the kernel. If neither has produced fresh data within
    SOURCE_FRESHNESS_S, zeros are emitted — never the last value. A stuck
    velocity command after operator disconnect would be a safety problem.
    """

    kernel_class = GamepadKernel

    def __init__(self):
        super().__init__()

        self.register_output_pin("axes", GamepadAxes)
        self.register_output_pin("buttons", GamepadButtons)
        self.register_output_pin("cmd_vel", Twist)

        self._relay_state: Tuple[dict, float] = (self._zero_state(), 0.0)
        self._hardware_state: Tuple[dict, float] = (self._zero_state(), 0.0)

        self._relay_subscription = self.create_subscription(
            String,
            RELAY_TOPIC,
            self._on_relay_message,
            10,
        )

        self.get_logger().info(
            f"Gamepad device={self.kernel.device_index} | poll_rate={self.kernel.poll_rate_hz}Hz | deadzone={self.kernel.deadzone}"
        )
        self.get_logger().info(f"Gamepad relay subscribed: {RELAY_TOPIC}")

    @staticmethod
    def _zero_state() -> dict:
        return {
            "left_x": 0.0, "left_y": 0.0,
            "right_x": 0.0, "right_y": 0.0,
            "buttons": {},
        }

    def _on_relay_message(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Dropping non-JSON gamepad relay payload: {msg.data!r}")
            return

        # Accept both camelCase (browser convention) and snake_case shapes.
        state = {
            "left_x": float(payload.get("leftX", payload.get("left_x", 0.0))),
            "left_y": float(payload.get("leftY", payload.get("left_y", 0.0))),
            "right_x": float(payload.get("rightX", payload.get("right_x", 0.0))),
            "right_y": float(payload.get("rightY", payload.get("right_y", 0.0))),
            "buttons": payload.get("buttons", {}) or {},
        }
        self._relay_state = (state, time.monotonic())

    def _read_hardware(self) -> Optional[dict]:
        """Sample the locally-attached gamepad and return the current state.

        Return None when no controller is attached or no new sample is
        available on this tick. Implementations can either poll synchronously
        here or push samples into self._hardware_state from a background
        coroutine — _select_source only cares about the timestamp.
        """
        # TODO: integrate with evdev / pygame.joystick / hidapi for the
        # specific controller hardware on this robot.
        return None

    def _select_source(self) -> Tuple[dict, str]:
        now = time.monotonic()
        relay_state, relay_ts = self._relay_state
        hw_state, hw_ts = self._hardware_state

        relay_fresh = (now - relay_ts) < SOURCE_FRESHNESS_S
        hw_fresh = (now - hw_ts) < SOURCE_FRESHNESS_S

        if relay_fresh and hw_fresh:
            # Tiebreak: most recent timestamp wins; if equal, relay wins
            # (a remote operator's intent is more deliberate than a controller
            # sitting on a table emitting deadzone noise).
            return (relay_state, "relay") if relay_ts >= hw_ts else (hw_state, "hardware")
        if relay_fresh:
            return relay_state, "relay"
        if hw_fresh:
            return hw_state, "hardware"
        return self._zero_state(), "none"

    async def run_async(self):
        self.get_logger().info("Gamepad starting up...")
        self.kernel._connected = True
        self.get_logger().info("Gamepad ready")

        period = 1.0 / self.kernel.poll_rate_hz
        last_source: Optional[str] = None

        while True:
            if self.should_run():
                hw_sample = self._read_hardware()
                if hw_sample is not None:
                    self._hardware_state = (hw_sample, time.monotonic())

                state, source = self._select_source()
                if source != last_source:
                    self.get_logger().info(f"Gamepad source: {source}")
                    last_source = source

                self.kernel.emit_gamepad_state(
                    left_x=state["left_x"],
                    left_y=state["left_y"],
                    right_x=state["right_x"],
                    right_y=state["right_y"],
                    buttons=state["buttons"],
                )

            await asyncio.sleep(period)


def main(args=None):
    GamepadNode.main(args)
