import time

from common.polyflow_kernel import PolyflowKernel


# Idle keepalive interval. Even when the value hasn't changed, we re-publish at
# this rate so consumers can use message arrival as a liveness signal (e.g. the
# diff drive switches teleop -> automated when teleop goes stale).
KEEPALIVE_INTERVAL_S = 0.2

# Per-axis change threshold (~1% of stick range). Smaller deltas are noise and
# don't warrant a publish on their own — keepalive will pick them up.
AXIS_CHANGE_THRESHOLD = 0.01

# Twist component change threshold in m/s or rad/s. Computed from axes scaled
# by max_*_speed, so this lives in physical units.
TWIST_CHANGE_THRESHOLD = 0.01


class GamepadKernel(PolyflowKernel):
    """
    Portable logic for a gamepad teleop input.

    Applies deadzone filtering and converts raw axis values into
    cmd_vel output. The host wrapper polls the actual gamepad hardware
    and calls emit_gamepad_state() with raw values.

    Output is throttled: a pin re-publishes only when its value changes
    meaningfully or KEEPALIVE_INTERVAL_S has elapsed since the last publish.
    This keeps idle traffic low (~5 Hz instead of poll_rate_hz) while still
    letting downstream consumers detect liveness from message arrival.

    Parameters:
        device_index:       Gamepad device index (default: 0).
        poll_rate_hz:       How often to poll the device (default: 60).
        deadzone:           Axis deadzone threshold (default: 0.05).
        max_linear_speed:   Max linear speed in m/s (default: 1.0).
        max_angular_speed:  Max angular rate in rad/s (default: 2.0).
        output_mode:        cmd_vel mapping: "diff_drive" (default) or "6dof".
                            "diff_drive" fills only linear.x + angular.z.
                            "6dof" maps all four axes plus shoulder/trigger
                            buttons across the full Twist.
    """

    def setup(self):
        self.device_index = int(self.get_param("device_index", 0))
        self.poll_rate_hz = float(self.get_param("poll_rate_hz", 60.0))
        self.deadzone = float(self.get_param("deadzone", 0.05))
        self.max_linear_speed = float(self.get_param("max_linear_speed", 1.0))
        self.max_angular_speed = float(self.get_param("max_angular_speed", 2.0))
        self.output_mode = str(self.get_param("output_mode", "diff_drive"))
        self._connected = False

        self._last_emit_time: dict = {}
        self._last_emit_value: dict = {}

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _maybe_emit(self, pin_id: str, value: dict, changed_fn) -> None:
        """Emit `value` on `pin_id` if it changed or keepalive elapsed."""
        now = time.monotonic()
        last_value = self._last_emit_value.get(pin_id)
        last_time = self._last_emit_time.get(pin_id, 0.0)

        if last_value is None or changed_fn(last_value, value) or (now - last_time) >= KEEPALIVE_INTERVAL_S:
            self._last_emit_value[pin_id] = value
            self._last_emit_time[pin_id] = now
            self.emit(pin_id, value)

    @staticmethod
    def _axes_changed(last: dict, current: dict) -> bool:
        if last.get("connected") != current.get("connected"):
            return True
        for key in ("left_x", "left_y", "right_x", "right_y"):
            if abs(last[key] - current[key]) > AXIS_CHANGE_THRESHOLD:
                return True
        return False

    @staticmethod
    def _buttons_changed(last: dict, current: dict) -> bool:
        return last != current

    @staticmethod
    def _twist_changed(last: dict, current: dict) -> bool:
        for axis in ("linear", "angular"):
            for key in ("x", "y", "z"):
                if abs(last[axis][key] - current[axis][key]) > TWIST_CHANGE_THRESHOLD:
                    return True
        return False

    def emit_gamepad_state(
        self,
        left_x: float = 0.0, left_y: float = 0.0,
        right_x: float = 0.0, right_y: float = 0.0,
        buttons: dict = None,
    ):
        """Emit axes, buttons, and computed cmd_vel from raw gamepad values."""
        axes = {
            "left_x": left_x, "left_y": left_y,
            "right_x": right_x, "right_y": right_y,
            "connected": self._connected,
        }
        self._maybe_emit("axes", axes, self._axes_changed)

        btn = buttons or {}
        button_state = {
            "a": btn.get("a", False), "b": btn.get("b", False),
            "x": btn.get("x", False), "y": btn.get("y", False),
            "lb": btn.get("lb", False), "rb": btn.get("rb", False),
            "start": btn.get("start", False), "select": btn.get("select", False),
            "connected": self._connected,
        }
        self._maybe_emit("buttons", button_state, self._buttons_changed)

        if self.output_mode == "6dof":
            # ROS body-frame conventions: +x forward, +y left, +z up.
            # Browser stick Y is positive when pushed down, so negate.
            rt = 1.0 if btn.get("rt", False) else 0.0
            lt = 1.0 if btn.get("lt", False) else 0.0
            rb = 1.0 if btn.get("rb", False) else 0.0
            lb = 1.0 if btn.get("lb", False) else 0.0
            twist = {
                "linear": {
                    "x": -left_y * self.max_linear_speed,
                    "y": -left_x * self.max_linear_speed,
                    "z": (rt - lt) * self.max_linear_speed,
                },
                "angular": {
                    "x": (rb - lb) * self.max_angular_speed,
                    "y": -right_y * self.max_angular_speed,
                    "z": -right_x * self.max_angular_speed,
                },
            }
        else:
            twist = {
                "linear": {"x": left_y * self.max_linear_speed, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": right_x * self.max_angular_speed},
            }
        self._maybe_emit("cmd_vel", twist, self._twist_changed)

    def process_input(self, pin_id: str, data: dict):
        pass
