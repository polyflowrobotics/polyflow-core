"""Background evdev pump for locally-attached gamepads.

BlueZ + the kernel HID layer expose paired Bluetooth controllers under
/dev/input/event* identically to USB pads, so a single evdev backend covers
both transports. The legacy /dev/input/jsN nodes mirror the same controllers;
this module reads only the event devices but numbers them with jsN semantics
(the Nth gamepad in udev enumeration order, skipping keyboards/mice/touchpads)
so the existing device_index parameter behaves intuitively.
"""

import asyncio
from typing import Callable, Optional

try:
    import evdev
    from evdev import ecodes
    _EVDEV_AVAILABLE = True
except ImportError:
    evdev = None
    ecodes = None
    _EVDEV_AVAILABLE = False


# Discovery cadence when no controller is connected. BT pairing/reconnect
# typically takes a second or two; polling faster than this just spams udev.
DISCOVERY_RETRY_S = 1.0

# Analog-trigger threshold (0..1) above which a trigger reports as "pressed"
# in the kernel's boolean button dict. Anything past a third of full pull is
# unambiguous intent.
TRIGGER_BUTTON_THRESHOLD = 0.33


def _button_code_map() -> dict:
    if not _EVDEV_AVAILABLE:
        return {}
    # Linux gamepad button conventions: SOUTH=A (bottom), EAST=B (right),
    # WEST=X (left), NORTH=Y (top) — matches Xbox layout.
    return {
        ecodes.BTN_SOUTH:  "a",
        ecodes.BTN_EAST:   "b",
        ecodes.BTN_WEST:   "x",
        ecodes.BTN_NORTH:  "y",
        ecodes.BTN_TL:     "lb",
        ecodes.BTN_TR:     "rb",
        ecodes.BTN_SELECT: "select",
        ecodes.BTN_START:  "start",
    }


def _is_gamepad(device) -> bool:
    caps = device.capabilities()
    abs_codes = {code for code, _ in caps.get(ecodes.EV_ABS, [])}
    key_codes = set(caps.get(ecodes.EV_KEY, []))
    has_stick = ecodes.ABS_X in abs_codes and ecodes.ABS_Y in abs_codes
    has_gamepad_button = bool(key_codes & {
        ecodes.BTN_SOUTH, ecodes.BTN_EAST, ecodes.BTN_WEST, ecodes.BTN_NORTH,
        ecodes.BTN_START, ecodes.BTN_SELECT,
    })
    return has_stick and has_gamepad_button


def _discover_gamepad(device_index: int):
    paths = sorted(evdev.list_devices())
    matched = 0
    for path in paths:
        try:
            dev = evdev.InputDevice(path)
        except (PermissionError, FileNotFoundError, OSError):
            continue
        try:
            if not _is_gamepad(dev):
                dev.close()
                continue
        except Exception:
            dev.close()
            continue
        if matched == device_index:
            return dev
        matched += 1
        dev.close()
    return None


class EvdevGamepadSource:
    """Background asyncio pump that reads a Linux evdev gamepad.

    Discovers a device by jsN-style index (retrying when none is present),
    decodes events into the dict shape GamepadKernel.emit_gamepad_state
    consumes, and reconnects when a BT controller drops or a USB pad is
    unplugged.

    on_state runs synchronously on the asyncio loop on every input event —
    keep it cheap (store the value, don't block).
    """

    def __init__(
        self,
        device_index: int,
        on_state: Callable[[dict], None],
        logger=None,
    ):
        self._device_index = device_index
        self._on_state = on_state
        self._logger = logger
        self._button_code_map = _button_code_map()
        self._axis_info: dict = {}
        self._stick_state: dict = {}
        self._trigger_state: dict = {}
        self._button_state: dict = {}
        self._reset_state()

    @staticmethod
    def available() -> bool:
        return _EVDEV_AVAILABLE

    def _log_info(self, msg: str):
        if self._logger is not None:
            self._logger.info(msg)

    def _log_warn(self, msg: str):
        if self._logger is not None:
            self._logger.warn(msg)

    def _reset_state(self):
        self._stick_state = {"left_x": 0.0, "left_y": 0.0, "right_x": 0.0, "right_y": 0.0}
        self._trigger_state = {"lt_raw": 0.0, "rt_raw": 0.0}
        self._button_state = {}

    def _normalize_axis(self, code: int, value: int) -> float:
        info = self._axis_info.get(code)
        if info is None:
            return 0.0
        mid = (info.max + info.min) / 2.0
        span = (info.max - info.min) / 2.0
        if span <= 0:
            return 0.0
        return max(-1.0, min(1.0, (value - mid) / span))

    def _normalize_trigger(self, code: int, value: int) -> float:
        info = self._axis_info.get(code)
        if info is None:
            return 0.0
        span = info.max - info.min
        if span <= 0:
            return 0.0
        return max(0.0, min(1.0, (value - info.min) / span))

    def _build_state(self) -> dict:
        buttons = dict(self._button_state)
        buttons["lt"] = self._trigger_state["lt_raw"] >= TRIGGER_BUTTON_THRESHOLD
        buttons["rt"] = self._trigger_state["rt_raw"] >= TRIGGER_BUTTON_THRESHOLD
        return {**self._stick_state, "buttons": buttons}

    def _handle_event(self, event) -> bool:
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X:
                self._stick_state["left_x"] = self._normalize_axis(event.code, event.value)
            elif event.code == ecodes.ABS_Y:
                self._stick_state["left_y"] = self._normalize_axis(event.code, event.value)
            elif event.code == ecodes.ABS_RX:
                self._stick_state["right_x"] = self._normalize_axis(event.code, event.value)
            elif event.code == ecodes.ABS_RY:
                self._stick_state["right_y"] = self._normalize_axis(event.code, event.value)
            elif event.code == ecodes.ABS_Z:
                self._trigger_state["lt_raw"] = self._normalize_trigger(event.code, event.value)
            elif event.code == ecodes.ABS_RZ:
                self._trigger_state["rt_raw"] = self._normalize_trigger(event.code, event.value)
            else:
                return False
            return True
        if event.type == ecodes.EV_KEY:
            name = self._button_code_map.get(event.code)
            if name is None:
                return False
            self._button_state[name] = bool(event.value)
            return True
        return False

    def _load_axis_info(self, device):
        self._axis_info = {}
        for code, info in device.capabilities().get(ecodes.EV_ABS, []):
            self._axis_info[code] = info

    async def run(self):
        if not _EVDEV_AVAILABLE:
            self._log_warn("evdev not installed; local gamepad disabled")
            return
        while True:
            device = _discover_gamepad(self._device_index)
            if device is None:
                await asyncio.sleep(DISCOVERY_RETRY_S)
                continue
            self._log_info(f"Gamepad hardware connected: {device.path} ({device.name})")
            self._load_axis_info(device)
            self._reset_state()
            try:
                async for event in device.async_read_loop():
                    if self._handle_event(event):
                        self._on_state(self._build_state())
            except (OSError, IOError) as e:
                # Disconnect (BT drop, USB unplug) surfaces as ENODEV.
                self._log_warn(f"Gamepad hardware disconnected: {e}")
            finally:
                try:
                    device.close()
                except Exception:
                    pass
                self._reset_state()
            # Loop back into discovery without emitting zeros — the node's
            # freshness gate will fall back to the relay (or zeros) on its own.
