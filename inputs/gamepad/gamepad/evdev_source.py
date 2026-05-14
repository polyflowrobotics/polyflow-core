"""Background gamepad pump that talks to the Linux evdev interface directly.

This module bypasses the `python-evdev` package: prebuilt wheels aren't
available for aarch64/cp312 on PyPI or piwheels, and the sdist build was
failing on the target. Instead we open /dev/input/event* ourselves, query
capabilities with ioctl, and parse the 24-byte input_event struct off the fd.

BlueZ + the kernel HID layer expose paired Bluetooth controllers under
/dev/input/event* identically to USB pads, so a single backend covers both
transports. The legacy /dev/input/jsN nodes mirror the same controllers; we
read only the event devices but number them with jsN semantics (the Nth
gamepad in enumeration order, skipping keyboards/mice/touchpads) so the
existing device_index parameter behaves intuitively.

Assumes a 64-bit Linux kernel (the only platform that has /dev/input/event*
and on which this node runs).
"""

import asyncio
import errno
import fcntl
import os
import struct
from collections import namedtuple
from typing import Callable, List, Optional


# --- Linux input event constants (from linux/input-event-codes.h) ------------

EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03

ABS_X  = 0x00
ABS_Y  = 0x01
ABS_Z  = 0x02
ABS_RX = 0x03
ABS_RY = 0x04
ABS_RZ = 0x05

BTN_SOUTH  = 0x130
BTN_EAST   = 0x131
BTN_NORTH  = 0x133
BTN_WEST   = 0x134
BTN_TL     = 0x136
BTN_TR     = 0x137
BTN_SELECT = 0x13a
BTN_START  = 0x13b

KEY_CNT = 0x300
ABS_CNT = 0x40

# Linux gamepad button conventions: SOUTH=A (bottom), EAST=B (right),
# WEST=X (left), NORTH=Y (top) — matches Xbox layout.
_BUTTON_CODE_MAP = {
    BTN_SOUTH:  "a",
    BTN_EAST:   "b",
    BTN_WEST:   "x",
    BTN_NORTH:  "y",
    BTN_TL:     "lb",
    BTN_TR:     "rb",
    BTN_SELECT: "select",
    BTN_START:  "start",
}

_GAMEPAD_FACE_BUTTONS = {BTN_SOUTH, BTN_EAST, BTN_WEST, BTN_NORTH, BTN_START, BTN_SELECT}


# --- ioctl request encoding (linux/asm-generic/ioctl.h) ----------------------
# aarch64, x86, x86_64, and arm all share the generic encoding.

_IOC_NRBITS    = 8
_IOC_TYPEBITS  = 8
_IOC_SIZEBITS  = 14
_IOC_NRSHIFT   = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT  = _IOC_SIZESHIFT + _IOC_SIZEBITS
_IOC_READ      = 2


def _IOR(type_char: str, nr: int, size: int) -> int:
    return (
        (_IOC_READ << _IOC_DIRSHIFT)
        | (size << _IOC_SIZESHIFT)
        | (ord(type_char) << _IOC_TYPESHIFT)
        | (nr << _IOC_NRSHIFT)
    )


def _EVIOCGNAME(length: int) -> int:
    return _IOR('E', 0x06, length)


def _EVIOCGBIT(ev_type: int, length: int) -> int:
    return _IOR('E', 0x20 + ev_type, length)


def _EVIOCGABS(abs_code: int) -> int:
    return _IOR('E', 0x40 + abs_code, _ABSINFO_STRUCT.size)


# struct input_event on 64-bit Linux: timeval (long sec, long usec) + u16 + u16 + s32
_EVENT_STRUCT = struct.Struct('@llHHi')
assert _EVENT_STRUCT.size == 24, "input_event size assumption broken (need 64-bit Linux)"

# struct input_absinfo: six int32s (value, min, max, fuzz, flat, resolution)
_ABSINFO_STRUCT = struct.Struct('@iiiiii')

AbsInfo = namedtuple("AbsInfo", ["value", "min", "max", "fuzz", "flat", "resolution"])


# Discovery cadence when no controller is connected. BT pairing/reconnect
# typically takes a second or two; polling faster than this just spams udev.
DISCOVERY_RETRY_S = 1.0

# Analog-trigger threshold (0..1) above which a trigger reports as "pressed"
# in the kernel's boolean button dict. Anything past a third of full pull is
# unambiguous intent.
TRIGGER_BUTTON_THRESHOLD = 0.33


def _has_bit(bits: bytes, code: int) -> bool:
    return bool(bits[code >> 3] & (1 << (code & 7)))


def _query_bits(fd: int, ev_type: int, max_code: int) -> bytes:
    n_bytes = (max_code + 7) // 8
    buf = bytearray(n_bytes)
    fcntl.ioctl(fd, _EVIOCGBIT(ev_type, n_bytes), buf)
    return bytes(buf)


def _query_name(fd: int) -> str:
    buf = bytearray(256)
    try:
        fcntl.ioctl(fd, _EVIOCGNAME(len(buf)), buf)
    except OSError:
        return ""
    return buf.split(b'\x00', 1)[0].decode('utf-8', errors='replace')


def _query_absinfo(fd: int, code: int) -> Optional[AbsInfo]:
    buf = bytearray(_ABSINFO_STRUCT.size)
    try:
        fcntl.ioctl(fd, _EVIOCGABS(code), buf)
    except OSError:
        return None
    return AbsInfo(*_ABSINFO_STRUCT.unpack_from(buf, 0))


class _Device:
    __slots__ = ("fd", "path", "name", "abs_info")

    def __init__(self, fd: int, path: str, name: str, abs_info: dict):
        self.fd = fd
        self.path = path
        self.name = name
        self.abs_info = abs_info

    def close(self):
        if self.fd >= 0:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = -1


def _list_event_paths() -> List[str]:
    try:
        entries = os.listdir('/dev/input')
    except FileNotFoundError:
        return []
    paths = []
    for name in entries:
        if not name.startswith('event'):
            continue
        try:
            idx = int(name[len('event'):])
        except ValueError:
            continue
        paths.append((idx, '/dev/input/' + name))
    paths.sort()
    return [p for _, p in paths]


def _is_gamepad(key_bits: bytes, abs_bits: bytes) -> bool:
    has_stick = _has_bit(abs_bits, ABS_X) and _has_bit(abs_bits, ABS_Y)
    has_gamepad_button = any(_has_bit(key_bits, b) for b in _GAMEPAD_FACE_BUTTONS)
    return has_stick and has_gamepad_button


def _try_open_gamepad(path: str) -> Optional[_Device]:
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        key_bits = _query_bits(fd, EV_KEY, KEY_CNT)
        abs_bits = _query_bits(fd, EV_ABS, ABS_CNT)
    except OSError:
        os.close(fd)
        return None
    if not _is_gamepad(key_bits, abs_bits):
        os.close(fd)
        return None
    abs_info = {}
    for code in (ABS_X, ABS_Y, ABS_RX, ABS_RY, ABS_Z, ABS_RZ):
        if _has_bit(abs_bits, code):
            info = _query_absinfo(fd, code)
            if info is not None:
                abs_info[code] = info
    name = _query_name(fd)
    return _Device(fd=fd, path=path, name=name, abs_info=abs_info)


def _discover_gamepad(device_index: int) -> Optional[_Device]:
    matched = 0
    for path in _list_event_paths():
        device = _try_open_gamepad(path)
        if device is None:
            continue
        if matched == device_index:
            return device
        matched += 1
        device.close()
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
        self._stick_state: dict = {}
        self._trigger_state: dict = {}
        self._button_state: dict = {}
        self._reset_state()

    @staticmethod
    def available() -> bool:
        # /dev/input/event* is the only thing we need; if the directory exists
        # the kernel has the evdev interface (true on every modern Linux).
        return os.path.isdir('/dev/input')

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

    def _normalize_axis(self, info: AbsInfo, value: int) -> float:
        mid = (info.max + info.min) / 2.0
        span = (info.max - info.min) / 2.0
        if span <= 0:
            return 0.0
        return max(-1.0, min(1.0, (value - mid) / span))

    def _normalize_trigger(self, info: AbsInfo, value: int) -> float:
        span = info.max - info.min
        if span <= 0:
            return 0.0
        return max(0.0, min(1.0, (value - info.min) / span))

    def _build_state(self) -> dict:
        buttons = dict(self._button_state)
        buttons["lt"] = self._trigger_state["lt_raw"] >= TRIGGER_BUTTON_THRESHOLD
        buttons["rt"] = self._trigger_state["rt_raw"] >= TRIGGER_BUTTON_THRESHOLD
        return {**self._stick_state, "buttons": buttons}

    def _handle_event(self, device: _Device, ev_type: int, ev_code: int, ev_value: int) -> bool:
        if ev_type == EV_ABS:
            info = device.abs_info.get(ev_code)
            if info is None:
                return False
            if ev_code == ABS_X:
                self._stick_state["left_x"] = self._normalize_axis(info, ev_value)
            elif ev_code == ABS_Y:
                self._stick_state["left_y"] = self._normalize_axis(info, ev_value)
            elif ev_code == ABS_RX:
                self._stick_state["right_x"] = self._normalize_axis(info, ev_value)
            elif ev_code == ABS_RY:
                self._stick_state["right_y"] = self._normalize_axis(info, ev_value)
            elif ev_code == ABS_Z:
                self._trigger_state["lt_raw"] = self._normalize_trigger(info, ev_value)
            elif ev_code == ABS_RZ:
                self._trigger_state["rt_raw"] = self._normalize_trigger(info, ev_value)
            else:
                return False
            return True
        if ev_type == EV_KEY:
            name = _BUTTON_CODE_MAP.get(ev_code)
            if name is None:
                return False
            self._button_state[name] = bool(ev_value)
            return True
        return False

    def _drain_events(self, device: _Device, disconnect_event: asyncio.Event):
        try:
            data = os.read(device.fd, 4096)
        except BlockingIOError:
            return
        except OSError as e:
            # Disconnect (BT drop, USB unplug) surfaces as ENODEV.
            if e.errno != errno.EAGAIN:
                self._log_warn(f"Gamepad hardware disconnected: {e}")
                disconnect_event.set()
            return
        if not data:
            disconnect_event.set()
            return
        changed = False
        for offset in range(0, len(data) - _EVENT_STRUCT.size + 1, _EVENT_STRUCT.size):
            _sec, _usec, ev_type, ev_code, ev_value = _EVENT_STRUCT.unpack_from(data, offset)
            if self._handle_event(device, ev_type, ev_code, ev_value):
                changed = True
        if changed:
            self._on_state(self._build_state())

    async def run(self):
        loop = asyncio.get_running_loop()
        while True:
            device = _discover_gamepad(self._device_index)
            if device is None:
                await asyncio.sleep(DISCOVERY_RETRY_S)
                continue
            self._log_info(f"Gamepad hardware connected: {device.path} ({device.name})")
            self._reset_state()
            disconnect_event = asyncio.Event()
            loop.add_reader(device.fd, self._drain_events, device, disconnect_event)
            try:
                await disconnect_event.wait()
            finally:
                try:
                    loop.remove_reader(device.fd)
                except Exception:
                    pass
                device.close()
                self._reset_state()
            # Loop back into discovery without emitting zeros — the node's
            # freshness gate will fall back to the relay (or zeros) on its own.
