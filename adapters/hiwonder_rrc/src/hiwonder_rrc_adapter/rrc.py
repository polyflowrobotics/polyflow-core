"""
Driver for Hiwonder RRC Lite board over USB serial.

Speaks the RRC binary protocol: [0xAA, 0x55, func, data_len, ...data, crc8]
Supports motor speed/duty control, buzzer, LEDs, RGB LEDs, and receiving
IMU, battery, button, and servo feedback from the board.

Protocol references:
  - github.com/dmberezovskyii/fast-hiwonder (send side)
  - github.com/Matzefritz/HiWonder_MentorPi (receive side)
"""

import enum
import queue
import struct
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

import serial

# --- Function codes ---

class Func(enum.IntEnum):
    SYS = 0
    LED = 1
    BUZZER = 2
    MOTOR = 3
    PWM_SERVO = 4
    BUS_SERVO = 5
    KEY = 6
    IMU = 7
    GAMEPAD = 8
    SBUS = 9
    OLED = 10
    RGB = 11
    NONE = 12  # sentinel — anything >= this is invalid


# --- Motor sub-commands ---
MOTOR_SUB_SPEED = 0x01
MOTOR_SUB_DUTY = 0x05

# --- Protocol constants ---
MAGIC_1 = 0xAA
MAGIC_2 = 0x55

# --- Receive state machine ---

class _RxState(enum.IntEnum):
    STARTBYTE1 = 0
    STARTBYTE2 = 1
    FUNCTION = 2
    LENGTH = 3
    DATA = 4
    CHECKSUM = 5


# fmt: off
_CRC8_TABLE = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
]
# fmt: on


def _crc8(data: bytes) -> int:
    check = 0
    for b in data:
        check = _CRC8_TABLE[check ^ b]
    return check


class HiwonderRRC:
    """
    Driver for the Hiwonder RRC Lite board.

    Handles both sending commands and receiving reports (IMU, battery, keys,
    servos) over USB serial. A background thread continuously parses incoming
    frames and routes them to per-function queues.

    Args:
        port: Serial device path (default: /dev/ttyAMA0).
        baudrate: Baud rate (default: 1000000).
        timeout: Serial read timeout in seconds (default: 0.1).
    """

    def __init__(
        self,
        port: str = "/dev/ttyAMA0",
        baudrate: int = 1_000_000,
        timeout: float = 0.1,
    ):
        self._port_path = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._write_lock = threading.Lock()

        # --- Receive state machine ---
        self._rx_state = _RxState.STARTBYTE1
        self._rx_frame: List[int] = []
        self._rx_count = 0
        self._rx_running = False
        self._rx_thread: Optional[threading.Thread] = None

        # --- Per-function receive queues ---
        self._queues: Dict[Func, queue.Queue] = {
            Func.SYS: queue.Queue(maxsize=4),
            Func.KEY: queue.Queue(maxsize=8),
            Func.IMU: queue.Queue(maxsize=4),
            Func.GAMEPAD: queue.Queue(maxsize=4),
            Func.BUS_SERVO: queue.Queue(maxsize=8),
            Func.PWM_SERVO: queue.Queue(maxsize=8),
            Func.SBUS: queue.Queue(maxsize=4),
        }

        # --- Optional user callbacks (called from rx thread) ---
        self.on_imu: Optional[Callable[[Tuple[float, ...]], None]] = None
        self.on_battery: Optional[Callable[[int], None]] = None
        self.on_key: Optional[Callable[[int, int], None]] = None

    # --- Connection lifecycle ---

    def open(self):
        """Open the serial connection and start the receive thread."""
        if self._serial is not None and self._serial.is_open:
            return
        self._serial = serial.Serial(
            port=self._port_path,
            baudrate=self._baudrate,
            timeout=self._timeout,
        )
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._rx_thread.start()

    def close(self):
        """Stop the receive thread and close the serial connection."""
        self._rx_running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            self._serial = None

    # --- Send ---

    def _send(self, func: int, data: List[int]):
        """Send a raw command frame to the board."""
        buf = bytearray([MAGIC_1, MAGIC_2, func, len(data)])
        buf.extend(data)
        buf.append(_crc8(bytes(buf[2:])))
        with self._write_lock:
            if self._serial is None or not self._serial.is_open:
                raise RuntimeError("Serial port not open")
            self._serial.write(bytes(buf))

    # --- Receive ---

    def _recv_loop(self):
        """Background thread: read bytes and feed through the frame state machine."""
        while self._rx_running:
            if self._serial is None or not self._serial.is_open:
                time.sleep(0.01)
                continue
            try:
                data = self._serial.read(64)
            except serial.SerialException:
                time.sleep(0.01)
                continue
            if not data:
                continue
            for byte in data:
                self._rx_feed(byte)

    def _rx_feed(self, byte: int):
        """Process one byte through the receive state machine."""
        if self._rx_state == _RxState.STARTBYTE1:
            if byte == MAGIC_1:
                self._rx_state = _RxState.STARTBYTE2

        elif self._rx_state == _RxState.STARTBYTE2:
            if byte == MAGIC_2:
                self._rx_state = _RxState.FUNCTION
            else:
                self._rx_state = _RxState.STARTBYTE1

        elif self._rx_state == _RxState.FUNCTION:
            if byte < Func.NONE:
                self._rx_frame = [byte, 0]
                self._rx_state = _RxState.LENGTH
            else:
                self._rx_state = _RxState.STARTBYTE1

        elif self._rx_state == _RxState.LENGTH:
            self._rx_frame[1] = byte
            self._rx_count = 0
            if byte == 0:
                self._rx_state = _RxState.CHECKSUM
            else:
                self._rx_state = _RxState.DATA

        elif self._rx_state == _RxState.DATA:
            self._rx_frame.append(byte)
            self._rx_count += 1
            if self._rx_count >= self._rx_frame[1]:
                self._rx_state = _RxState.CHECKSUM

        elif self._rx_state == _RxState.CHECKSUM:
            expected = _crc8(bytes(self._rx_frame))
            if expected == byte:
                self._dispatch(self._rx_frame)
            self._rx_state = _RxState.STARTBYTE1

    def _dispatch(self, frame: List[int]):
        """Route a validated frame to the appropriate queue and callback."""
        try:
            func = Func(frame[0])
        except ValueError:
            return
        data = bytes(frame[2:])

        # Enqueue for polling-style access
        q = self._queues.get(func)
        if q is not None:
            try:
                q.put_nowait(data)
            except queue.Full:
                try:
                    q.get_nowait()  # drop oldest
                except queue.Empty:
                    pass
                try:
                    q.put_nowait(data)
                except queue.Full:
                    pass

        # Fire callbacks
        if func == Func.IMU and self.on_imu is not None and len(data) >= 24:
            self.on_imu(struct.unpack("<6f", data[:24]))
        elif func == Func.SYS and self.on_battery is not None and len(data) >= 3:
            if data[0] == 0x04:
                self.on_battery(struct.unpack("<H", data[1:3])[0])
        elif func == Func.KEY and self.on_key is not None and len(data) >= 2:
            self.on_key(data[0], data[1])

    # --- Polling read methods ---

    def get_imu(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        Read the latest IMU report (non-blocking).

        Returns:
            (ax, ay, az, gx, gy, gz) or None if no data available.
        """
        try:
            data = self._queues[Func.IMU].get_nowait()
            if len(data) >= 24:
                return struct.unpack("<6f", data[:24])
        except queue.Empty:
            pass
        return None

    def get_battery(self) -> Optional[int]:
        """
        Read the latest battery voltage in mV (non-blocking).

        Returns:
            Voltage in millivolts, or None if no data available.
        """
        try:
            data = self._queues[Func.SYS].get_nowait()
            if len(data) >= 3 and data[0] == 0x04:
                return struct.unpack("<H", data[1:3])[0]
        except queue.Empty:
            pass
        return None

    def get_key_event(self) -> Optional[Tuple[int, int]]:
        """
        Read the latest button event (non-blocking).

        Returns:
            (key_id, event_type) or None if no data available.
        """
        try:
            data = self._queues[Func.KEY].get_nowait()
            if len(data) >= 2:
                return (data[0], data[1])
        except queue.Empty:
            pass
        return None

    # --- Motor commands ---

    def set_motor_speed(self, motors: List[Tuple[int, float]]):
        """
        Set motor speeds (closed-loop PID control on the board).

        Args:
            motors: List of (motor_id, speed_rps) tuples.
                    motor_id is 1-indexed (1-4).
                    speed_rps is rotations per second.
        """
        data = bytearray([MOTOR_SUB_SPEED, len(motors)])
        for motor_id, speed in motors:
            data.extend(struct.pack("<Bf", motor_id - 1, float(speed)))
        self._send(Func.MOTOR, list(data))

    def set_motor_duty(self, motors: List[Tuple[int, float]]):
        """
        Set motor duty cycles (open-loop PWM control).

        Args:
            motors: List of (motor_id, duty) tuples.
                    motor_id is 1-indexed (1-4).
                    duty is -100.0 to 100.0.
        """
        data = bytearray([MOTOR_SUB_DUTY, len(motors)])
        for motor_id, duty in motors:
            data.extend(struct.pack("<Bf", motor_id - 1, float(duty)))
        self._send(Func.MOTOR, list(data))

    def stop_all_motors(self):
        """Stop all 4 motors."""
        self.set_motor_speed([(1, 0.0), (2, 0.0), (3, 0.0), (4, 0.0)])

    # --- PWM Servo commands ---

    def set_pwm_servo(self, duration_ms: int, servos: List[Tuple[int, int]]):
        """
        Move PWM servos to target positions.

        Args:
            duration_ms: Movement duration in milliseconds.
            servos: List of (servo_id, pulse_width) tuples.
                    servo_id is 1-indexed (1-4).
                    pulse_width is typically 500-2500 (microseconds).
        """
        data = bytearray([0x01, len(servos)])
        data.extend(struct.pack("<H", duration_ms))
        for servo_id, pulse in servos:
            data.extend(struct.pack("<BH", servo_id, pulse))
        self._send(Func.PWM_SERVO, list(data))

    # --- Bus Servo commands ---

    def set_bus_servo_position(self, servo_id: int, position: int, duration_ms: int):
        """
        Move a bus servo (LX-series) to a position.

        Args:
            servo_id: Servo ID (1-253).
            position: Target position (0-1000).
            duration_ms: Movement duration in milliseconds (0-30000).
        """
        data = [0x01, 1]
        data.extend(struct.pack("<BHH", servo_id, position, duration_ms))
        self._send(Func.BUS_SERVO, list(data))

    def read_bus_servo_position(self, servo_id: int) -> Optional[int]:
        """
        Read current position of a bus servo (blocking, with timeout).

        Args:
            servo_id: Servo ID to query.

        Returns:
            Position (0-1000) or None if read failed.
        """
        # Drain stale data
        while not self._queues[Func.BUS_SERVO].empty():
            try:
                self._queues[Func.BUS_SERVO].get_nowait()
            except queue.Empty:
                break

        # Send read command (sub_cmd 0x02 = read position)
        data = [0x02, servo_id]
        self._send(Func.BUS_SERVO, data)

        try:
            resp = self._queues[Func.BUS_SERVO].get(timeout=0.1)
            if len(resp) >= 4 and resp[0] == servo_id and resp[1] == 0x02:
                return struct.unpack("<h", resp[2:4])[0]
        except queue.Empty:
            pass
        return None

    # --- Buzzer ---

    def set_buzzer(self, freq_hz: int, on_ms: int, off_ms: int, repeat: int = 1):
        """
        Play a tone on the buzzer.

        Args:
            freq_hz: Frequency in Hz (0 to turn off).
            on_ms: On duration in milliseconds.
            off_ms: Off duration in milliseconds.
            repeat: Number of repetitions.
        """
        data = list(struct.pack("<HHHH", freq_hz, on_ms, off_ms, repeat))
        self._send(Func.BUZZER, data)

    # --- LEDs ---

    def set_led(self, led_id: int, on_ms: int, off_ms: int, repeat: int = 1):
        """
        Blink an onboard LED.

        Args:
            led_id: LED index (1-based).
            on_ms: On duration in milliseconds.
            off_ms: Off duration in milliseconds.
            repeat: Number of repetitions (0 = infinite).
        """
        data = [led_id]
        data.extend(struct.pack("<HHH", on_ms, off_ms, repeat))
        self._send(Func.LED, data)

    def set_rgb(self, pixels: List[Tuple[int, int, int, int]]):
        """
        Set RGB LED colors.

        Args:
            pixels: List of (index, r, g, b) tuples.
                    index is 1-based. r, g, b are 0-255.
        """
        data = bytearray([0x01, len(pixels)])
        for index, r, g, b in pixels:
            data.extend([index - 1, r, g, b])
        self._send(Func.RGB, list(data))

    # --- Context manager ---

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()
