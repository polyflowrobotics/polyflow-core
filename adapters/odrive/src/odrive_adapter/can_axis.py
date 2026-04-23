"""
CANSimple transport for a single ODrive axis.

Lifted from polyflow-core/controllers/odrive_s1/odrive_s1/node.py as part
of the adapter extraction. Depends only on python-can and stdlib.
"""

import logging
import math
import struct
import threading
import time
from typing import Any, Dict, Optional


logger = logging.getLogger(__name__)


def _odrive_cansimple_arbitration_id(node_id: int, command_id: int) -> int:
    # CANSimple uses 11-bit standard IDs:
    #   arbitration_id = (node_id << 5) | command_id
    return (int(node_id) << 5) | (int(command_id) & 0x1F)


def _now_monotonic_s() -> float:
    return time.monotonic()


class _CANEncoder:
    def __init__(self) -> None:
        self.pos_estimate = 0.0
        self.vel_estimate = 0.0


class _CANMotorFOC:
    def __init__(self) -> None:
        self.Iq_measured = 0.0


class _CANMotor:
    def __init__(self, torque_constant: Optional[float] = None) -> None:
        self.torque_constant = torque_constant
        self.foc = _CANMotorFOC()


class _CANControllerConfig:
    def __init__(self) -> None:
        self.vel_limit: Optional[float] = None


class _CANController:
    def __init__(self, axis: "CANSimpleAxis") -> None:
        self._axis = axis
        self.config = _CANControllerConfig()

    @property
    def input_pos(self) -> float:
        return float(self._axis._last_command_pos_turns or 0.0)

    @input_pos.setter
    def input_pos(self, value: float) -> None:
        self._axis.set_input_pos(float(value))

    @property
    def input_vel(self) -> float:
        return float(self._axis._last_command_vel_turns_s or 0.0)

    @input_vel.setter
    def input_vel(self, value: float) -> None:
        self._axis.set_input_vel(float(value))

    @property
    def input_torque(self) -> float:
        return float(self._axis._last_command_torque_nm or 0.0)

    @input_torque.setter
    def input_torque(self, value: float) -> None:
        self._axis.set_input_torque(float(value))


class CANSimpleAxis:
    """
    Minimal CANSimple transport for a single ODrive axis / node_id.

    Requires the `python-can` package and a configured CAN interface
    (e.g. SocketCAN `can0`).
    """

    # CANSimple command IDs (5-bit). Defaults match ODrive CANSimple v0.x.
    CMD_HEARTBEAT = 0x001
    CMD_SET_AXIS_STATE = 0x007
    CMD_GET_ENCODER_ESTIMATES = 0x009
    CMD_SET_CONTROLLER_MODE = 0x00B
    CMD_SET_INPUT_POS = 0x00C
    CMD_SET_INPUT_VEL = 0x00D
    CMD_SET_INPUT_TORQUE = 0x00E
    CMD_GET_IQ = 0x014

    AXIS_STATE_IDLE = 1
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8

    CONTROL_MODE_TORQUE_CONTROL = 1
    CONTROL_MODE_VELOCITY_CONTROL = 2
    CONTROL_MODE_POSITION_CONTROL = 3

    INPUT_MODE_PASSTHROUGH = 1

    def __init__(
        self,
        *,
        node_id: int,
        can_interface: str,
        can_channel: str,
        can_bitrate: Optional[int] = None,
        torque_constant: Optional[float] = None,
        command_ids: Optional[Dict[str, int]] = None,
    ) -> None:
        self.node_id = int(node_id)
        self.encoder = _CANEncoder()
        self.motor = _CANMotor(torque_constant=torque_constant)
        self.controller = _CANController(self)

        self._last_heartbeat_error: Optional[int] = None
        self._last_heartbeat_state: Optional[int] = None
        self._last_heartbeat_time_s: Optional[float] = None

        self._last_command_pos_turns: Optional[float] = None
        self._last_command_vel_turns_s: Optional[float] = None
        self._last_command_torque_nm: Optional[float] = None

        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        if command_ids:
            self.CMD_SET_AXIS_STATE = int(command_ids.get("set_axis_state", self.CMD_SET_AXIS_STATE))
            self.CMD_GET_ENCODER_ESTIMATES = int(command_ids.get("get_encoder_estimates", self.CMD_GET_ENCODER_ESTIMATES))
            self.CMD_SET_CONTROLLER_MODE = int(command_ids.get("set_controller_mode", self.CMD_SET_CONTROLLER_MODE))
            self.CMD_SET_INPUT_POS = int(command_ids.get("set_input_pos", self.CMD_SET_INPUT_POS))
            self.CMD_SET_INPUT_VEL = int(command_ids.get("set_input_vel", self.CMD_SET_INPUT_VEL))
            self.CMD_SET_INPUT_TORQUE = int(command_ids.get("set_input_torque", self.CMD_SET_INPUT_TORQUE))
            self.CMD_GET_IQ = int(command_ids.get("get_iq", self.CMD_GET_IQ))

        import can  # type: ignore

        bus_kwargs: Dict[str, Any] = {"interface": can_interface, "channel": can_channel}
        if can_bitrate is not None:
            bus_kwargs["bitrate"] = int(can_bitrate)
        self._bus = can.Bus(**bus_kwargs)

        self._rx_thread = threading.Thread(target=self._rx_loop, name="odrive-can-rx", daemon=True)
        self._rx_thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        try:
            self._rx_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self._bus.shutdown()
        except Exception:
            pass

    def _send(self, command_id: int, data: bytes = b"") -> None:
        import can  # type: ignore

        arb_id = _odrive_cansimple_arbitration_id(self.node_id, command_id)
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        with self._lock:
            try:
                self._bus.send(msg)
            except Exception as exc:
                logger.warning(f"Failed to send CAN message: {exc}")

    def _rx_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                msg = self._bus.recv(timeout=0.25)
            except Exception as exc:
                logger.warning(f"CAN recv failed: {exc}")
                continue
            if msg is None:
                continue

            try:
                self._handle_message(msg.arbitration_id, bytes(msg.data))
            except Exception as exc:
                logger.warning(f"Failed to parse CAN message: {exc}")

    def _handle_message(self, arbitration_id: int, data: bytes) -> None:
        src_node_id = (int(arbitration_id) >> 5) & 0x3F
        if src_node_id != self.node_id:
            return
        command_id = int(arbitration_id) & 0x1F

        now_s = _now_monotonic_s()
        if command_id == self.CMD_HEARTBEAT:
            if len(data) >= 5:
                error = struct.unpack_from("<I", data, 0)[0]
                state = int(data[4])
                with self._lock:
                    self._last_heartbeat_error = int(error)
                    self._last_heartbeat_state = int(state)
                    self._last_heartbeat_time_s = now_s
            return

        if command_id == self.CMD_GET_ENCODER_ESTIMATES and len(data) >= 8:
            pos_turns, vel_turns_s = struct.unpack_from("<ff", data, 0)
            if math.isfinite(pos_turns) and math.isfinite(vel_turns_s):
                with self._lock:
                    self.encoder.pos_estimate = float(pos_turns)
                    self.encoder.vel_estimate = float(vel_turns_s)
            return

        if command_id == self.CMD_GET_IQ and len(data) >= 8:
            _, iq_measured = struct.unpack_from("<ff", data, 0)
            if math.isfinite(iq_measured):
                with self._lock:
                    self.motor.foc.Iq_measured = float(iq_measured)
            return

    def last_heartbeat_age_s(self) -> Optional[float]:
        with self._lock:
            if self._last_heartbeat_time_s is None:
                return None
            return max(0.0, _now_monotonic_s() - self._last_heartbeat_time_s)

    def set_axis_state(self, requested_state: int) -> None:
        self._send(self.CMD_SET_AXIS_STATE, struct.pack("<I", int(requested_state)))

    def set_controller_mode(self, control_mode: int, input_mode: int = INPUT_MODE_PASSTHROUGH) -> None:
        self._send(self.CMD_SET_CONTROLLER_MODE, struct.pack("<II", int(control_mode), int(input_mode)))

    def request_encoder_estimates(self) -> None:
        self._send(self.CMD_GET_ENCODER_ESTIMATES, b"")

    def request_iq(self) -> None:
        self._send(self.CMD_GET_IQ, b"")

    def set_input_pos(self, pos_turns: float, vel_ff_turns_s: float = 0.0, torque_ff_nm: float = 0.0) -> None:
        vel_ff_i16 = int(max(-32768, min(32767, round(float(vel_ff_turns_s) / 0.001))))
        torque_ff_i16 = int(max(-32768, min(32767, round(float(torque_ff_nm) / 0.001))))
        payload = struct.pack("<fhh", float(pos_turns), vel_ff_i16, torque_ff_i16)
        self._send(self.CMD_SET_INPUT_POS, payload)
        with self._lock:
            self._last_command_pos_turns = float(pos_turns)

    def set_input_vel(self, vel_turns_s: float, torque_ff_nm: float = 0.0) -> None:
        payload = struct.pack("<ff", float(vel_turns_s), float(torque_ff_nm))
        self._send(self.CMD_SET_INPUT_VEL, payload)
        with self._lock:
            self._last_command_vel_turns_s = float(vel_turns_s)
            self._last_command_torque_nm = float(torque_ff_nm)

    def set_input_torque(self, torque_nm: float) -> None:
        payload = struct.pack("<f", float(torque_nm))
        self._send(self.CMD_SET_INPUT_TORQUE, payload)
        with self._lock:
            self._last_command_torque_nm = float(torque_nm)
