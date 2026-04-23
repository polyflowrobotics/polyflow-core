"""
ODriveCANAdapter — consumes JointCommand / HardwareLifecycle for a single
ODrive axis over CANSimple. Publishes JointState by polling the axis at
configurable rate.
"""

from typing import Any, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    JOINT_MODE_POSITION,
    JOINT_MODE_VELOCITY,
    JOINT_MODE_TORQUE,
    LIFECYCLE_ENABLE,
    LIFECYCLE_DISABLE,
    LIFECYCLE_ESTOP,
    LEVEL_OK,
    LEVEL_WARN,
    LEVEL_ERROR,
)

from .can_axis import CANSimpleAxis
from .units import compute_scales


class ODriveCANAdapter(HardwareAdapter):
    driver_name = "odrive_can"
    command_types = ["JointCommand", "HardwareLifecycle"]
    state_types = ["JointState"]

    _MODE_TO_CTRL = {
        JOINT_MODE_POSITION: CANSimpleAxis.CONTROL_MODE_POSITION_CONTROL,
        JOINT_MODE_VELOCITY: CANSimpleAxis.CONTROL_MODE_VELOCITY_CONTROL,
        JOINT_MODE_TORQUE: CANSimpleAxis.CONTROL_MODE_TORQUE_CONTROL,
    }

    def configure(self) -> None:
        self.node_id = int(self.params["node_id"])
        self.interface = str(self.params.get("interface", "socketcan"))
        self.channel = str(self.params.get("channel", "can0"))
        bitrate = self.params.get("bitrate")
        self.bitrate = int(bitrate) if bitrate else None
        self.torque_constant: Optional[float] = (
            float(self.params["torque_constant"]) if self.params.get("torque_constant") is not None else None
        )

        self.poll_hz = float(self.params.get("poll_hz", 50.0))
        self.heartbeat_timeout_s = float(self.params.get("heartbeat_timeout_s", 2.0))
        self.request_iq = bool(self.params.get("request_iq", False))
        self.enable_closed_loop_on_start = bool(self.params.get("enable_closed_loop_on_start", True))

        gear_ratio = float(self.params.get("gear_ratio", 1.0))
        units = str(self.params.get("units", "radians"))
        self._scales = compute_scales(gear_ratio, units)

        self._axis: Optional[CANSimpleAxis] = None
        self._current_mode: int = JOINT_MODE_POSITION
        self._closed_loop: bool = False
        self._poll_handle: Any = None

    def start(self) -> None:
        try:
            self._axis = CANSimpleAxis(
                node_id=self.node_id,
                can_interface=self.interface,
                can_channel=self.channel,
                can_bitrate=self.bitrate,
                torque_constant=self.torque_constant,
            )
        except ImportError:
            self.report_status(LEVEL_ERROR, "python_can_missing", "python-can is not installed")
            return
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "bus_open_failed", str(exc))
            return

        # Set initial controller mode and optionally engage closed loop.
        try:
            self._axis.set_controller_mode(CANSimpleAxis.CONTROL_MODE_POSITION_CONTROL)
        except Exception as exc:
            self.log(f"set_controller_mode failed on start: {exc}")

        if self.enable_closed_loop_on_start:
            try:
                self._axis.set_axis_state(CANSimpleAxis.AXIS_STATE_CLOSED_LOOP_CONTROL)
                self._closed_loop = True
            except Exception as exc:
                self.report_status(LEVEL_WARN, "closed_loop_start_failed", str(exc))

        self._poll_handle = self.schedule_poll(self.poll_hz, self._poll_state)

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type == "JointCommand":
            self._handle_joint_command(data)
        elif command_type == "HardwareLifecycle":
            self._handle_lifecycle(data)

    # --- Command handlers ---

    def _handle_joint_command(self, data: dict) -> None:
        axis = self._axis
        if axis is None:
            return

        mode = int(data.get("mode", self._current_mode))
        if mode != self._current_mode:
            ctrl = self._MODE_TO_CTRL.get(mode)
            if ctrl is not None:
                try:
                    axis.set_controller_mode(ctrl)
                    self._current_mode = mode
                except Exception as exc:
                    self.report_status(LEVEL_WARN, "mode_switch_failed", str(exc), {"mode": str(mode)})

        try:
            if mode == JOINT_MODE_POSITION and data.get("has_position"):
                pos_turns = float(data["position"]) * self._scales.cmd_position_scale
                axis.set_input_pos(pos_turns)
            elif mode == JOINT_MODE_VELOCITY and data.get("has_velocity"):
                vel_turns_s = float(data["velocity"]) * self._scales.cmd_velocity_scale
                axis.set_input_vel(vel_turns_s)
            elif mode == JOINT_MODE_TORQUE and data.get("has_torque"):
                axis.set_input_torque(float(data["torque"]))
        except Exception as exc:
            self.report_status(LEVEL_WARN, "command_write_failed", str(exc))

    def _handle_lifecycle(self, data: dict) -> None:
        axis = self._axis
        if axis is None:
            return

        request = int(data.get("request", -1))
        try:
            if request == LIFECYCLE_ENABLE:
                axis.set_axis_state(CANSimpleAxis.AXIS_STATE_CLOSED_LOOP_CONTROL)
                self._closed_loop = True
            elif request in (LIFECYCLE_DISABLE, LIFECYCLE_ESTOP):
                axis.set_axis_state(CANSimpleAxis.AXIS_STATE_IDLE)
                self._closed_loop = False
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "lifecycle_failed", str(exc), {"request": str(request)})

    # --- State polling ---

    def _poll_state(self) -> None:
        axis = self._axis
        if axis is None:
            return

        try:
            axis.request_encoder_estimates()
            if self.request_iq:
                axis.request_iq()
        except Exception as exc:
            self.log(f"encoder request failed: {exc}")
            return

        pos_rad = float(axis.encoder.pos_estimate) * self._scales.state_position_scale
        vel_rad_s = float(axis.encoder.vel_estimate) * self._scales.state_velocity_scale

        effort = 0.0
        if self.torque_constant is not None:
            iq = axis.motor.foc.Iq_measured
            if iq is not None:
                effort = float(self.torque_constant) * float(iq)

        heartbeat_age = axis.last_heartbeat_age_s()
        connected = heartbeat_age is not None and heartbeat_age < self.heartbeat_timeout_s

        self.publish_state("JointState", {
            "joint_id": self.target_id,
            "position": pos_rad,
            "velocity": vel_rad_s,
            "effort": effort,
            "mode": self._current_mode,
            "closed_loop": self._closed_loop,
            "connected": connected,
        })

    # --- Safety hooks ---

    def estop(self) -> None:
        axis = self._axis
        if axis is None:
            return
        try:
            axis.set_axis_state(CANSimpleAxis.AXIS_STATE_IDLE)
        except Exception as exc:
            self.log(f"estop failed: {exc}")
        self._closed_loop = False

    def shutdown(self) -> None:
        if self._poll_handle is not None:
            try:
                self._poll_handle.cancel()
            except Exception:
                pass
            self._poll_handle = None

        axis = self._axis
        self._axis = None
        if axis is not None:
            try:
                axis.set_axis_state(CANSimpleAxis.AXIS_STATE_IDLE)
            except Exception:
                pass
            try:
                axis.shutdown()
            except Exception as exc:
                self.log(f"axis shutdown error: {exc}")
