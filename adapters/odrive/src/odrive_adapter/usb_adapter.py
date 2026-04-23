"""
ODriveUSBAdapter — consumes JointCommand / HardwareLifecycle for a single
ODrive via USB. Uses the `odrive` Python package for device discovery
and control.
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
    LEVEL_WARN,
    LEVEL_ERROR,
)

from .units import compute_scales


class ODriveUSBAdapter(HardwareAdapter):
    driver_name = "odrive_usb"
    command_types = ["JointCommand", "HardwareLifecycle"]
    state_types = ["JointState"]

    def configure(self) -> None:
        self.serial_number = self.params.get("serial_number")  # optional
        self.axis_index = int(self.params.get("axis_index", 0))
        self.poll_hz = float(self.params.get("poll_hz", 50.0))
        self.enable_closed_loop_on_start = bool(self.params.get("enable_closed_loop_on_start", True))

        gear_ratio = float(self.params.get("gear_ratio", 1.0))
        units = str(self.params.get("units", "radians"))
        self._scales = compute_scales(gear_ratio, units)

        self._odrive = None      # the odrive module
        self._enums = None       # namespace with AxisState / ControlMode / InputMode
        self._odrv = None        # the connected ODrive device
        self._axis = None        # the selected axis on the device
        self._current_mode: int = JOINT_MODE_POSITION
        self._closed_loop: bool = False
        self._poll_handle: Any = None

    def start(self) -> None:
        try:
            import odrive  # type: ignore
            from odrive import enums as odrive_enums  # type: ignore
        except ImportError:
            self.report_status(LEVEL_ERROR, "odrive_pkg_missing", "odrive python package not installed")
            return

        self._odrive = odrive
        self._enums = odrive_enums

        try:
            if self.serial_number:
                self._odrv = odrive.find_any(serial_number=str(self.serial_number), timeout=10)
            else:
                self._odrv = odrive.find_any(timeout=10)
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "usb_connect_failed", str(exc))
            return

        if self._odrv is None:
            self.report_status(LEVEL_ERROR, "usb_not_found", "No ODrive found via find_any()")
            return

        axis_attr = f"axis{self.axis_index}"
        if not hasattr(self._odrv, axis_attr):
            self.report_status(LEVEL_ERROR, "axis_missing", f"ODrive has no {axis_attr}")
            return
        self._axis = getattr(self._odrv, axis_attr)

        try:
            self._axis.controller.config.control_mode = self._enums.ControlMode.POSITION_CONTROL
            self._axis.controller.config.input_mode = self._enums.InputMode.PASSTHROUGH
        except Exception as exc:
            self.log(f"set control mode failed on start: {exc}")

        if self.enable_closed_loop_on_start:
            try:
                self._axis.requested_state = self._enums.AxisState.CLOSED_LOOP_CONTROL
                self._closed_loop = True
            except Exception as exc:
                self.report_status(LEVEL_WARN, "closed_loop_start_failed", str(exc))

        self._poll_handle = self.schedule_poll(self.poll_hz, self._poll_state)

    def on_command(self, command_type: str, data: dict) -> None:
        if command_type == "JointCommand":
            self._handle_joint_command(data)
        elif command_type == "HardwareLifecycle":
            self._handle_lifecycle(data)

    def _handle_joint_command(self, data: dict) -> None:
        axis = self._axis
        enums = self._enums
        if axis is None or enums is None:
            return

        mode = int(data.get("mode", self._current_mode))
        if mode != self._current_mode:
            try:
                ctrl = {
                    JOINT_MODE_POSITION: enums.ControlMode.POSITION_CONTROL,
                    JOINT_MODE_VELOCITY: enums.ControlMode.VELOCITY_CONTROL,
                    JOINT_MODE_TORQUE: enums.ControlMode.TORQUE_CONTROL,
                }[mode]
                axis.controller.config.control_mode = ctrl
                axis.controller.config.input_mode = enums.InputMode.PASSTHROUGH
                self._current_mode = mode
            except Exception as exc:
                self.report_status(LEVEL_WARN, "mode_switch_failed", str(exc), {"mode": str(mode)})

        try:
            if mode == JOINT_MODE_POSITION and data.get("has_position"):
                pos_turns = float(data["position"]) * self._scales.cmd_position_scale
                axis.controller.input_pos = pos_turns
            elif mode == JOINT_MODE_VELOCITY and data.get("has_velocity"):
                vel_turns_s = float(data["velocity"]) * self._scales.cmd_velocity_scale
                axis.controller.input_vel = vel_turns_s
            elif mode == JOINT_MODE_TORQUE and data.get("has_torque"):
                axis.controller.input_torque = float(data["torque"])
        except Exception as exc:
            self.report_status(LEVEL_WARN, "command_write_failed", str(exc))

    def _handle_lifecycle(self, data: dict) -> None:
        axis = self._axis
        enums = self._enums
        if axis is None or enums is None:
            return

        request = int(data.get("request", -1))
        try:
            if request == LIFECYCLE_ENABLE:
                axis.requested_state = enums.AxisState.CLOSED_LOOP_CONTROL
                self._closed_loop = True
            elif request in (LIFECYCLE_DISABLE, LIFECYCLE_ESTOP):
                axis.requested_state = enums.AxisState.IDLE
                self._closed_loop = False
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "lifecycle_failed", str(exc), {"request": str(request)})

    def _poll_state(self) -> None:
        axis = self._axis
        if axis is None:
            return

        try:
            pos_turns = float(axis.encoder.pos_estimate)
            vel_turns_s = float(axis.encoder.vel_estimate)
            iq = float(getattr(axis.motor, "current_control", None).Iq_measured) \
                if hasattr(axis.motor, "current_control") else 0.0
            torque_constant = getattr(axis.motor.config, "torque_constant", None)
        except Exception as exc:
            self.log(f"state read failed: {exc}")
            return

        pos_rad = pos_turns * self._scales.state_position_scale
        vel_rad_s = vel_turns_s * self._scales.state_velocity_scale
        effort = float(torque_constant) * iq if torque_constant else 0.0

        connected = True  # if we could read, the link is alive

        self.publish_state("JointState", {
            "joint_id": self.target_id,
            "position": pos_rad,
            "velocity": vel_rad_s,
            "effort": effort,
            "mode": self._current_mode,
            "closed_loop": self._closed_loop,
            "connected": connected,
        })

    def estop(self) -> None:
        axis = self._axis
        enums = self._enums
        if axis is None or enums is None:
            return
        try:
            axis.requested_state = enums.AxisState.IDLE
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
        enums = self._enums
        self._axis = None
        if axis is not None and enums is not None:
            try:
                axis.requested_state = enums.AxisState.IDLE
            except Exception:
                pass

        self._odrv = None
