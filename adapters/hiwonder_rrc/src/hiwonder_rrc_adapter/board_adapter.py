"""
HiwonderRRCBoardAdapter — owns the serial link to an RRC Lite board.

Acts as a resource provider: child adapters (motor, pwm_servo, bus_servo)
declare `requires = ["hiwonder_rrc_board"]` and receive this adapter's
HiwonderRRC driver instance via `resources`.

Publishes sensor_msgs/Imu and sensor_msgs/BatteryState by draining the
driver's receive queues on a timer.
"""

from typing import Any, Optional

from polyflow_adapter_sdk import (
    HardwareAdapter,
    LEVEL_ERROR,
    LEVEL_OK,
    LEVEL_STALE,
)

from .rrc import HiwonderRRC


class HiwonderRRCBoardAdapter(HardwareAdapter):
    driver_name = "hiwonder_rrc_board"
    command_types: list = []
    state_types = ["Imu", "BatteryState"]

    def configure(self) -> None:
        self.device = str(self.params.get("device", "/dev/ttyAMA0"))
        self.baud = int(self.params.get("baud", 1_000_000))
        self.poll_hz = float(self.params.get("poll_hz", 100.0))
        self.battery_cells = int(self.params.get("battery_cells", 0))

        self._rrc: Optional[HiwonderRRC] = None
        self._poll_handle: Any = None
        self._connected: bool = False
        self._last_battery_v: Optional[float] = None

    def start(self) -> None:
        try:
            self._rrc = HiwonderRRC(port=self.device, baudrate=self.baud)
            self._rrc.open()
        except ImportError:
            self.report_status(LEVEL_ERROR, "pyserial_missing", "pyserial is not installed")
            self._rrc = None
            return
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "serial_open_failed", str(exc), {"device": self.device})
            self._rrc = None
            return

        self._connected = True
        self.report_status(LEVEL_OK, "connected", f"RRC board open on {self.device}")
        self._poll_handle = self.schedule_poll(self.poll_hz, self._drain_queues)

    def get_resource(self) -> Any:
        return self._rrc

    # --- Polling ---

    def _drain_queues(self) -> None:
        rrc = self._rrc
        if rrc is None:
            return

        # Drain IMU queue — one publish per sample received.
        while True:
            sample = rrc.get_imu()
            if sample is None:
                break
            ax, ay, az, gx, gy, gz = sample
            self.publish_state("Imu", {
                "header": {"frame_id": self.target_id},
                "orientation_covariance": [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "angular_velocity": {"x": float(gx), "y": float(gy), "z": float(gz)},
                "linear_acceleration": {"x": float(ax), "y": float(ay), "z": float(az)},
            })

        # Drain battery queue.
        while True:
            mv = rrc.get_battery()
            if mv is None:
                break
            voltage = float(mv) / 1000.0
            self._last_battery_v = voltage
            self.publish_state("BatteryState", {
                "header": {"frame_id": self.target_id},
                "voltage": voltage,
                "present": True,
                "power_supply_status": 0,     # UNKNOWN
                "power_supply_health": 0,     # UNKNOWN
                "power_supply_technology": 0, # UNKNOWN
            })

    # --- Lifecycle ---

    def estop(self) -> None:
        rrc = self._rrc
        if rrc is None:
            return
        try:
            rrc.stop_all_motors()
        except Exception as exc:
            self.log(f"estop stop_all_motors failed: {exc}")

    def shutdown(self) -> None:
        if self._poll_handle is not None:
            try:
                self._poll_handle.cancel()
            except Exception:
                pass
            self._poll_handle = None

        rrc = self._rrc
        self._rrc = None
        self._connected = False
        if rrc is not None:
            try:
                rrc.stop_all_motors()
            except Exception:
                pass
            try:
                rrc.close()
            except Exception as exc:
                self.log(f"serial close error: {exc}")
            self.report_status(LEVEL_STALE, "closed", "RRC board connection closed")
