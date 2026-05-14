from common.polyflow_kernel import PolyflowKernel


class BatteryMonitorKernel(PolyflowKernel):
    """
    Portable logic for a generic battery monitor.

    Forwards sensor_msgs/BatteryState dicts received from one or more
    hardware adapters onto the `battery_state` output pin. Each message
    retains its header.frame_id so downstream consumers can identify
    which battery (adapter target) it came from.

    Parameters:
        target_id:           Optional filter — listen only to this adapter
                             target. Leave unset to mirror every
                             BatteryState source on the bus.
        discovery_period_s:  How often the host wrapper should rescan for
                             new BatteryState topics (default: 2.0).
    """

    def setup(self):
        self.target_id = str(self.get_param("target_id", "") or "")
        self._connected = False

    def update_state(self, data: dict):
        if not self.should_run():
            return
        self._connected = True
        self.emit("battery_state", data)
