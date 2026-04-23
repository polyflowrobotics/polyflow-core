"""
Host-side protocol — the contract adapters use to talk back to the daemon.

Adapters never create ROS publishers, subscribers, or timers directly.
They call through the AdapterHost interface, which routes to the
system-manager's managed resources (publishers, diagnostics, timers).

The AdapterHost is a `typing.Protocol`; the concrete implementation
lives in polyflow-os's system-manager. This keeps the SDK free of ROS /
rclpy dependencies and pip-installable on any machine.
"""

from typing import Callable, Protocol, runtime_checkable


@runtime_checkable
class ScheduleHandle(Protocol):
    """Returned by AdapterHost.schedule; call cancel() to stop the timer."""

    def cancel(self) -> None: ...


@runtime_checkable
class AdapterHost(Protocol):
    """
    Interface the system-manager exposes to hardware adapters.

    Every helper method on `HardwareAdapter` (publish_state, report_status,
    schedule_poll, log) is a thin wrapper that delegates to the host. The
    host owns all side-effectful resources (ROS publishers, the logger,
    the event loop / timer scheduler).
    """

    def publish_state(self, target_id: str, state_type: str, data: dict) -> None:
        """
        Publish a state message on /prp/hardware/state/<target_id>/<state_type>.

        `state_type` is the message type name (e.g. "JointState",
        "BatteryState"). `data` is a dict whose keys match the message
        type's field names; the host converts to the typed ROS message.
        """
        ...

    def report_status(
        self,
        target_id: str,
        level: int,
        code: str,
        message: str,
        values: dict,
    ) -> None:
        """
        Publish a diagnostic_msgs/DiagnosticStatus for this target on
        /diagnostics. `level` is one of LEVEL_OK/WARN/ERROR/STALE.
        """
        ...

    def schedule(self, rate_hz: float, fn: Callable[[], None]) -> ScheduleHandle:
        """Call `fn` at `rate_hz` until the returned handle is cancelled."""
        ...

    def log(self, message: str) -> None:
        """Write a line to the daemon's log."""
        ...
