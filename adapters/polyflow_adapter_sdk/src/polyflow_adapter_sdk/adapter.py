"""
HardwareAdapter — base class for Polyflow hardware adapters.
"""

from typing import Any, Callable, Optional

from .host import AdapterHost, ScheduleHandle


class HardwareAdapter:
    """
    Base class for Polyflow hardware adapters.

    An adapter is a plugin hosted by polyflow-os's system-manager that
    consumes typed PRP commands and produces typed PRP state for a
    single logical hardware target (e.g. one joint, one motor, one IMU).

    Subclasses declare what they handle via class attributes, implement
    the four lifecycle hooks (configure, start, on_command, shutdown),
    and use the provided helpers (publish_state, report_status,
    schedule_poll, log) to interact with the daemon.

    Resource composition: an adapter that owns a shared resource (e.g.
    a serial port or CAN bus) overrides `get_resource()` to return its
    handle. Dependent adapters declare the resource's driver_name in
    `requires`, and the daemon injects the resolved instance via the
    `resources` dict at construction time.
    """

    # --- Class attributes (set by adapter authors) ---

    driver_name: str = ""
    """Unique identifier referenced by hardware.yaml to instantiate this adapter."""

    command_types: list = []
    """PRP command message types this adapter consumes, e.g. ["JointCommand"]."""

    state_types: list = []
    """PRP state message types this adapter publishes, e.g. ["JointState"]."""

    requires: list = []
    """Driver names of resource-providing adapters this one depends on."""

    # --- Construction (called by the daemon) ---

    def __init__(
        self,
        host: AdapterHost,
        target_id: str,
        params: dict,
        resources: Optional[dict] = None,
    ) -> None:
        self._host = host
        self.target_id = target_id
        """The logical hardware id this instance owns (from hardware.yaml)."""
        self.params: dict = params
        """Params dict for this entry, as declared in hardware.yaml."""
        self.resources: dict = resources or {}
        """Resolved resource handles, keyed by the required driver_name."""

    # --- Lifecycle hooks (override in subclasses) ---

    def configure(self) -> None:
        """
        Called once after construction. Open hardware handles, resolve
        params, prepare internal state. Must not publish or begin polling
        yet — that is start()'s job.
        """

    def start(self) -> None:
        """
        Called after all adapters are configured. Begin polling, emit
        initial state, enter active operation.
        """

    def on_command(self, command_type: str, data: dict) -> None:
        """
        Dispatched when a PRP command targeting this adapter arrives.

        `command_type` is one of self.command_types (e.g. "JointCommand").
        `data` is the message payload as a dict — keys match the .msg
        field names exactly.
        """

    def estop(self) -> None:
        """
        Apply an emergency stop. Called by the daemon when a
        HardwareLifecycle.ESTOP broadcast is received. Default no-op;
        override to cut motor power, disable outputs, etc.
        """

    def shutdown(self) -> None:
        """
        Release hardware resources. Close serial ports, cancel timers,
        join threads. The daemon calls this in reverse-dependency order.
        """

    # --- Resource-provider hook (for adapters that own a shared resource) ---

    def get_resource(self) -> Any:
        """
        Return this adapter's shared resource (e.g. an opened serial
        Device, a CAN Bus instance) for injection into dependents
        declared via `requires`. Default None; override in resource
        owners such as bus adapters.
        """
        return None

    # --- Helpers (do not override) ---

    def publish_state(self, state_type: str, data: dict) -> None:
        """Publish a state message under this adapter's target_id."""
        self._host.publish_state(self.target_id, state_type, data)

    def report_status(
        self,
        level: int,
        code: str,
        message: str,
        values: Optional[dict] = None,
    ) -> None:
        """Publish a diagnostic entry on /diagnostics for this adapter."""
        self._host.report_status(self.target_id, level, code, message, values or {})

    def schedule_poll(self, rate_hz: float, fn: Callable[[], None]) -> ScheduleHandle:
        """Call `fn()` at `rate_hz`. The returned handle cancels the timer."""
        return self._host.schedule(rate_hz, fn)

    def log(self, message: str) -> None:
        """Write a log entry tagged with this adapter's identity."""
        self._host.log(f"[{self.driver_name}:{self.target_id}] {message}")
