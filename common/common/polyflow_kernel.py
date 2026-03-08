import json
import time
from typing import Any, Callable, Dict, List, Optional, Union


class PolyflowKernel:
    """
    Pure-Python kernel for Polyflow node logic. No ROS or hardware dependencies.

    This class contains the portable decision-making logic of a Polyflow node:
    the execution state machine (run/pause/step/breakpoints), parameter access,
    and the process_input/emit pattern. It is designed to run identically in:

      - CPython  (wrapped by PolyflowNode for ROS transport + real hardware)
      - Pyodide  (wrapped by a JS PolyflowNode for browser transport + 3D scene mocks)
      - Cloud    (wrapped by a cloud PolyflowNode for remote transport + sim mocks)

    All I/O is mediated through plain Python dicts:
      - Incoming messages arrive as dicts via process_input(pin_id, data).
      - Outgoing messages are emitted via emit(pin_id, data), routed by the
        host wrapper to ROS publishers, JS postMessage, WebSocket, etc.
      - Logging goes through the on_log callback.

    Hardware interaction is NOT part of the kernel. The kernel computes what
    should happen (e.g. "set motor speed to 0.5") and emits it. The host
    wrapper is responsible for actually talking to hardware, or forwarding
    to a 3D scene mock.

    Subclass contract:
      - Override setup() for initialisation that reads self.parameters.
      - Override process_input(pin_id, data) for message handling.
      - Call self.emit(pin_id, data) to produce output.
      - Call self.log(message) to produce log output.
    """

    def __init__(
        self,
        node_id: str,
        parameters: Optional[Dict[str, Any]] = None,
        configuration: Optional[Dict[str, Any]] = None,
        on_emit: Optional[Callable[[str, dict], None]] = None,
        on_log: Optional[Callable[[str], None]] = None,
    ):
        self.node_id = node_id
        self.parameters: Dict[str, Any] = parameters or {}
        self.configuration: Dict[str, Any] = configuration or {}

        # Callbacks set by the host wrapper (PolyflowNode, JS bridge, cloud bridge)
        self.on_emit: Optional[Callable[[str, dict], None]] = on_emit
        self.on_log: Optional[Callable[[str], None]] = on_log

        # --- Execution State ---
        self._run_state: str = "RUN"
        self._step_budget: int = 0
        self._active_breakpoints: Dict[str, Dict[str, Any]] = {}
        self._breakpoint_hit_id: Optional[str] = None

        self.setup()

    def setup(self):
        """Override to perform initialisation after parameters are available."""
        pass

    def get_param(self, keys: Union[str, List[str]], default: Any = None) -> Any:
        if isinstance(keys, str):
            keys = [keys]
        for key in keys:
            if key in self.parameters:
                return self.parameters[key]
        return default

    def should_run(self, trigger_info: Optional[Dict[str, Any]] = None) -> bool:
        if self._run_state == "RUN":
            for bp_id, bp_details in self._active_breakpoints.items():
                if bp_details.get("hit"):
                    continue
                condition = bp_details.get("condition", {})
                condition_met = True
                if 'input_pin_id' in condition:
                    if not trigger_info or trigger_info.get('pin_id') != condition['input_pin_id']:
                        condition_met = False
                if condition_met:
                    self._breakpoint_hit_id = bp_id
                    self._active_breakpoints[bp_id]["hit"] = True
                    self._run_state = "BREAKPOINT_HIT"
                    self.log(f"Node '{self.node_id}' hit breakpoint '{bp_id}'. Pausing execution.")
                    return False
            return True
        elif self._run_state == "PAUSE":
            if self._step_budget > 0:
                self._step_budget -= 1
                return True
            return False
        elif self._run_state == "BREAKPOINT_HIT":
            return False
        return False

    def handle_control(self, data: Dict[str, Any]):
        """Process a control command (run/pause/step/breakpoint).

        This is the dict-based equivalent of PolyflowNode._control_message_callback.
        The host wrapper deserialises the control message and forwards it here.
        """
        target_nodes = data.get("target_nodes", [])
        if target_nodes and self.node_id not in target_nodes:
            return

        command = data.get("command")

        if command == "set_state":
            new_state = data.get("state", "RUN")
            if self._run_state == "BREAKPOINT_HIT" and new_state == "PAUSE":
                return
            self._run_state = new_state

        elif command == "step":
            if self._run_state == "PAUSE":
                self._step_budget += data.get("steps", 1)

        elif command == "set_breakpoint":
            breakpoint_id = data.get("breakpoint_id")
            condition = data.get("condition", {})
            if breakpoint_id:
                self._active_breakpoints[breakpoint_id] = {"condition": condition, "hit": False}

        elif command == "continue_breakpoint":
            breakpoint_id = data.get("breakpoint_id")
            if breakpoint_id and breakpoint_id in self._active_breakpoints:
                del self._active_breakpoints[breakpoint_id]
                if self._breakpoint_hit_id == breakpoint_id:
                    self._breakpoint_hit_id = None
                    if not self._active_breakpoints and self._run_state == "BREAKPOINT_HIT":
                        self._run_state = "RUN"
            elif not breakpoint_id:
                self._active_breakpoints.clear()
                self._breakpoint_hit_id = None
                self._run_state = "RUN"

        elif command == "clear_breakpoints":
            breakpoint_id = data.get("breakpoint_id")
            if breakpoint_id:
                if breakpoint_id in self._active_breakpoints:
                    del self._active_breakpoints[breakpoint_id]
                    if self._breakpoint_hit_id == breakpoint_id:
                        self._breakpoint_hit_id = None
                        if not self._active_breakpoints and self._run_state == "BREAKPOINT_HIT":
                            self._run_state = "RUN"
            else:
                self._active_breakpoints.clear()
                self._breakpoint_hit_id = None
                if self._run_state == "BREAKPOINT_HIT":
                    self._run_state = "RUN"

    def emit(self, pin_id: str, data: dict):
        """Emit a message on an output pin. The host wrapper handles transport."""
        if self.on_emit:
            self.on_emit(pin_id, data)

    def log(self, message: str):
        """Emit a log message. The host wrapper handles transport."""
        if self.on_log:
            self.on_log(message)

    def get_status(self) -> Dict[str, Any]:
        """Return the current kernel status as a plain dict."""
        return {
            "node": self.node_id,
            "state": self._run_state,
            "active_breakpoints": list(self._active_breakpoints.keys()),
            "breakpoint_hit_id": self._breakpoint_hit_id,
            "timestamp": time.time(),
        }

    def process_input(self, pin_id: str, data: dict):
        """Override to handle incoming messages. Data is a plain dict."""
        pass
