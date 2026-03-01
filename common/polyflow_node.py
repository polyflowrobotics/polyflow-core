import os
import json
import time
import asyncio
import sys
import threading
import traceback
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from typing import Any, Dict, List, Optional, Set, Union


class PolyflowNode(Node):
    """
    Standard base class for Polyflow-managed ROS 2 nodes.

    This class automatically handles:
    - Node identification and naming from environment variables.
    - Typed ROS topic publishers/subscribers for each registered pin.
    - Graph execution state (RUN/PAUSE/STEP/BREAKPOINT_HIT) via the /ros/graph/control topic.
    - Breakpoint management: nodes can be paused at specific points.
    - Status reporting to the /ros/graph/node_status topic.

    Topic convention: /graph/{node_id}/{pin_id}
    Each output pin publishes on its own typed topic. Each input pin subscribes
    to the source node's output topic as defined in POLYFLOW_INBOUND_CONNECTIONS.
    """

    def __init__(self):
        # 1. Get Node ID and initialize the ROS Node
        self.node_id = os.environ.get("POLYFLOW_NODE_ID")
        if not self.node_id:
            raise ValueError("POLYFLOW_NODE_ID environment variable not set.")

        # Sanitize node_id to be a valid ROS node name (ROS node names cannot contain hyphens)
        node_name = self.node_id.replace('-', '_')
        super().__init__(node_name)

        # --- Load Parameters & Configuration ---
        self.parameters: Dict[str, Any] = self._load_json_env("POLYFLOW_PARAMETERS", {})
        self.configuration: Dict[str, Any] = self._load_json_env("POLYFLOW_CONFIGURATION", {})

        # --- Execution State ---
        self._run_state: str = "RUN"  # Can be "RUN", "PAUSE", "BREAKPOINT_HIT"
        self._step_budget: int = 0    # Number of steps allowed when paused

        # Breakpoint management
        self._active_breakpoints: Dict[str, Dict[str, Any]] = {}
        self._breakpoint_hit_id: Optional[str] = None

        # --- Graph Connection Setup ---
        self.inbound_connections: List[Dict[str, Any]] = self._load_json_env("POLYFLOW_INBOUND_CONNECTIONS", [])
        self.outbound_connections: List[Dict[str, Any]] = self._load_json_env("POLYFLOW_OUTBOUND_CONNECTIONS", [])

        # Build outbound map for efficient lookup
        self.outbound_map: Dict[str, List[Dict[str, Any]]] = {}
        for conn in self.outbound_connections:
            pin_id = conn.get("source_pin_id")
            if pin_id not in self.outbound_map:
                self.outbound_map[pin_id] = []
            self.outbound_map[pin_id].append(conn)

        # --- Typed Pin Publishers & Subscribers ---
        self._pin_publishers: Dict[str, Any] = {}   # pin_id -> ROS publisher
        self._pin_subscribers: Dict[str, Any] = {}   # pin_id -> ROS subscription
        self._output_pin_types: Dict[str, type] = {} # pin_id -> message type
        self._input_pin_types: Dict[str, type] = {}  # pin_id -> message type

        self.get_logger().info(f"Initialized PolyflowNode '{self.node_id}'")
        self.get_logger().info(f"Inbound connections: {len(self.inbound_connections)}")
        self.get_logger().info(f"Outbound connections: {len(self.outbound_connections)}")

        # --- Control & Status (remain as JSON String topics) ---
        self.control_subscription = self.create_subscription(
            String,
            '/ros/graph/control',
            self._control_message_callback,
            10
        )
        self.status_publisher = self.create_publisher(String, '/ros/graph/node_status', 10)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # --- Logging (system-level topic, not a pin) ---
        self._log_publisher = None  # Lazy-initialized on first self.log() call

    def register_output_pin(self, pin_id: str, msg_type: type, queue_size: int = 10):
        """
        Register a typed output pin. Creates a ROS publisher on /graph/{node_id}/{pin_id}.

        Args:
            pin_id: The output pin identifier.
            msg_type: The ROS message type to publish.
            queue_size: Publisher queue depth (default: 10).
        """
        topic = f"/graph/{self.node_id}/{pin_id}"
        self._pin_publishers[pin_id] = self.create_publisher(msg_type, topic, queue_size)
        self._output_pin_types[pin_id] = msg_type
        self.get_logger().info(f"Output pin '{pin_id}' -> {topic} [{msg_type.__name__}]")

    def register_input_pin(self, pin_id: str, msg_type: type, queue_size: int = 10):
        """
        Register a typed input pin. Creates ROS subscribers for each inbound
        connection that targets this pin, subscribing to the source node's
        output topic.

        Args:
            pin_id: The input pin identifier.
            msg_type: The ROS message type to subscribe to.
            queue_size: Subscriber queue depth (default: 10).
        """
        self._input_pin_types[pin_id] = msg_type

        # Find all inbound connections targeting this pin and subscribe to each source
        sources_found = False
        for conn in self.inbound_connections:
            if conn.get("target_pin_id") == pin_id:
                source_node_id = conn.get("source_node_id")
                source_pin_id = conn.get("source_pin_id")
                topic = f"/graph/{source_node_id}/{source_pin_id}"

                def make_callback(p_id):
                    return lambda msg: self._typed_input_callback(p_id, msg)

                sub = self.create_subscription(
                    msg_type,
                    topic,
                    make_callback(pin_id),
                    queue_size
                )
                self._pin_subscribers[f"{pin_id}:{source_node_id}:{source_pin_id}"] = sub
                self.get_logger().info(f"Input pin '{pin_id}' <- {topic} [{msg_type.__name__}]")
                sources_found = True

        if not sources_found:
            self.get_logger().debug(f"Input pin '{pin_id}' registered but no inbound connections found")

    def _typed_input_callback(self, pin_id: str, msg: Any):
        """Routes incoming typed messages to process_input."""
        try:
            self.process_input(pin_id, msg)
        except Exception as e:
            self.get_logger().error(f"Error processing input on pin '{pin_id}': {e}")

    def _load_json_env(self, key: str, default: Any) -> Any:
        """Safely loads a JSON string from an environment variable."""
        raw = os.environ.get(key)
        if not raw:
            return default
        try:
            return json.loads(raw)
        except json.JSONDecodeError as e:
            log_msg = f"Failed to parse JSON from env var '{key}': {e}. Using default. Raw: '{raw}'"
            try:
                self.get_logger().error(log_msg)
            except Exception:
                print(f"[ERROR] {log_msg}", file=sys.stderr)
            return default

    def get_param(self, keys: Union[str, List[str]], default: Any = None) -> Any:
        """
        Gets a parameter value, trying multiple keys in order.

        Args:
            keys: A single key or a list of keys to try.
            default: The value to return if no key is found.
        """
        if isinstance(keys, str):
            keys = [keys]
        for key in keys:
            if key in self.parameters:
                return self.parameters[key]
        return default

    def _control_message_callback(self, msg: String):
        """Handles control commands like RUN, PAUSE, STEP, BREAKPOINT from the ROSDaemon."""
        try:
            data = json.loads(msg.data)

            target_nodes = data.get("target_nodes", [])
            if target_nodes and self.node_id not in target_nodes:
                return

            command = data.get("command")

            if command == "set_state":
                new_state = data.get("state", "RUN")
                if self._run_state == "BREAKPOINT_HIT" and new_state == "PAUSE":
                    self.get_logger().info(f"Node '{self.node_id}' is at a breakpoint. Ignoring PAUSE command.")
                    return

                self._run_state = new_state
                self.get_logger().info(f"Graph state set to: {self._run_state}")

            elif command == "step":
                if self._run_state == "PAUSE":
                    steps = data.get("steps", 1)
                    self._step_budget += steps
                    self.get_logger().info(f"Stepping {steps} frames")

            elif command == "set_breakpoint":
                breakpoint_id = data.get("breakpoint_id")
                condition = data.get("condition", {})
                if breakpoint_id:
                    self._active_breakpoints[breakpoint_id] = {"condition": condition, "hit": False}
                    self.get_logger().info(f"Breakpoint '{breakpoint_id}' set on node '{self.node_id}'")

            elif command == "continue_breakpoint":
                breakpoint_id = data.get("breakpoint_id")
                if breakpoint_id and breakpoint_id in self._active_breakpoints:
                    del self._active_breakpoints[breakpoint_id]
                    if self._breakpoint_hit_id == breakpoint_id:
                        self._breakpoint_hit_id = None
                        if not self._active_breakpoints and self._run_state == "BREAKPOINT_HIT":
                            self._run_state = "RUN"
                            self.get_logger().info(f"Node '{self.node_id}' resumed from breakpoint '{breakpoint_id}'.")
                    self.get_logger().info(f"Breakpoint '{breakpoint_id}' cleared from node '{self.node_id}'.")
                elif not breakpoint_id:
                    self.get_logger().info(f"Continuing all breakpoints on node '{self.node_id}'.")
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
                        self.get_logger().info(f"Breakpoint '{breakpoint_id}' cleared from node '{self.node_id}'.")
                else:
                    self._active_breakpoints.clear()
                    self._breakpoint_hit_id = None
                    if self._run_state == "BREAKPOINT_HIT":
                        self._run_state = "RUN"
                    self.get_logger().info(f"All breakpoints cleared from node '{self.node_id}'.")

        except json.JSONDecodeError:
            self.get_logger().error(f"Received invalid JSON on /ros/graph/control: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error handling control message: {e}")

    def _publish_status(self):
        """Periodically reports this node's status and connectivity to the ROSDaemon."""
        status_payload = {
            "node": self.node_id,
            "state": self._run_state,
            "inputs": list(self._input_pin_types.keys()),
            "outputs": list(self._output_pin_types.keys()),
            "active_breakpoints": list(self._active_breakpoints.keys()),
            "breakpoint_hit_id": self._breakpoint_hit_id,
            "timestamp": time.time()
        }

        envelope = {
            "prp": "0.1",
            "type": "graph.node_state",
            "payload": status_payload
        }

        msg = String()
        msg.data = json.dumps(envelope)
        self.status_publisher.publish(msg)

    def publish_to_pin(self, pin_id: str, msg: Any):
        """
        Publishes a typed ROS message on an output pin.

        Args:
            pin_id: The output pin identifier (must have been registered with register_output_pin).
            msg: A ROS message instance matching the pin's registered type.
        """
        publisher = self._pin_publishers.get(pin_id)
        if not publisher:
            self.get_logger().debug(f"No publisher for output pin '{pin_id}' on node '{self.node_id}'")
            return

        publisher.publish(msg)
        self.get_logger().debug(f"Published on pin '{pin_id}'")

    def log(self, message: str):
        """
        Publish a log entry to the shared /graph/log system topic.

        This is picked up by the Logger utility node (if present) without
        requiring explicit pin wiring.

        Args:
            message: The log message string.
        """
        if self._log_publisher is None:
            from polyflow_msgs.msg import LogEntry
            self._log_publisher = self.create_publisher(LogEntry, '/graph/log', 10)
            self._log_entry_cls = LogEntry

        entry = self._log_entry_cls()
        entry.pin_id = self.node_id
        entry.data = message
        entry.timestamp = time.time()
        self._log_publisher.publish(entry)

    def should_run(self, trigger_info: Optional[Dict[str, Any]] = None) -> bool:
        """
        Guard method to check if the node's main processing logic should execute.
        This respects the PAUSE, STEP, and BREAKPOINT commands from the control plane.

        Args:
            trigger_info: Optional dictionary with context about the execution trigger.
                          For input processing, this could be `{'pin_id': 'the_pin'}`.

        Returns:
            True if the node should execute its logic, False otherwise.
        """
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
                    self.get_logger().info(f"Node '{self.node_id}' hit breakpoint '{bp_id}'. Pausing execution.")
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

    @property
    def output_pins(self) -> List[str]:
        """Returns a list of registered output pin IDs."""
        return list(self._output_pin_types.keys())

    @property
    def input_pins(self) -> List[str]:
        """Returns a list of registered input pin IDs."""
        return list(self._input_pin_types.keys())

    def process_input(self, pin_id: str, data: Any):
        """
        **User-overridable method.** Called when a typed message arrives on an input pin.

        Args:
            pin_id: The ID of the input pin that received the data.
            data: The typed ROS message.
        """
        self.get_logger().warn(f"Node '{self.node_id}' received data on pin '{pin_id}' but 'process_input' is not implemented.")

    async def run_async(self):
        """
        **User-overridable async method.** Implement long-running async tasks here
        (e.g., hardware connections, periodic publishing loops).
        """
        self.get_logger().info(f"Node '{self.node_id}' has no async tasks. 'run_async' is not implemented.")
        while rclpy.ok():
            await asyncio.sleep(1.0)

    @classmethod
    def main(cls, args=None):
        """
        Standard main entry point for a Polyflow node.
        Handles ROS initialization, executor spinning, and async loop management.
        """
        try:
            rclpy.init(args=args)
            node = cls()

            executor = SingleThreadedExecutor()
            executor.add_node(node)
            ros_thread = threading.Thread(target=executor.spin, daemon=True)
            ros_thread.start()
            node.get_logger().info("ROS executor started (background thread)")

            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                node.get_logger().info(f"Attempting to run async tasks for {node.get_name()}...")
                loop.run_until_complete(node.run_async())
            except KeyboardInterrupt:
                node.get_logger().info("Keyboard interrupt received")
            except Exception as e:
                node.get_logger().error(f"Async loop for {node.get_name()} crashed: {e}")
                traceback.print_exc()
            finally:
                node.get_logger().info("Shutting down")
                executor.shutdown()
                if ros_thread.is_alive():
                    ros_thread.join(timeout=1.0)
                loop.stop()
                loop.close()
                node.destroy_node()
                rclpy.shutdown()
        except Exception as e:
            print(f"=== {cls.__name__.upper()} NODE CRASH ON STARTUP ===", file=sys.stderr, flush=True)
            traceback.print_exc()
            raise
