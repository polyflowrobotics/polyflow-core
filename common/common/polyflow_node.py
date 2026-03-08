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

from common.polyflow_kernel import PolyflowKernel


def _ros_msg_to_dict(msg: Any) -> dict:
    """Convert a ROS message instance to a plain dict, recursively."""
    if hasattr(msg, 'get_fields_and_field_types'):
        result = {}
        for field_name in msg.get_fields_and_field_types():
            value = getattr(msg, field_name)
            result[field_name] = _ros_msg_to_dict(value)
        return result
    elif isinstance(msg, (list, tuple)):
        return [_ros_msg_to_dict(item) for item in msg]
    elif isinstance(msg, bytes):
        return list(msg)
    else:
        return msg


def _dict_to_ros_msg(data: dict, msg_type: type) -> Any:
    """Populate a ROS message instance from a plain dict, recursively."""
    msg = msg_type()
    if not isinstance(data, dict):
        return data

    fields = msg.get_fields_and_field_types() if hasattr(msg, 'get_fields_and_field_types') else {}
    for field_name, field_type_str in fields.items():
        if field_name not in data:
            continue
        value = data[field_name]
        current_field = getattr(msg, field_name)

        if hasattr(current_field, 'get_fields_and_field_types'):
            # Nested ROS message — recurse
            nested = _dict_to_ros_msg(value, type(current_field))
            setattr(msg, field_name, nested)
        elif isinstance(value, list) and 'uint8' in field_type_str and '[]' in field_type_str:
            # uint8[] fields expect bytes
            setattr(msg, field_name, bytes(value))
        else:
            setattr(msg, field_name, value)
    return msg


class PolyflowNode(Node):
    """
    Standard base class for Polyflow-managed ROS 2 nodes.

    This class automatically handles:
    - Node identification and naming from environment variables.
    - Typed ROS topic publishers/subscribers for each registered pin.
    - Graph execution state (RUN/PAUSE/STEP/BREAKPOINT_HIT) via the /ros/graph/control topic.
    - Breakpoint management: nodes can be paused at specific points.
    - Status reporting to the /ros/graph/node_status topic.

    All portable decision-making logic lives in a PolyflowKernel instance
    (self.kernel). Subclasses should set kernel_class to provide their own
    kernel, or override process_input directly for ROS-specific behaviour.

    Topic convention: /graph/{node_id}/{pin_id}
    Each output pin publishes on its own typed topic. Each input pin subscribes
    to the source node's output topic as defined in POLYFLOW_INBOUND_CONNECTIONS.
    """

    kernel_class: type = PolyflowKernel

    def __init__(self):
        # 1. Get Node ID and initialize the ROS Node
        self.node_id = os.environ.get("POLYFLOW_NODE_ID")
        if not self.node_id:
            raise ValueError("POLYFLOW_NODE_ID environment variable not set.")

        # Sanitize node_id for use in ROS node names and topic paths
        # (no hyphens, must not start with a number)
        self.ros_safe_id = self.node_id.replace('-', '_')
        if self.ros_safe_id[0].isdigit():
            self.ros_safe_id = f"n{self.ros_safe_id}"
        super().__init__(self.ros_safe_id)

        # --- Load Parameters & Configuration ---
        self.parameters: Dict[str, Any] = self._load_json_env("POLYFLOW_PARAMETERS", {})
        self.configuration: Dict[str, Any] = self._load_json_env("POLYFLOW_CONFIGURATION", {})

        # --- Instantiate the portable kernel ---
        self.kernel = self.kernel_class(
            node_id=self.node_id,
            parameters=self.parameters,
            configuration=self.configuration,
            on_emit=self._kernel_emit,
            on_log=self._kernel_log,
        )

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

    # --- Kernel callbacks ---

    def _kernel_emit(self, pin_id: str, data: dict):
        """Called by the kernel's emit(). Converts dict -> ROS msg and publishes."""
        msg_type = self._output_pin_types.get(pin_id)
        if not msg_type:
            self.get_logger().debug(f"Kernel emitted on unregistered pin '{pin_id}'")
            return
        ros_msg = _dict_to_ros_msg(data, msg_type)
        self.publish_to_pin(pin_id, ros_msg)

    def _kernel_log(self, message: str):
        """Called by the kernel's log(). Routes to ROS log topic."""
        self.log(message)

    # --- Pin Registration ---

    def register_output_pin(self, pin_id: str, msg_type: type, queue_size: int = 10):
        """
        Register a typed output pin. Creates a ROS publisher on /graph/{node_id}/{pin_id}.

        Args:
            pin_id: The output pin identifier.
            msg_type: The ROS message type to publish.
            queue_size: Publisher queue depth (default: 10).
        """
        topic = f"/graph/{self.ros_safe_id}/{pin_id}"
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
                safe_source_id = source_node_id.replace('-', '_')
                if safe_source_id[0].isdigit():
                    safe_source_id = f"n{safe_source_id}"
                topic = f"/graph/{safe_source_id}/{source_pin_id}"

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

    # --- Helpers ---

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
        """Delegates to the kernel's get_param."""
        return self.kernel.get_param(keys, default)

    def _control_message_callback(self, msg: String):
        """Handles control commands — delegates to kernel after JSON parsing."""
        try:
            data = json.loads(msg.data)
            self.kernel.handle_control(data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Received invalid JSON on /ros/graph/control: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error handling control message: {e}")

    def _publish_status(self):
        """Periodically reports this node's status and connectivity to the ROSDaemon."""
        status = self.kernel.get_status()
        status["inputs"] = list(self._input_pin_types.keys())
        status["outputs"] = list(self._output_pin_types.keys())

        envelope = {
            "prp": "0.1",
            "type": "graph.node_state",
            "payload": status
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
        """Delegates to the kernel's execution state machine."""
        return self.kernel.should_run(trigger_info)

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
        Called when a typed ROS message arrives on an input pin.

        By default, converts the ROS message to a dict and forwards to
        the kernel's process_input. Subclasses that need direct ROS message
        access (e.g. for hardware I/O in the same callback) can override
        this and call self.kernel.process_input() when appropriate.

        Args:
            pin_id: The ID of the input pin that received the data.
            data: The typed ROS message.
        """
        data_dict = _ros_msg_to_dict(data)
        self.kernel.process_input(pin_id, data_dict)

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
