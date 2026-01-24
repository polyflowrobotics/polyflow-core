# /path/to/your/polyflow_ros_utils/polyflow_node.py
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
    - Subscription and publishing based on graph connections defined in env vars.
    - Graph execution state (RUN/PAUSE/STEP/BREAKPOINT_HIT) via the /ros/graph/control topic.
    - Breakpoint management: nodes can be paused at specific points.
    - Status reporting to the /ros/graph/node_status topic.
    - Automatic routing of messages between nodes via a single /graph/data topic.
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
        # These are loaded first so other initializers can use them.
        self.parameters: Dict[str, Any] = self._load_json_env("POLYFLOW_PARAMETERS", {})
        self.configuration: Dict[str, Any] = self._load_json_env("POLYFLOW_CONFIGURATION", {})

        # --- Execution State ---
        self._run_state: str = "RUN"  # Can be "RUN", "PAUSE", "BREAKPOINT_HIT"
        self._step_budget: int = 0    # Number of steps allowed when paused
        
        # Breakpoint management
        # Stores active breakpoints: {breakpoint_id: {"condition": str, "hit": bool}}
        self._active_breakpoints: Dict[str, Dict[str, Any]] = {}
        self._breakpoint_hit_id: Optional[str] = None # The ID of the breakpoint currently pausing this node

        # --- Graph Connection Setup ---
        # Parse inbound and outbound connections from environment variables
        self.inbound_connections: List[Dict[str, Any]] = self._load_json_env("POLYFLOW_INBOUND_CONNECTIONS", [])
        self.outbound_connections: List[Dict[str, Any]] = self._load_json_env("POLYFLOW_OUTBOUND_CONNECTIONS", [])
        
        # Create a map for efficient lookup of outbound connections by source pin
        # This allows a node to easily find all destinations for a given output pin.
        self.outbound_map: Dict[str, List[Dict[str, Any]]] = {}
        for conn in self.outbound_connections:
            pin_id = conn.get("source_pin_id")
            if pin_id not in self.outbound_map:
                self.outbound_map[pin_id] = []
            self.outbound_map[pin_id].append(conn)

        self.get_logger().info(f"Initialized PolyflowNode '{self.node_id}'")
        self.get_logger().info(f"Inbound connections: {len(self.inbound_connections)}")
        self.get_logger().info(f"Outbound connections: {len(self.outbound_connections)}")

        # --- ROS Communication Setup ---
        # A single data bus for all node-to-node communication within the graph
        self.graph_publisher = self.create_publisher(String, '/graph/data', 10)
        self.graph_subscription = self.create_subscription(
            String,
            '/graph/data',
            self._graph_message_callback,
            10
        )

        # Subscription for control messages from the ROSDaemon (e.g., pause, run, step, breakpoints)
        self.control_subscription = self.create_subscription(
            String,
            '/ros/graph/control',
            self._control_message_callback,
            10
        )

        # Publisher for reporting node status back to the ROSDaemon
        self.status_publisher = self.create_publisher(String, '/ros/graph/node_status', 10)
        
        # Timer to periodically publish node status
        self.status_timer = self.create_timer(1.0, self._publish_status)

    def _load_json_env(self, key: str, default: Any) -> Any:
        """Safely loads a JSON string from an environment variable."""
        raw = os.environ.get(key)
        if not raw:
            return default
        try:
            return json.loads(raw)
        except json.JSONDecodeError as e:
            # Use a logger if available, otherwise print to stderr.
            # The logger might not be fully initialized when this is first called.
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

    def _graph_message_callback(self, msg: String):
        """
        Receives all messages on the /graph/data bus and filters for messages
        targeted specifically at this node's ID and its input pins.
        """
        # Messages are processed even if paused/breakpoint hit, but the user's
        # `process_input` method will be guarded by `should_run()`.
        # This allows messages to queue up or be inspected even when paused.
        try:
            envelope = json.loads(msg.data)
            
            # Filter for messages targeted specifically at this node
            if envelope.get("target_node_id") != self.node_id:
                return

            target_pin_id = envelope.get("target_pin_id")
            data = envelope.get("data")
            
            if target_pin_id is None:
                self.get_logger().warning(f"Received message with no target_pin_id: {envelope}")
                return

            # Dispatch to the user-defined processing method
            self.process_input(target_pin_id, data)

        except json.JSONDecodeError:
            self.get_logger().error(f"Received invalid JSON on /graph/data: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing graph message: {e}")

    def _control_message_callback(self, msg: String):
        """Handles control commands like RUN, PAUSE, STEP, BREAKPOINT from the ROSDaemon."""
        try:
            data = json.loads(msg.data)
            
            # Check if this message is targeted at this node (or all nodes if target_nodes is empty)
            target_nodes = data.get("target_nodes", [])
            if target_nodes and self.node_id not in target_nodes:
                return

            command = data.get("command")
            
            if command == "set_state":
                new_state = data.get("state", "RUN")
                if self._run_state == "BREAKPOINT_HIT" and new_state == "PAUSE":
                    self.get_logger().info(f"Node '{self.node_id}' is at a breakpoint. Ignoring PAUSE command.")
                    return # Don't override breakpoint with a general pause
                
                self._run_state = new_state
                self.get_logger().info(f"Graph state set to: {self._run_state}")
                
            elif command == "step":
                if self._run_state == "PAUSE":
                    steps = data.get("steps", 1)
                    self._step_budget += steps
                    self.get_logger().info(f"Stepping {steps} frames")
            
            elif command == "set_breakpoint":
                breakpoint_id = data.get("breakpoint_id")
                condition = data.get("condition", {}) # For conditional breakpoints, e.g., {"input_pin_id": "my_pin"}
                if breakpoint_id:
                    self._active_breakpoints[breakpoint_id] = {"condition": condition, "hit": False}
                    self.get_logger().info(f"Breakpoint '{breakpoint_id}' set on node '{self.node_id}'")
            
            elif command == "continue_breakpoint":
                breakpoint_id = data.get("breakpoint_id")
                if breakpoint_id and breakpoint_id in self._active_breakpoints:
                    del self._active_breakpoints[breakpoint_id]
                    if self._breakpoint_hit_id == breakpoint_id:
                        self._breakpoint_hit_id = None
                        # If no other breakpoints are hit, and not globally paused, resume RUN
                        if not self._active_breakpoints and self._run_state == "BREAKPOINT_HIT":
                            self._run_state = "RUN"
                            self.get_logger().info(f"Node '{self.node_id}' resumed from breakpoint '{breakpoint_id}'.")
                    self.get_logger().info(f"Breakpoint '{breakpoint_id}' cleared from node '{self.node_id}'.")
                elif not breakpoint_id: # Continue all breakpoints on this node
                    self.get_logger().info(f"Continuing all breakpoints on node '{self.node_id}'.")
                    self._active_breakpoints.clear()
                    self._breakpoint_hit_id = None
                    self._run_state = "RUN" # Assume RUN if no other state was active
            
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
                else: # Clear all breakpoints
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
            "inputs": [conn.get("target_pin_id") for conn in self.inbound_connections],
            "outputs": list(self.outbound_map.keys()),
            "active_breakpoints": list(self._active_breakpoints.keys()),
            "breakpoint_hit_id": self._breakpoint_hit_id,
            "timestamp": time.time()
        }
        
        # Wrap the status in a PRP-like envelope for the ROSDaemon to forward
        envelope = {
            "prp": "0.1", # Add PRP version for consistency, though rosd adds it
            "type": "graph.node_state", # This will be used by ROSDaemon to set PRP type
            "payload": status_payload
        }
        
        msg = String()
        msg.data = json.dumps(envelope)
        self.status_publisher.publish(msg)

    def publish_to_pin(self, source_pin_id: str, data: Any):
        """
        Publishes data from a specific output pin to all connected nodes.
        The base class handles finding the destination nodes and pins.
        
        Args:
            source_pin_id: The ID of the output pin from which to send data.
            data: The payload to send. This should be JSON-serializable.
        """
        connections = self.outbound_map.get(source_pin_id)
        if not connections:
            self.get_logger().debug(f"No outbound connections for pin '{source_pin_id}' from node '{self.node_id}'")
            return

        for conn in connections:
            envelope = {
                "source_node_id": self.node_id,
                "source_pin_id": source_pin_id,
                "target_node_id": conn.get("target_node_id"),
                "target_pin_id": conn.get("target_pin_id"),
                "data": data
            }
            
            msg = String()
            msg.data = json.dumps(envelope)
            self.graph_publisher.publish(msg)
            self.get_logger().debug(f"Published from pin '{source_pin_id}' to node '{conn.get('target_node_id')}' on pin '{conn.get('target_pin_id')}'")

    def should_run(self, trigger_info: Optional[Dict[str, Any]] = None) -> bool:
        """
        Guard method to check if the node's main processing logic should execute.
        This respects the PAUSE, STEP, and BREAKPOINT commands from the control plane.
        
        Args:
            trigger_info: Optional dictionary with context about the execution trigger.
                          For input processing, this could be `{'pin_id': 'the_pin'}`.
                          This is used for evaluating conditional breakpoints.

        Returns:
            True if the node should execute its logic, False otherwise.
        """
        if self._run_state == "RUN":
            # Check for active breakpoints
            for bp_id, bp_details in self._active_breakpoints.items():
                if bp_details.get("hit"):
                    continue # Already hit, don't re-evaluate

                condition = bp_details.get("condition", {})
                condition_met = True # Default to true for unconditional breakpoints

                # --- Evaluate Breakpoint Condition ---
                # Check for an input pin condition.
                if 'input_pin_id' in condition:
                    # If condition requires a pin, but we don't have one, it's not a match.
                    if not trigger_info or trigger_info.get('pin_id') != condition['input_pin_id']:
                        condition_met = False
                
                # ... other conditions like data value checks could be added here ...

                if condition_met:
                    self._breakpoint_hit_id = bp_id
                    self._active_breakpoints[bp_id]["hit"] = True # Mark as hit
                    self._run_state = "BREAKPOINT_HIT"
                    self.get_logger().info(f"Node '{self.node_id}' hit breakpoint '{bp_id}'. Pausing execution.")
                    return False # Stop execution at breakpoint
            return True
        
        elif self._run_state == "PAUSE":
            if self._step_budget > 0:
                self._step_budget -= 1
                return True
            return False
        
        elif self._run_state == "BREAKPOINT_HIT":
            return False # Node is paused at a breakpoint

        return False # Default to not running if state is unknown or not RUN/PAUSE/BREAKPOINT_HIT

    @property
    def output_pins(self) -> List[str]:
        """Returns a list of available output pin IDs for this node."""
        return list(self.outbound_map.keys())

    @property
    def input_pins(self) -> List[str]:
        """Returns a list of available input pin IDs for this node."""
        # Extract unique target_pin_id from inbound_connections
        return list(set(conn.get("target_pin_id") for conn in self.inbound_connections))

    def process_input(self, pin_id: str, data: Any):
        """
        **User-overridable method.** This is where you implement your node's specific logic.
        It is called automatically when data arrives on one of the node's input pins
        and the node is in a 'RUN' state or has a 'step' budget.
        
        Args:
            pin_id: The ID of the input pin that received the data.
            data: The payload of the message.
        """
        self.get_logger().warn(f"Node '{self.node_id}' received data on pin '{pin_id}' but 'process_input' is not implemented.")
        # Example implementation (uncomment and modify in your subclass):
        # # Always check should_run() in your processing logic.
        # # Pass trigger_info for conditional breakpoints on specific pins.
        # if self.should_run(trigger_info={'pin_id': pin_id}):
        #      self.get_logger().info(f"Processing input on '{pin_id}': {data}")
        #      new_data = {"processed_by": self.node_id, "original_data": data}
        #      self.publish_to_pin("my_output_pin", new_data)

    async def run_async(self):
        """
        **User-overridable async method.** This is where you can implement long-running
        asynchronous tasks for your node, such as connecting to hardware. This method
        is called by `PolyflowNode.main()`.
        
        The default implementation keeps the node alive but performs no actions.
        """
        self.get_logger().info(f"Node '{self.node_id}' has no async tasks. 'run_async' is not implemented.")
        # Keep the coroutine alive so ROS subscriptions continue working
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

            # Spin ROS in the background so subscriptions/timers actually run
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
            # Re-raise the exception to ensure the process exits with a non-zero status
            raise
