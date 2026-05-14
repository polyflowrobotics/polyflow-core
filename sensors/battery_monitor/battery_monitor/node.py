"""
BatteryMonitorNode — generic graph-visible bridge for hardware BatteryState.

Discovers every /prp/hardware/state/<target_id>/battery_state topic on
the bus and republishes incoming sensor_msgs/BatteryState messages on a
single graph output pin. Downstream consumers can demultiplex by the
message's header.frame_id (set by each adapter to its target_id).

Adapter-agnostic: by default the node subscribes to all BatteryState
sources. An optional `target_id` parameter restricts it to a single
adapter target when more than one battery is on the bus.
"""

import asyncio
import re
from typing import Dict, Optional

from sensor_msgs.msg import BatteryState

from common.polyflow_node import PolyflowNode, _ros_msg_to_dict
from battery_monitor.kernel import BatteryMonitorKernel


BATTERY_TOPIC_RE = re.compile(r"^/prp/hardware/state/([^/]+)/battery_state$")
BATTERY_MSG_TYPE = "sensor_msgs/msg/BatteryState"


class BatteryMonitorNode(PolyflowNode):
    kernel_class = BatteryMonitorKernel

    def __init__(self):
        super().__init__()

        self.register_output_pin("battery_state", BatteryState)

        self._subscriptions_by_topic: Dict[str, object] = {}
        self._discovery_period_s = float(
            self.kernel.get_param("discovery_period_s", 2.0)
        )
        self._discovery_timer = self.create_timer(
            self._discovery_period_s, self._discover_topics
        )
        self._discover_topics()

        target_filter = self.kernel.target_id or "<all>"
        self.get_logger().info(
            f"BatteryMonitor listening for target_id={target_filter} "
            f"(discovery every {self._discovery_period_s}s)"
        )

    def _discover_topics(self) -> None:
        target_filter = self.kernel.target_id
        for topic_name, type_names in self.get_topic_names_and_types():
            if topic_name in self._subscriptions_by_topic:
                continue
            match = BATTERY_TOPIC_RE.match(topic_name)
            if not match:
                continue
            if BATTERY_MSG_TYPE not in type_names:
                continue
            target_id = match.group(1)
            if target_filter and target_id != target_filter:
                continue
            self._subscriptions_by_topic[topic_name] = self.create_subscription(
                BatteryState,
                topic_name,
                self._on_prp_battery_state,
                10,
            )
            self.get_logger().info(
                f"Discovered BatteryState source '{target_id}' on {topic_name}"
            )

    def _on_prp_battery_state(self, msg: BatteryState) -> None:
        self.kernel.update_state(_ros_msg_to_dict(msg))

    async def run_async(self):
        self.get_logger().info("BatteryMonitor ready")
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    BatteryMonitorNode.main(args=args)


if __name__ == "__main__":
    main()
