import asyncio
import json
import os

from polyflow_msgs.msg import LogEntry
from common.polyflow_node import PolyflowNode


class LoggerNode(PolyflowNode):
    """
    Logger utility node.

    Subscribes to the system-level /graph/log topic (published by any node
    via self.log()) and writes entries to a log file and/or stdout.
    No pin wiring required — all nodes can log without explicit connections.

    Parameters (via POLYFLOW_PARAMETERS):
        log_file:           Path to the log file (default: None, stdout only).
        log_to_stdout:      Whether to also print to stdout (default: True).
    """

    def __init__(self):
        super().__init__()

        self.log_file = self.get_param("log_file", None)
        self.log_to_stdout = self.get_param("log_to_stdout", True)

        self._file_handle = None
        self._message_count = 0

        # Subscribe to the shared system log topic (not a pin)
        self.create_subscription(
            LogEntry,
            '/graph/log',
            self._on_log_entry,
            10
        )

        self.get_logger().info(
            f"Logger | file={self.log_file or 'stdout only'} | stdout={self.log_to_stdout}"
        )

    def _on_log_entry(self, msg: LogEntry):
        entry = {
            "node_id": msg.pin_id,
            "data": msg.data,
            "timestamp": msg.timestamp,
        }
        entry_str = json.dumps(entry)
        self._message_count += 1

        if self.log_to_stdout:
            self.get_logger().info(f"[LOG] {entry_str}")

        if self._file_handle:
            self._file_handle.write(entry_str + "\n")
            self._file_handle.flush()

    async def run_async(self):
        if self.log_file:
            log_dir = os.path.dirname(self.log_file)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            self._file_handle = open(self.log_file, "a")
            self.get_logger().info(f"Logging to file: {self.log_file}")

        self.get_logger().info("Logger ready")

        try:
            while True:
                await asyncio.sleep(1.0)
        finally:
            if self._file_handle:
                self._file_handle.close()


def main(args=None):
    LoggerNode.main(args)
