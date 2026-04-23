# polyflow-adapter-sdk

Base classes and helpers for writing Polyflow hardware adapters.

## What is an adapter?

A Polyflow hardware adapter is a Python class that runs inside the
polyflow-os `system-manager` daemon. It:

- Consumes typed PRP commands (e.g., `JointCommand`, `MotorCommand`) from
  the ROS node graph.
- Publishes typed PRP state (e.g., `JointState`) for downstream consumers.
- Owns (or uses) a physical hardware resource such as a serial port or
  CAN bus.

Adapters are hardware-specific; the node graph is not. An ODrive adapter
and a Dynamixel adapter both consume `JointCommand` and produce
`JointState` — any generic joint-controller node works with either.

## Writing an adapter

```python
from polyflow_adapter_sdk import HardwareAdapter, JOINT_MODE_POSITION

class MyJointAdapter(HardwareAdapter):
    driver_name = "my_joint_driver"
    command_types = ["JointCommand", "HardwareLifecycle"]
    state_types   = ["JointState"]

    def configure(self):
        self.port = self.params["port"]
        # open hardware handles here

    def start(self):
        self.schedule_poll(rate_hz=50, fn=self._poll)

    def on_command(self, command_type, data):
        if command_type == "JointCommand" and data.get("has_position"):
            self._write_position(data["position"])

    def _poll(self):
        self.publish_state("JointState", {
            "joint_id": self.target_id,
            "position": self._read_position(),
            "connected": True,
        })

    def shutdown(self):
        # close hardware handles
        pass
```

## adapter.json

Each adapter package declares the drivers it provides in a top-level
`adapter.json` file:

```json
{
  "schema_version": 1,
  "name": "my_package",
  "drivers": [
    { "name": "my_joint_driver", "class": "my_package.adapter:MyJointAdapter" }
  ]
}
```

`system-manager` scans `adapters/*/adapter.json` at startup, builds a
`driver_name -> class` registry, and instantiates adapters based on the
deployment's `hardware.yaml`.
