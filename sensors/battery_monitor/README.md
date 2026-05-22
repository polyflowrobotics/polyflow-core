# Battery Monitor

Mirrors `sensor_msgs/BatteryState` messages from one or more hardware adapters onto a graph pin. Each message keeps its original `header.frame_id` so downstream consumers can identify which battery (which adapter target) it came from.

This node is a thin bridge — it doesn't read hardware directly. A hardware adapter (RRC, ODrive, a smart-BMS driver, etc.) publishes `BatteryState` on the PRP hardware bus; the Battery Monitor subscribes, optionally filters by target, and re-emits on the graph.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `battery_state` | publish | `sensor_msgs/BatteryState` | Forwarded battery telemetry (voltage, current, charge %, temperature, etc.). |

## Parameters

- **Target ID Filter** (`target_id`) — adapter target to listen to (e.g. `rrc0`, `main_battery`). Leave blank to mirror every `BatteryState` on the bus.
- **Discovery Period** (`discovery_period_s`) — how often to rescan the bus for new `BatteryState` topics. Default `2.0` s.

## Typical use

Wire the `battery_state` pin to a Studio battery widget for live charge/voltage display, or fan it into a Mission Runner / safety node that should abort on low battery. Run one Battery Monitor per battery you want to chart separately, or run a single unfiltered one to aggregate everything onto one widget.
