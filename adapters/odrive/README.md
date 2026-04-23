# polyflow-odrive-adapter

Polyflow hardware adapter for ODrive S1 controllers.

Provides two drivers:

- **`odrive_can`** — CAN transport via `python-can` + CANSimple protocol. One adapter instance per axis (node_id). Each instance opens its own CAN bus connection.
- **`odrive_usb`** — USB transport via the `odrive` Python package. One instance per USB-attached ODrive.

## hardware.yaml example

```yaml
hardware:
  - id: shoulder
    driver: odrive_can
    params:
      node_id: 0
      channel: can0
      bitrate: 1000000
      gear_ratio: 5.0
      torque_constant: 0.08
      poll_hz: 50

  - id: gripper
    driver: odrive_usb
    params:
      serial_number: "208036904C4D"   # optional; find_any() otherwise
      gear_ratio: 1.0
      poll_hz: 50
```

## Commands consumed

- `JointCommand` — position/velocity/torque commands in SI units. The adapter converts to ODrive-native turns using `gear_ratio` and `units`.
- `HardwareLifecycle` — ENABLE/DISABLE/ESTOP toggle closed-loop control.

## State published

- `JointState` — encoder position/velocity (converted back to SI), estimated effort (from Iq × torque_constant when available), and connection status derived from heartbeat age.
