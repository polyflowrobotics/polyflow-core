# polyflow-core

A ROS 2 node library for the Polyflow robotics platform.

Polyflow lets you build a robot's behavior as a **graph of nodes**: each node is a
small, focused unit (a gamepad reader, a kinematics solver, a motor controller)
that consumes messages on input pins and produces messages on output pins. You
wire pins together in Polyflow Studio, deploy the graph to a robot, and the nodes
run as a connected ROS 2 system. This repo is the catalog of those nodes, plus
the shared base classes, message types, and hardware adapters they rely on.

If you're new here, the fastest way to understand a node is to read its own
`README.md` — every node package has one documenting its pins, parameters, and
typical wiring. This file is the map.

## Repository layout

```
common/            Shared base classes (PolyflowKernel + PolyflowNode)
polyflow_msgs/     Custom ROS 2 message types (.msg)
inputs/            Operator / external input nodes
sensors/           Nodes that read or forward sensor data
controllers/       Per-actuator command nodes
orchestrators/     Higher-level coordination & planning nodes
utilities/         Glue, logging, and type-bridging nodes
adapters/          Hardware adapters (run in the polyflow-os daemon, not the graph)
```

## How a node is structured

Every node is split into two classes — a portable **kernel** and a ROS **wrapper**.
This split is the central design idea of the codebase:

- **`*Kernel`** (e.g. `GamepadKernel`) — pure Python, no ROS or hardware
  dependencies. Holds all the decision logic. It receives messages as plain
  dicts via `process_input(pin_id, data)`, produces output via
  `self.emit(pin_id, data)`, and logs via `self.log(message)`. Because it has no
  ROS dependency, the exact same kernel runs in CPython (on a robot), in Pyodide
  (in the browser, against a simulated 3D scene), and in the cloud.

- **`*Node`** (e.g. `GamepadNode`) — the ROS 2 wrapper. It subclasses
  `PolyflowNode`, sets `kernel_class = GamepadKernel`, and handles the things the
  kernel can't: ROS publishers/subscribers, message ↔ dict conversion, and real
  hardware I/O.

Base classes live in [common/common/](common/common/):

- [polyflow_kernel.py](common/common/polyflow_kernel.py) — `PolyflowKernel`: the
  execution state machine (run/pause/step/breakpoints), parameter access, and the
  `process_input` / `emit` / `log` contract.
- [polyflow_node.py](common/common/polyflow_node.py) — `PolyflowNode`: the ROS
  wrapper. Auto-instantiates the node's `kernel_class`, converts ROS messages to
  dicts on the way in (`_ros_msg_to_dict`) and back on the way out
  (`_dict_to_ros_msg`), and routes emits to the right publisher.

A typical node package looks like:

```
inputs/gamepad/
  gamepad/
    kernel.py        # GamepadKernel — portable logic
    node.py          # GamepadNode — ROS wrapper
    evdev_source.py  # optional hardware/host helpers
  template.json      # node definition for Studio (pins, params, defaults)
  README.md          # pin/parameter reference for this node
  package.xml, setup.py, ...   # standard ament_python package files
```

To write a new node: subclass `PolyflowKernel` with your logic, subclass
`PolyflowNode` and point `kernel_class` at it, and describe the pins/params in
`template.json`. Keep all dict-shape conversion in the kernel; the node only
types dicts up into ROS messages.

## The nodes

### Inputs

| Node                                | What it does                                                                                                                                            |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Gamepad](inputs/gamepad/README.md) | Polls a gamepad, applies deadzones, and publishes axes, button states, and a derived `Twist` (tank-style for diff-drive, or full 6-DoF for arm teleop). |

### Sensors

| Node                                                 | What it does                                                                                                                       |
| ---------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| [Camera](sensors/camera/README.md)                   | Captures frames from a camera device and publishes `CameraFrame`s, either free-running or triggered on demand via a `capture` pin. |
| [Optical Encoder](sensors/optical_encoder/README.md) | Publishes a rotary encoder's tick count, integrated position, and velocity; accepts a reset to re-zero.                            |
| [Battery Monitor](sensors/battery_monitor/README.md) | Mirrors `BatteryState` telemetry from hardware adapters onto a graph pin, optionally filtered by target.                           |

### Controllers

| Node                                                       | What it does                                                                                                                                                                                                                                         |
| ---------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Joint Controller](controllers/joint_controller/README.md) | Hardware-agnostic single-joint controller. Picks its joint out of a multi-joint trajectory, clamps/quantizes to SI limits, smooths, and emits a `JointCommand`; forwards feedback as `JointState`. Runtime-switchable position/velocity/torque mode. |
| [Motor Controller](controllers/motor_controller/README.md) | Open-loop motor driver. Takes a scalar command (speed or duty), clamps/reverses it, and emits a `MotorCommand`; republishes measured state as a deadbanded float.                                                                                    |

### Orchestrators

| Node                                                             | What it does                                                                                                                                                |
| ---------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Differential Drive](orchestrators/differential_drive/README.md) | 4-wheel skid-steer kinematics. Turns `Twist` into per-wheel speeds, with a teleop-over-autonomy priority arbiter.                                           |
| [Inverse Kinematics](orchestrators/inverse_kinematics/README.md) | Damped-least-squares IK solver. Turns a target end-effector pose into per-joint trajectory points for the Joint Controllers; republishes the achieved pose. |
| [Mission Runner](orchestrators/mission_runner/README.md)         | Plays back a scripted sequence of timed `{command, duration}` driving steps as `Twist` commands with status output.                                         |

### Utilities

| Node                                                       | What it does                                                                                                                                                              |
| ---------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Twist To Pose](utilities/twist_to_pose/README.md)         | Integrates a `Twist` into a running `Pose` so an operator can "fly" an end-effector with a stick. Has seed-pose anchoring, a deadman gate, and optional workspace clamps. |
| [Bool From Gamepad](utilities/bool_from_gamepad/README.md) | Extracts one button from the Gamepad's `buttons` struct and republishes it as a plain `Bool` (e.g. a deadman for Twist To Pose).                                          |
| [Logger](utilities/logger/README.md)                       | Captures every node's `self.log(...)` output from the runtime log topic and writes it to stdout and/or a file. No pin wiring needed.                                      |

## Messages

Custom message types live in [polyflow_msgs/msg/](polyflow_msgs/msg/). Nodes also
use standard ROS types (`geometry_msgs/Twist`, `geometry_msgs/Pose`,
`sensor_msgs/JointState`, `sensor_msgs/BatteryState`, `std_msgs/*`, etc.) wherever
a standard type fits — the custom types are mostly for the hardware-bridge layer.

| Message                         | Used by                                  |
| ------------------------------- | ---------------------------------------- |
| `GamepadAxes`, `GamepadButtons` | Gamepad, Bool From Gamepad               |
| `CameraFrame`                   | Camera                                   |
| `EncoderState`                  | Optical Encoder                          |
| `JointCommand`, `JointState`    | Joint Controller ↔ adapters              |
| `MotorCommand`, `MotorState`    | Motor Controller ↔ adapters              |
| `ServoCommand`, `ServoState`    | servo adapters                           |
| `HardwareLifecycle`             | ENABLE / DISABLE / ESTOP across adapters |
| `LogEntry`                      | runtime log topic (Logger)               |

## Hardware adapters

Nodes are hardware-agnostic; **adapters** are where hardware-specific code lives.
An adapter is a Python class that runs inside the polyflow-os `system-manager`
daemon (not in the node graph). It consumes typed PRP commands (`JointCommand`,
`MotorCommand`, …), drives a physical resource (serial port, CAN bus), and
publishes typed state back. Because an ODrive adapter and a Hiwonder adapter both
speak `JointCommand`/`MotorCommand`, the same generic node works with either.

- [polyflow_adapter_sdk](adapters/polyflow_adapter_sdk/README.md) — base classes
  and helpers (`HardwareAdapter`) for writing adapters.
- [odrive](adapters/odrive/README.md) — ODrive S1 controllers over CAN or USB.
- [hiwonder_rrc](adapters/hiwonder_rrc/README.md) — Hiwonder RRC Lite board
  (motors, PWM/bus servos, IMU, battery) over USB serial.
