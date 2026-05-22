# Joint Controller

Hardware-agnostic single-joint controller. Takes a `trajectory_msgs/JointTrajectoryPoint` (which may carry several joints), picks out the entry for *this* joint, clamps and quantizes the setpoint against the joint's SI-unit limits, optionally smooths it, and emits a `JointCommand` for the hardware adapter to consume over PRP. Also forwards hardware state feedback to the graph as a single-joint `sensor_msgs/JointState`.

Control mode is runtime-switchable via the `mode` pin.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `trajectory` | subscribe | `trajectory_msgs/JointTrajectoryPoint` | Multi-joint setpoint. This node consumes the entry whose name matches `joint_id`. |
| `mode` | subscribe | `std_msgs/String` | Runtime mode switch: `"position"`, `"velocity"`, or `"torque"`. |
| `state` | publish | `sensor_msgs/JointState` | Feedback as a single-joint array (`name=[joint_id]`, etc.). |

## Parameters

- **Joint** (`joint_id`) — joint this controller manages. Resolves against the Robot's IO and serializes to `hardware.json` at deploy.
- **Default Mode** (`default_mode`) — initial control mode (`position` / `velocity` / `torque`). Overridable at runtime via `mode`.
- **Smoothing Factor** (`smoothing_alpha`) — exponential smoothing on the commanded value. `0.0` = no smoothing, `0.9` = heavy. Recommended `0.3`–`0.7` for smooth motion; higher values trade response for smoothness.
- **Limits group** (`min_angle`, `max_angle`, `position_step`, `max_effort`, `effort_step`, `max_velocity`, `velocity_step`) — read-only, bound from the joint's specs in the Robot definition. All SI units (`rad`, `rad/s`, `Nm`). The kernel clamps and quantizes to these on every command.

## Typical use

One Joint Controller per actuated joint, all subscribed to the same `joint_commands` trajectory pin from Inverse Kinematics (each controller picks its joint by name). Fan their `state` pins back into IK's `joint_states` for closed-loop seeding. Use the `mode` pin to switch a joint between position holding and torque-limited compliance from a mission step or operator control.
