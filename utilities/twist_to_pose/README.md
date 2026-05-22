# Twist To Pose

Integrates a `geometry_msgs/Twist` (linear and angular velocity) into a running `geometry_msgs/Pose`. Bridges teleop sources (Gamepad, joystick) into pose-controlled orchestrators (Inverse Kinematics, mobile-base waypoints) — letting an operator "fly" the end-effector with a stick instead of placing absolute targets.

The kernel holds an internal pose, applies the latest Twist at the configured integration rate, and publishes the running pose every tick.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `cmd_vel` | subscribe | `geometry_msgs/Twist` | Latest velocity command (m/s, rad/s). |
| `seed_pose` | subscribe | `geometry_msgs/Pose` | Anchor for the integrator. The first message becomes the running pose; later messages are remembered as the falling-edge re-snap target. |
| `enable` | subscribe | `std_msgs/Bool` | Deadman gate. While `false`, velocity is treated as zero; on the falling edge, the pose snaps back to the last `seed_pose`. |
| `target_pose` | publish | `geometry_msgs/Pose` | Integrated pose, emitted once per tick. |

## Parameters

- **Integration Rate** (`rate_hz`) — how often to integrate the latest Twist. Default `50` Hz.
- **Twist Frame** (`frame`) — `body` interprets velocity in the current pose's frame (forward = where the EE points); `world` applies velocity to fixed world axes.
- **Require Seed Pose** (`require_seed`) — gate output until a `seed_pose` has been received. Prevents emitting `(0,0,0)` before the integrator is anchored. Default `true`.
- **Require Enable** (`require_enable`) — gate velocity on the `enable` Bool. On falling edge, the integrator re-snaps to the last `seed_pose` so re-enabling starts cleanly. Default `true`.
- **Workspace bounds** (`ws_min_x` … `ws_max_z`) — optional axis-aligned position clamps. Leave any field empty to leave that bound unconstrained.

## Typical use

`Gamepad.cmd_vel` → `Twist To Pose.cmd_vel`, `Inverse Kinematics.current_pose` → `Twist To Pose.seed_pose`, `Bool From Gamepad.value` → `Twist To Pose.enable`, and `Twist To Pose.target_pose` → `Inverse Kinematics.target_pose`. The result: hold a button, push a stick, and the arm flies in that direction relative to where the tip is currently pointing.
