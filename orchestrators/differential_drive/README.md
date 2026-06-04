# Differential Drive

4-wheel skid-steer kinematics. Consumes `geometry_msgs/Twist` commands and emits per-wheel angular-velocity setpoints (rad/s) on four motor pins. The two Twist inputs implement a priority arbiter: teleop wins whenever a recent message has arrived; otherwise the automated input is used.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `cmd_vel_teleop` | subscribe | `geometry_msgs/Twist` | Operator / gamepad command. Takes priority while fresh. |
| `cmd_vel_automated` | subscribe | `geometry_msgs/Twist` | Autonomous planner command. Used when teleop is stale or has never published. |
| `front_left_motor` | publish | `std_msgs/Float64` | Left-side wheel speed (rad/s). |
| `back_left_motor` | publish | `std_msgs/Float64` | Left-side wheel speed (rad/s). |
| `front_right_motor` | publish | `std_msgs/Float64` | Right-side wheel speed (rad/s). |
| `back_right_motor` | publish | `std_msgs/Float64` | Right-side wheel speed (rad/s). |

## Parameters

- **Wheel Radius** (`wheel_radius`) — wheel radius in meters. Default `0.05`.
- **Wheel Separation** (`wheel_separation`) — distance between left and right wheels in meters. Default `0.3`. **Must match the Gamepad node's `wheel_separation` in tank mode**, otherwise the Twist round-trip won't be clean.
- **Max Wheel Speed** (`max_wheel_speed`) — per-wheel clamp in m/s before conversion to rad/s. Default `1.0`.
- **Teleop Timeout** (`teleop_timeout_s`) — seconds without a teleop message before falling back to the automated input. Default `1.0`.

## Typical use

Wire `Gamepad.cmd_vel` (in `diff_drive` mode) → `cmd_vel_teleop`, your planner / Mission Runner → `cmd_vel_automated`, and each motor pin → its corresponding Motor Controller. Operator input transparently overrides autonomy without needing a manual mode switch.
