# Gamepad

Teleop input. Polls a connected gamepad, applies deadzone filtering, and publishes raw axes, button states, and a derived `geometry_msgs/Twist` ready to feed into either a Differential Drive base or a 6-DoF pose controller.

Output is throttled: each pin re-publishes only when its value changes meaningfully or a 200 ms keepalive elapses. This keeps idle traffic low while still letting downstream consumers detect liveness from message arrival (Differential Drive's teleop/automated arbiter relies on this).

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `axes` | publish | `polyflow_msgs/GamepadAxes` | Deadzoned stick values: `left_x`, `left_y`, `right_x`, `right_y`. |
| `buttons` | publish | `polyflow_msgs/GamepadButtons` | Per-button booleans (`a`, `b`, `x`, `y`, `lb`, `rb`, `start`, `select`). |
| `cmd_vel` | publish | `geometry_msgs/Twist` | Computed velocity command — shape depends on `output_mode`. |

## Parameters

- **Device Index** (`device_index`) — OS-level gamepad index. Default `0`.
- **Poll Rate** (`poll_rate_hz`) — how often to read the device. Default `60` Hz.
- **Deadzone** (`deadzone`) — per-axis threshold below which values snap to zero. Default `0.05`.
- **Max Linear Speed** (`max_linear_speed`) — m/s scale for stick → linear velocity.
- **Max Angular Speed** (`max_angular_speed`) — rad/s scale for stick → angular velocity. Used only in `6dof` mode; in `diff_drive` mode `angular.z` is derived from `max_linear_speed` and `wheel_separation` so the Twist round-trips cleanly through standard skid-steer kinematics.
- **Wheel Separation** (`wheel_separation`) — distance between left and right wheels. Used only in `diff_drive` mode. **Must match the Differential Drive node's `wheel_separation`**, otherwise pushing one stick will spin both treads.
- **cmd_vel Mapping** (`output_mode`):
  - `diff_drive` (default) — tank style. Left stick Y drives the left tread, right stick Y drives the right tread; only `linear.x` + `angular.z` are filled.
  - `6dof` — full Twist. Left stick → linear X/Y, right stick → angular Y/Z, `LT`/`RT` → linear Z, `LB`/`RB` → angular X. For pose-controlled arms / IK.

## Typical use

Pair with Differential Drive (`cmd_vel` → `cmd_vel_teleop`) for a basic teleop loop. For arm teleop: switch to `6dof`, route `cmd_vel` into Twist To Pose, and use Bool From Gamepad to expose a deadman (`rb` is conventional) on Twist To Pose's `enable` pin.
