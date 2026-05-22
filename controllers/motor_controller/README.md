# Motor Controller

Open-loop motor driver. Takes a scalar `std_msgs/Float64` command, clamps and optionally reverses it, and emits a `MotorCommand` to the hardware adapter over PRP. Republishes measured motor state as a clean float scalar (with a small deadband around zero to suppress sensor noise) so it can be charted directly in Studio.

Unlike Joint Controller, this node has no notion of position or feedback control — it just pushes a speed or duty value through to the driver.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `command` | subscribe | `std_msgs/Float64` | Scalar command (rad/s in `speed` mode, `[-1, 1]` in `duty` mode). |
| `state` | publish | `std_msgs/Float64` | Measured value, deadbanded around zero. |

## Parameters

- **Motor** (`motor_id`) — joint this controller drives in open-loop speed/duty mode. Resolves to a joint IO on a Part of the Robot; serialized to `hardware.json` at deploy.
- **Mode** (`mode`) — `speed` (rad/s, clamped to `max_speed`) or `duty` (clamped to `[-1, 1]`).
- **Max Speed** (`max_speed`) — read-only, bound from the joint's `max_velocity` spec. Used only in `speed` mode.
- **Reverse Direction** (`reverse`) — negate command before clamping. Use for motors mounted mirrored (e.g. left-side drive wheels whose joint axis points opposite the right side).
- **Command Timeout** (`timeout_s`) — watchdog seconds applied to every command. `0` keeps the driver default.

## Typical use

Wire each `*_motor` pin from a Differential Drive into its own Motor Controller's `command` pin. Set `reverse=true` on the left-side controllers so both treads spin the right way in response to a positive linear velocity. Pipe `state` into a Studio chart to compare commanded vs. measured speeds.
