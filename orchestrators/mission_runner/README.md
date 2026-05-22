# Mission Runner

Plays back a scripted sequence of timed driving commands. Each step is `{command, duration}`; commands are translated into `geometry_msgs/Twist` and published on `cmd_vel` for the configured duration, with a status string emitted at each transition. Intended for short, deterministic demos and recorded routines rather than full path planning.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `cmd_vel` | publish | `geometry_msgs/Twist` | Velocity command for the current step. |
| `status` | publish | `std_msgs/String` | Human-readable status (e.g. `"running step 2: forward"`). |

## Parameters

- **Mission** (`mission`) — JSON array of steps. Each step is `{"command": "forward" | "backward" | "turn_left" | "turn_right" | "stop", "duration": <seconds>}`. Invalid commands or missing `duration` fields are rejected at startup.
- **Linear Speed** (`linear_speed`) — m/s used by `forward` and `backward`. Default `0.5`.
- **Angular Speed** (`angular_speed`) — rad/s used by `turn_left` and `turn_right`. Default `1.0`.

## Typical use

Wire `cmd_vel` into a Differential Drive's `cmd_vel_automated` input. Teleop on the same drive transparently overrides the mission while held; release teleop and the script resumes. Use `status` to drive a Studio text widget for live "now performing" feedback during demos.
