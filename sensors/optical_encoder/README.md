# Optical Encoder

Publishes the state of a rotary optical encoder — raw tick count, integrated position (rad), and angular velocity (rad/s) — and accepts a reset command to zero the count.

The host node reads ticks from the underlying hardware (GPIO, USB encoder board, etc.) and feeds them to the kernel; the kernel handles tick-to-radians conversion using the configured resolution.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `reset` | subscribe | `std_msgs/Empty` | Zeroes ticks, position, and velocity. |
| `encoder_state` | publish | `polyflow_msgs/EncoderState` | `{encoder_id, ticks, position_rad, velocity_rad_s, connected}`. |

## Parameters

- **Encoder ID** (`encoder_id`) — string identifier for this encoder, included in every published message.
- **Ticks Per Revolution** (`ticks_per_rev`) — encoder resolution. Default `1024`.
- **Publish Rate** (`publish_rate_hz`) — how often `encoder_state` is published. Default `50` Hz.

## Typical use

Use as a wheel-odometry source for a Differential Drive, or as a joint feedback source where higher-level controllers want raw position/velocity rather than the abstracted JointState pipeline. Send `reset` from a button or mission step when you need to re-zero a known reference.
