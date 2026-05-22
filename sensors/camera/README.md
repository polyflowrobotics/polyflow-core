# Camera

Captures frames from a local camera device (or any video source the host wrapper can read) and publishes them as `polyflow_msgs/CameraFrame` messages. Frames can be published on a free-running timer or pulled on demand by sending a message to the `capture` pin.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `capture` | subscribe | `std_msgs/Empty` | Trigger a single frame capture. |
| `frame` | publish | `polyflow_msgs/CameraFrame` | `{camera_id, width, height, encoding, data, connected}`. |

## Parameters

- **Camera ID** (`camera_id`) — string identifier included in each frame.
- **Device Index** (`device_index`) — OS-level camera index (0 = first camera).
- **Width / Height** (`width`, `height`) — capture resolution in pixels. Defaults `640 × 480`.
- **FPS** (`fps`) — capture frame rate. Default `30`.

## Typical use

Pair with the Studio `video` widget for a live viewer, or feed the `frame` pin into a vision pipeline. Send to `capture` from a mission step or button when you want one-shot snapshots instead of a continuous stream — leave the publish loop running for monitoring, or stop it and rely entirely on `capture` triggers.
