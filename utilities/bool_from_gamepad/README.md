# Bool From Gamepad

Extracts a single button's pressed state from a `polyflow_msgs/GamepadButtons` struct and republishes it as a plain `std_msgs/Bool`. Bridges the typed gap between the Gamepad node's `buttons` output and any pin that expects a Bool — most commonly the `enable` deadman on Twist To Pose.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `buttons` | subscribe | `polyflow_msgs/GamepadButtons` | Full button struct from the Gamepad node. |
| `value` | publish | `std_msgs/Bool` | The selected button's state, optionally inverted. |

## Parameters

- **Button** (`button`) — which button to forward. One of `a`, `b`, `x`, `y`, `lb`, `rb`, `start`, `select`.
- **Invert** (`invert`) — if true, publishes the negation of the button state. Default `false`.
- **Emit Only on Change** (`emit_only_on_change`) — edge-triggered output. Default `true`; set false if you want a Bool every input frame (e.g. for keepalive).

## Typical use

Wire `Gamepad.buttons` → `Bool From Gamepad.buttons`, set the button you want (e.g. `rb`), and feed `value` into a downstream Bool input. Drop multiple Bool From Gamepad instances side-by-side to expose more than one button at a time.
