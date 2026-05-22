# Logger

Captures every `self.log(...)` call from every node in the graph and writes it to stdout and/or a file. The Logger subscribes to the system-wide `/prp/graph/runtime/log` topic — no pin wiring required, just drop the node in the graph and it picks up logs from the whole runtime.

## Pins

This node has no graph pins. It listens on the runtime log topic that every PolyflowNode publishes to automatically.

## Parameters

- **Log File Path** (`log_file`) — destination path for log entries (one JSON object per line). Leave blank to log only to stdout. Parent directories are created automatically.
- **Log to Stdout** (`log_to_stdout`) — also print entries to stdout. Default `true`.

## Typical use

Add a Logger to any graph where you want a permanent record of node-emitted log events. Combine `log_file` + `log_to_stdout=false` for headless deployments that ship logs off-box. Multiple Loggers can run side-by-side (e.g. one to a file, one to stdout) without contention.
