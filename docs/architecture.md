# Architecture and contracts

## Runtime pipeline

`IFrameSource` emits a timestamped `Frame`. `IPerceptionBackend` converts it into a health-aware `PerceptionResult`; `IPlanner` proposes a `ControlCommand`; `ISafetySupervisor` is the only component allowed to approve the command; `IActuatorSink` receives the final command. The replay application records every boundary into `telemetry.v1.jsonl`.

The core deliberately has no camera, serial, OpenCV, Paddle, ROS, or vendor SDK dependency. Those belong in optional adapters. A missing adapter is an explicit error, never a silent fallback to vehicle actuation.

## Safety contract

1. A stale or impossible timestamp is a hard fault and enters `LATCHED_STOP`.
2. A backend failure is a hard fault and enters `LATCHED_STOP`.
3. Short missing observations enter `DEGRADED`; repeated misses enter `SAFE_STOP`.
4. Every speed and steering command is clamped to a configured envelope.
5. Steering slew is limited between consecutive cycles.
6. `LATCHED_STOP` emits zero speed until `reset()` is called by an explicit owner.

## Extension points

- `OpenCVFrameSource` can translate camera/video frames into `Frame`.
- `OnnxPerceptionBackend` or `PaddleLitePerceptionBackend` can implement model inference without changing safety logic.
- A future board adapter may implement `IActuatorSink`; it must be opt-in and separately tested.
- Telemetry field names are versioned so a future VSS/KUKSA bridge can be added without changing the replay contract.
