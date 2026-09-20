# Protocol

`contracts/agent_link.proto` is the engine-neutral contract between a Drone Runtime and a Brain. It does not define Unity replication; the Environment Server and Unity Drone Runtime use a dedicated realtime transport and translate into these messages at the sensor/control boundary.

## Session handshake

1. Brain sends `AgentHello` with protocol version, identity, supported capabilities, and requested vehicle lease.
2. Runtime validates the session token out of band and returns `RuntimeHello` with accepted protocol version, vehicle profile, sensor manifest, and timing limits.
3. Runtime starts the bidirectional stream only after the lease is active.
4. Either side can reject unsupported major versions. Minor additive changes remain backward compatible.

## Stream directions

| Direction | Message | Rate | Reliability |
| --- | --- | ---: | --- |
| Runtime → Brain | `SensorPacket` | 20–60 Hz | latest data preferred; bounded queue |
| Runtime → Brain | `MissionCommand` | event-driven | reliable and acknowledged |
| Runtime → Brain | `ControlLease` | event-driven | reliable and ordered |
| Brain → Runtime | `ControlSetpoint` | 20–100 Hz | latest valid setpoint preferred |
| Brain → Runtime | `MissionStatus` | event-driven | reliable and ordered |
| both | `Heartbeat` | 1 Hz | reliable enough for liveness |

Transport policy is not encoded into domain messages. Initial local integration may use length-prefixed Protocol Buffers over a loopback stream. Shared-memory camera frames and a remote network adapter can be added behind the same `AgentLink` interface.

## Backpressure

Sensor queues are capacity one by default: a slow Brain receives the newest complete packet instead of accumulating latency. Mission and lifecycle messages use a separate reliable queue. Frame buffers are immutable until the consumer releases them or their lease expires.

## Coordinate convention

The public API uses right-handed ENU:

- +X east;
- +Y north;
- +Z up;
- heading is clockwise from north, in degrees.

The Unity adapter owns conversion to and from Unity coordinates. Units are SI.

## Ground-truth perception contract

`TargetObservation` is explicitly marked `GROUND_TRUTH_ADAPTER`. It represents what a future detector and geolocation pipeline would output, not unrestricted world access. A target absent from the current frame is absent from the packet. Image-space center coordinates are normalized to `[-1, 1]`. An optional ENU position estimate may be supplied only for a visible target and carries an error standard deviation so exact ground truth can later be replaced by a noisy estimate.

## Compatibility rules

- field numbers are never reused;
- enum value zero is always an unspecified/default state;
- removing or changing field meaning requires a new major protocol version;
- unknown enum values must fail safely;
- receivers reject non-finite numeric values and over-limit vectors;
- all IDs are opaque strings with bounded length.
