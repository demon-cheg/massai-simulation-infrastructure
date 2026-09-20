# Architecture

## Purpose

The platform tests multiple autonomous systems inside one consistent simulated world while preserving the same boundary that exists on a physical vehicle: the autonomy workload receives sensors and emits control intent. It does not own the world, physics, or other actors.

## Runtime responsibilities

| Component | Owns | Must not own |
| --- | --- | --- |
| Environment Server | simulation clock, physics, collisions, dynamic actors, environmental parameters, spawn/lease validation, authoritative vehicle state | agent policy, operator UI, per-agent image analysis |
| Unity Drone Runtime | replicated local world, interpolation, onboard camera/sensor rendering, sensor timestamps, low-level vehicle adapter, operator video mirror | world truth, mission policy, arbitrary access to other agents' hidden state |
| Python Brain | mission state machine, sensor interpretation, control setpoints, health reporting | Unity scene access, authoritative transform writes, remote operator transport |
| Operator Console | observation, telemetry, mission submission, explicit time-bounded manual takeover | permanent vehicle ownership, simulation authority |
| Control Plane | session lifecycle, workload allocation, identities, endpoints, short-lived credentials | physics tick, high-rate state replication |

## Two planes

**Simulation plane** is latency-sensitive. It carries snapshots, commands, setpoints, sensor descriptors, and health. A temporary Control Plane outage must not pause an active simulation.

**Management plane** is lifecycle-oriented. It creates sessions, allocates workloads, issues identities, and gathers coarse status. It is not in the per-tick path.

## Authority and data flow

1. The Environment Server advances a fixed simulation tick and publishes authoritative snapshots.
2. Each Drone Runtime maintains an interpolated replica suitable for local sensor rendering.
3. The runtime renders only sensors configured for its leased vehicle.
4. The runtime sends a timestamped `SensorPacket` to its local or remote Brain through an `AgentLink` adapter.
5. The Brain returns a bounded `ControlSetpoint`; it never sends a transform.
6. The runtime converts the setpoint into a vehicle input command and sends it to the Environment Server.
7. The server validates identity, lease, sequence, age, and value limits before applying input at a tick boundary.

## Camera and perception boundary

The logical camera belongs to the simulated drone. Technically, it is rendered by that drone's Unity Runtime from a local replica of the shared world. This preserves onboard semantics without giving the Python Brain a copy of the Unity scene graph.

The first milestone uses `GroundTruthPerceptionAdapter` instead of computer vision. It may expose a target observation only when all of these conditions hold:

- the target is inside the configured camera frustum;
- the target passes a line-of-sight query;
- the target belongs to an allowed semantic class;
- the observation is emitted with the same timestamp and frame id as the corresponding image.

The adapter reports image-space coordinates, optional simulated range, and an optional target-position estimate. The estimate is emitted only while the object is visible, is explicitly marked as ground-truth-derived, and can be degraded with configurable noise, latency, and dropout. Later, a real detector and geolocation pipeline can replace the adapter without changing the Brain's mission controller.

## Control hierarchy

The Brain emits a platform-neutral setpoint:

- desired velocity in ENU coordinates;
- desired yaw rate;
- desired camera-gimbal pitch rate;
- optional altitude hold;
- monotonically increasing sequence and expiry time.

A vehicle-specific flight controller turns that setpoint into forces/torques or rotor commands. This keeps the reference Brain simple while leaving room for physically richer vehicle models.

## Time model

Every high-rate message carries:

- `simulation_tick`: authoritative discrete tick;
- `simulation_time_ns`: deterministic scenario time;
- `monotonic_time_ns`: local latency measurement only;
- `sequence`: stream-local ordering;
- `valid_until_tick`: stale-command rejection.

Wall-clock timestamps are optional observability metadata and never drive physics.

## Deployment topologies

### Local developer mode

- Environment Server runs in the Unity Editor for inspection.
- each Drone Runtime is a standalone Unity process;
- each Brain is a standalone Python process;
- PowerShell starts and connects them using explicit session and agent configuration.

### Isolated cloud mode

- one headless Environment Server per pod;
- one Drone Node per pod;
- a Drone Node contains a Unity Runtime container plus a Python Brain sidecar;
- local sensor transport can use shared memory while network replication uses the pod network.

### Packed training mode (later)

One sensor worker renders cameras for several logical drones while each Brain retains a separate identity, budget, and observation stream. This trades isolation for GPU utilization and must be benchmarked before use.

## Failure behavior

| Failure | Required behavior |
| --- | --- |
| Brain misses deadline | runtime holds the last safe setpoint briefly, then enters hover/failsafe |
| Runtime disconnects | server invalidates its lease and applies scenario failsafe |
| stale/out-of-order input | server rejects and increments a metric |
| target observation disappears | Brain stops tag dwell and returns to search/hold |
| operator takes control | lease epoch increments; Brain output is ignored until control is returned |
| Control Plane unavailable | active session continues; new allocation is unavailable |

## Security baseline

- session-scoped agent identity and vehicle lease;
- short-lived credentials supplied at process start, never committed;
- explicit protocol versions and maximum message sizes;
- rate, sequence, expiry, and numeric-range validation;
- separate observer and controller permissions;
- auditable control-lease transitions;
- no public inbound port for the Python sidecar in isolated mode.
