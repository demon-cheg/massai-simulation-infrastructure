# Delivery plan

## Stage 1 — contracts and reference Brain

Deliverables: architecture boundaries, Protocol Buffers schema, deterministic mission FSM, closed-loop harness, and tests.

Exit gate: one command completes take-off → navigate → observe → track → virtual tag → return → land, with no Unity dependency.

## Stage 2 — authoritative Unity vertical slice

Deliverables: one Unity project with Environment Server and Drone Runtime build modes, fixed-tick world, one quadrotor model, snapshot replication, lease validation, and two connected runtimes.

Exit gate: two drones see the same moving inspection object and cannot authoritatively modify it.

## Stage 3 — real Agent Link

Deliverables: generated protocol bindings, bounded sensor stream, control setpoint stream, frame transport adapter, health/deadline handling, and Python Brain integration.

Exit gate: the Stage 1 Brain completes the same mission through Unity without code changes in its controller.

## Stage 4 — four-agent demo and operator path

Deliverables: PowerShell launch/scale commands, four independent agents, browser telemetry, compressed video mirror, and explicit manual control lease.

Exit gate: closing the browser does not stop agents; takeover and return-to-autonomy are auditable.

## Stage 5 — adverse conditions and realism

Deliverables: wind field, sensor noise, control delay, packet loss/bandwidth profiles, replay, and deterministic scenario seeds.

Exit gate: one recorded scenario can be replayed and compared across two Brain versions.

## Stage 6 — deployment and evidence

Deliverables: containers, Compose, Kubernetes manifests, Environment allocation strategy, workload health, CI scenario, load generator, metrics dashboard, benchmark report, threat model, and a short demo script.

Exit gate: architecture claims are tied to measured p50/p95/p99 latency, tick stability, bandwidth, CPU/GPU memory, reconnect behavior, and tested agent counts.

