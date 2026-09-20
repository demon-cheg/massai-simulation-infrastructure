from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Iterable

from .brain import ReferenceBrain
from .model import (
    ControlSetpoint,
    MissionCommand,
    MissionState,
    MissionType,
    SensorPacket,
    TargetObservation,
    Vec3,
    VehicleState,
)


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


@dataclass(slots=True)
class SimulatedTarget:
    object_id: str
    position_enu_m: Vec3
    semantic_class: str


class KinematicHarness:
    """Deterministic integration harness; not the production flight model."""

    def __init__(
        self,
        *,
        tick_hz: int,
        spawn: Vec3,
        heading_deg: float,
        target: SimulatedTarget,
        horizontal_fov_deg: float = 90.0,
        vertical_fov_deg: float = 60.0,
        max_view_range_m: float = 80.0,
        camera_pitch_deg: float = -35.0,
    ) -> None:
        self.tick_hz = tick_hz
        self.dt_s = 1.0 / tick_hz
        self.position = spawn
        self.velocity = Vec3()
        self.heading_deg = heading_deg
        self.target = target
        self.horizontal_fov_deg = horizontal_fov_deg
        self.vertical_fov_deg = vertical_fov_deg
        self.max_view_range_m = max_view_range_m
        self.camera_pitch_deg = camera_pitch_deg
        self.tick = 0

    def packet(self) -> SensorPacket:
        observation = self._observe_target()
        targets = (observation,) if observation is not None else ()
        return SensorPacket(
            sequence=self.tick,
            simulation_tick=self.tick,
            simulation_time_s=self.tick * self.dt_s,
            dt_s=self.dt_s,
            vehicle=VehicleState(
                position_enu_m=self.position,
                velocity_enu_mps=self.velocity,
                heading_deg=self.heading_deg,
                grounded=self.position.z <= 0.001,
                camera_pitch_deg=self.camera_pitch_deg,
            ),
            targets=targets,
        )

    def apply(self, setpoint: ControlSetpoint) -> None:
        if setpoint.valid_until_tick < self.tick:
            self.velocity = Vec3()
            self.tick += 1
            return

        alpha = min(1.0, self.dt_s * 5.0)
        requested = setpoint.desired_velocity_enu_mps
        self.velocity = self.velocity * (1.0 - alpha) + requested * alpha
        self.heading_deg = (self.heading_deg + setpoint.desired_yaw_rate_degps * self.dt_s) % 360.0
        self.camera_pitch_deg = max(
            -90.0,
            min(30.0, self.camera_pitch_deg + setpoint.desired_camera_pitch_rate_degps * self.dt_s),
        )
        next_position = self.position + self.velocity * self.dt_s
        if next_position.z <= 0.0:
            next_position = Vec3(next_position.x, next_position.y, 0.0)
            self.velocity = Vec3(self.velocity.x, self.velocity.y, 0.0)
        self.position = next_position
        self.tick += 1

    def _observe_target(self) -> TargetObservation | None:
        relative = self.target.position_enu_m - self.position
        distance = relative.length()
        if distance <= 0.001 or distance > self.max_view_range_m:
            return None

        target_heading = math.degrees(math.atan2(relative.x, relative.y)) % 360.0
        horizontal_angle = _wrap_degrees(target_heading - self.heading_deg)
        horizontal_distance = max(0.001, relative.horizontal_length())
        vertical_angle = math.degrees(math.atan2(relative.z, horizontal_distance))
        camera_relative_vertical_angle = vertical_angle - self.camera_pitch_deg

        half_h = self.horizontal_fov_deg / 2.0
        half_v = self.vertical_fov_deg / 2.0
        if abs(horizontal_angle) > half_h or abs(camera_relative_vertical_angle) > half_v:
            return None

        apparent_size = min(0.35, max(0.025, 2.5 / distance))
        return TargetObservation(
            object_id=self.target.object_id,
            semantic_class=self.target.semantic_class,
            center_x_normalized=horizontal_angle / half_h,
            center_y_normalized=camera_relative_vertical_angle / half_v,
            width_normalized=apparent_size,
            height_normalized=apparent_size,
            simulated_range_m=distance,
            estimated_position_enu_m=self.target.position_enu_m,
            position_error_stddev_m=0.0,
        )


def load_scenario(path: Path) -> tuple[KinematicHarness, list[MissionCommand]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    spawn = Vec3(*raw["vehicle"]["spawn_enu_m"])
    target_config = raw["inspection_object"]
    target = SimulatedTarget(
        object_id=target_config["object_id"],
        position_enu_m=Vec3(*target_config["position_enu_m"]),
        semantic_class=target_config["semantic_class"],
    )
    harness = KinematicHarness(
        tick_hz=int(raw["tick_hz"]),
        spawn=spawn,
        heading_deg=float(raw["vehicle"]["heading_deg"]),
        target=target,
    )

    commands: list[MissionCommand] = []
    for index, item in enumerate(raw["mission"], start=1):
        waypoint = item.get("waypoint_enu_m")
        commands.append(
            MissionCommand(
                command_id=f"mission-{index:02d}",
                type=MissionType(item["type"]),
                waypoint_enu_m=Vec3(*waypoint) if waypoint else None,
                target_altitude_m=item.get("altitude_m"),
                target_object_id=item.get("target_object_id"),
                desired_range_m=float(item.get("desired_range_m", 12.0)),
                dwell_seconds=float(item.get("dwell_seconds", 1.0)),
            )
        )
    return harness, commands


def run_demo(path: Path, max_steps: int = 5000) -> dict[str, Any]:
    harness, commands = load_scenario(path)
    brain = ReferenceBrain()
    for command in commands:
        brain.submit(command)

    completed: list[str] = []
    event_log: list[dict[str, Any]] = []
    for _ in range(max_steps):
        packet = harness.packet()
        output = brain.step(packet)
        if output.mission_update is not None:
            entry = {
                "event": "mission.status",
                "tick": packet.simulation_tick,
                "command_id": output.mission_update.command_id,
                "state": output.mission_update.state.value,
            }
            event_log.append(entry)
            print(json.dumps(entry, separators=(",", ":")))
            if output.mission_update.state is MissionState.SUCCEEDED:
                completed.append(output.mission_update.command_id)
        for event in output.events:
            entry = {"event": event, "tick": packet.simulation_tick}
            event_log.append(entry)
            print(json.dumps(entry, separators=(",", ":")))
        harness.apply(output.setpoint)
        if brain.idle:
            summary = {
                "event": "demo.completed",
                "ticks": harness.tick,
                "simulation_time_s": round(harness.tick * harness.dt_s, 3),
                "commands_succeeded": len(completed),
                "final_position_enu_m": [
                    round(harness.position.x, 3),
                    round(harness.position.y, 3),
                    round(harness.position.z, 3),
                ],
            }
            print(json.dumps(summary, separators=(",", ":")))
            return summary

    raise RuntimeError(f"Demo did not finish within {max_steps} steps")


def mission_states(events: Iterable[dict[str, Any]]) -> dict[str, str]:
    return {
        event["command_id"]: event["state"]
        for event in events
        if event.get("event") == "mission.status"
    }
