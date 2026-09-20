from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from .model import (
    BrainOutput,
    ControlSetpoint,
    MissionCommand,
    MissionState,
    MissionType,
    MissionUpdate,
    SensorPacket,
    TargetObservation,
    Vec3,
)


@dataclass(slots=True)
class ControllerLimits:
    max_horizontal_speed_mps: float = 6.0
    max_vertical_speed_mps: float = 2.5
    max_yaw_rate_degps: float = 80.0
    navigation_gain: float = 0.65
    altitude_gain: float = 0.8
    image_yaw_gain: float = 70.0
    image_gimbal_gain: float = 55.0
    range_gain: float = 0.35


class ReferenceBrain:
    """Simple deterministic mission controller, intentionally not an ML model."""

    def __init__(self, limits: ControllerLimits | None = None) -> None:
        self._limits = limits or ControllerLimits()
        self._queue: deque[MissionCommand] = deque()
        self._active: MissionCommand | None = None
        self._active_started = False
        self._dwell_s = 0.0
        self._home: Vec3 | None = None
        self._cruise_altitude_m = 10.0

    @property
    def idle(self) -> bool:
        return self._active is None and not self._queue

    def submit(self, command: MissionCommand) -> None:
        self._validate(command)
        self._queue.append(command)

    def step(self, packet: SensorPacket) -> BrainOutput:
        if self._home is None:
            self._home = packet.vehicle.position_enu_m

        update: MissionUpdate | None = None
        if self._active is None and self._queue:
            self._active = self._queue.popleft()
            self._active_started = False
            self._dwell_s = 0.0

        if self._active is None:
            return BrainOutput(self._hold(packet))

        if not self._active_started:
            self._active_started = True
            update = MissionUpdate(self._active.command_id, MissionState.RUNNING)

        setpoint, succeeded, events = self._execute(self._active, packet)
        if succeeded:
            update = MissionUpdate(self._active.command_id, MissionState.SUCCEEDED)
            self._active = None
            self._active_started = False
            self._dwell_s = 0.0

        return BrainOutput(setpoint, update, tuple(events))

    def _execute(
        self, command: MissionCommand, packet: SensorPacket
    ) -> tuple[ControlSetpoint, bool, list[str]]:
        if command.type is MissionType.HOLD:
            return self._hold(packet), True, []
        if command.type is MissionType.TAKEOFF:
            return self._takeoff(command, packet)
        if command.type is MissionType.NAVIGATE:
            return self._navigate(command.waypoint_enu_m, packet)
        if command.type is MissionType.OBSERVE:
            return self._observe(command, packet)
        if command.type is MissionType.TRACK:
            return self._track(command, packet, complete_on_dwell=True)
        if command.type is MissionType.TAG_OBJECT:
            setpoint, succeeded, _ = self._track(command, packet, complete_on_dwell=True)
            events = [f"object.tagged:{command.target_object_id}"] if succeeded else []
            return setpoint, succeeded, events
        if command.type is MissionType.RETURN_HOME:
            assert self._home is not None
            target = Vec3(self._home.x, self._home.y, self._cruise_altitude_m)
            return self._navigate(target, packet)
        if command.type is MissionType.LAND:
            return self._land(packet)
        raise ValueError(f"Unsupported mission type: {command.type}")

    def _takeoff(
        self, command: MissionCommand, packet: SensorPacket
    ) -> tuple[ControlSetpoint, bool, list[str]]:
        target_altitude = command.target_altitude_m or self._cruise_altitude_m
        self._cruise_altitude_m = target_altitude
        error = target_altitude - packet.vehicle.position_enu_m.z
        succeeded = abs(error) <= 0.25 and abs(packet.vehicle.velocity_enu_mps.z) <= 0.3
        vertical = self._clamp(error * self._limits.altitude_gain, self._limits.max_vertical_speed_mps)
        return self._setpoint(packet, Vec3(0.0, 0.0, vertical)), succeeded, []

    def _navigate(
        self, target: Vec3 | None, packet: SensorPacket
    ) -> tuple[ControlSetpoint, bool, list[str]]:
        if target is None:
            raise ValueError("navigate requires waypoint_enu_m")
        error = target - packet.vehicle.position_enu_m
        horizontal_error = Vec3(error.x, error.y, 0.0)
        horizontal_velocity = (horizontal_error * self._limits.navigation_gain).clamp_length(
            self._limits.max_horizontal_speed_mps
        )
        vertical = self._clamp(
            error.z * self._limits.altitude_gain, self._limits.max_vertical_speed_mps
        )
        velocity = Vec3(horizontal_velocity.x, horizontal_velocity.y, vertical)
        desired_heading = self._heading_for_velocity(velocity, packet.vehicle.heading_deg)
        yaw_rate = self._yaw_rate_to(desired_heading, packet.vehicle.heading_deg)
        succeeded = error.length() <= 0.75 and packet.vehicle.velocity_enu_mps.length() <= 0.8
        return self._setpoint(packet, velocity, yaw_rate), succeeded, []

    def _observe(
        self, command: MissionCommand, packet: SensorPacket
    ) -> tuple[ControlSetpoint, bool, list[str]]:
        target = packet.find_target(command.target_object_id)
        if target is None:
            self._dwell_s = 0.0
            search_rate = 22.0
            return self._setpoint(packet, Vec3(), search_rate), False, []

        yaw_rate = self._clamp(
            target.center_x_normalized * self._limits.image_yaw_gain,
            self._limits.max_yaw_rate_degps,
        )
        gimbal_rate = self._clamp(target.center_y_normalized * self._limits.image_gimbal_gain, 60.0)
        if target.centered:
            self._dwell_s += packet.dt_s
        else:
            self._dwell_s = 0.0
        return (
            self._setpoint(packet, Vec3(), yaw_rate, gimbal_rate),
            self._dwell_s >= command.dwell_seconds,
            [],
        )

    def _track(
        self,
        command: MissionCommand,
        packet: SensorPacket,
        *,
        complete_on_dwell: bool,
    ) -> tuple[ControlSetpoint, bool, list[str]]:
        target = packet.find_target(command.target_object_id)
        if target is None:
            self._dwell_s = 0.0
            return self._setpoint(packet, Vec3(), 18.0), False, []

        yaw_rate = self._clamp(
            target.center_x_normalized * self._limits.image_yaw_gain,
            self._limits.max_yaw_rate_degps,
        )
        gimbal_rate = self._clamp(target.center_y_normalized * self._limits.image_gimbal_gain, 60.0)
        forward_speed = 0.0
        range_ok = target.simulated_range_m is None
        if target.simulated_range_m is not None:
            range_error = target.simulated_range_m - command.desired_range_m
            forward_speed = self._clamp(range_error * self._limits.range_gain, 2.5)
            range_ok = abs(range_error) <= 1.0

        heading_rad = math.radians(packet.vehicle.heading_deg)
        forward_enu = Vec3(math.sin(heading_rad), math.cos(heading_rad), 0.0)
        velocity = forward_enu * forward_speed
        stable = target.centered and range_ok
        self._dwell_s = self._dwell_s + packet.dt_s if stable else 0.0
        succeeded = complete_on_dwell and self._dwell_s >= command.dwell_seconds
        return self._setpoint(packet, velocity, yaw_rate, gimbal_rate), succeeded, []

    def _land(self, packet: SensorPacket) -> tuple[ControlSetpoint, bool, list[str]]:
        if packet.vehicle.grounded:
            return self._setpoint(packet, Vec3()), True, []
        altitude = packet.vehicle.position_enu_m.z
        descent = -min(1.2, max(0.25, altitude * 0.35))
        return self._setpoint(packet, Vec3(0.0, 0.0, descent)), False, []

    def _hold(self, packet: SensorPacket) -> ControlSetpoint:
        return self._setpoint(packet, Vec3())

    @staticmethod
    def _validate(command: MissionCommand) -> None:
        if command.type is MissionType.NAVIGATE and command.waypoint_enu_m is None:
            raise ValueError("navigate requires waypoint_enu_m")
        if command.type in {MissionType.OBSERVE, MissionType.TRACK, MissionType.TAG_OBJECT}:
            if not command.target_object_id:
                raise ValueError(f"{command.type} requires target_object_id")
        if command.dwell_seconds < 0.0:
            raise ValueError("dwell_seconds must be non-negative")

    def _setpoint(
        self,
        packet: SensorPacket,
        velocity: Vec3,
        yaw_rate: float = 0.0,
        camera_pitch_rate: float = 0.0,
    ) -> ControlSetpoint:
        return ControlSetpoint(
            desired_velocity_enu_mps=velocity,
            desired_yaw_rate_degps=self._clamp(yaw_rate, self._limits.max_yaw_rate_degps),
            desired_camera_pitch_rate_degps=self._clamp(camera_pitch_rate, 60.0),
            valid_until_tick=packet.simulation_tick + 3,
        )

    def _yaw_rate_to(self, desired_deg: float, current_deg: float) -> float:
        error = (desired_deg - current_deg + 180.0) % 360.0 - 180.0
        return self._clamp(error * 2.0, self._limits.max_yaw_rate_degps)

    @staticmethod
    def _heading_for_velocity(velocity: Vec3, fallback_deg: float) -> float:
        if velocity.horizontal_length() < 0.05:
            return fallback_deg
        return math.degrees(math.atan2(velocity.x, velocity.y)) % 360.0

    @staticmethod
    def _clamp(value: float, maximum_absolute: float) -> float:
        return max(-maximum_absolute, min(maximum_absolute, value))
