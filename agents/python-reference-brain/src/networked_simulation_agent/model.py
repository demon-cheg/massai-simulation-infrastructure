from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
import math


@dataclass(frozen=True, slots=True)
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def horizontal_length(self) -> float:
        return math.hypot(self.x, self.y)

    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def clamp_length(self, maximum: float) -> "Vec3":
        magnitude = self.length()
        if magnitude <= maximum or magnitude == 0.0:
            return self
        return self * (maximum / magnitude)


@dataclass(frozen=True, slots=True)
class VehicleState:
    position_enu_m: Vec3
    velocity_enu_mps: Vec3
    heading_deg: float
    grounded: bool
    camera_pitch_deg: float = -35.0
    battery_fraction: float = 1.0


@dataclass(frozen=True, slots=True)
class TargetObservation:
    object_id: str
    semantic_class: str
    center_x_normalized: float
    center_y_normalized: float
    width_normalized: float
    height_normalized: float
    confidence: float = 1.0
    simulated_range_m: float | None = None
    estimated_position_enu_m: Vec3 | None = None
    position_error_stddev_m: float | None = None

    @property
    def centered(self) -> bool:
        return abs(self.center_x_normalized) <= 0.12 and abs(self.center_y_normalized) <= 0.16


@dataclass(frozen=True, slots=True)
class SensorPacket:
    sequence: int
    simulation_tick: int
    simulation_time_s: float
    dt_s: float
    vehicle: VehicleState
    targets: tuple[TargetObservation, ...] = ()

    def find_target(self, object_id: str | None) -> TargetObservation | None:
        if object_id is None:
            return None
        return next((target for target in self.targets if target.object_id == object_id), None)


class MissionType(StrEnum):
    HOLD = "hold"
    TAKEOFF = "takeoff"
    NAVIGATE = "navigate"
    OBSERVE = "observe"
    TRACK = "track"
    TAG_OBJECT = "tag_object"
    RETURN_HOME = "return_home"
    LAND = "land"


class MissionState(StrEnum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True, slots=True)
class MissionCommand:
    command_id: str
    type: MissionType
    waypoint_enu_m: Vec3 | None = None
    target_altitude_m: float | None = None
    target_object_id: str | None = None
    desired_range_m: float = 12.0
    dwell_seconds: float = 1.0


@dataclass(frozen=True, slots=True)
class ControlSetpoint:
    desired_velocity_enu_mps: Vec3 = field(default_factory=Vec3)
    desired_yaw_rate_degps: float = 0.0
    desired_camera_pitch_rate_degps: float = 0.0
    valid_until_tick: int = 0


@dataclass(frozen=True, slots=True)
class MissionUpdate:
    command_id: str
    state: MissionState
    reason_code: str = ""
    detail: str = ""


@dataclass(frozen=True, slots=True)
class BrainOutput:
    setpoint: ControlSetpoint
    mission_update: MissionUpdate | None = None
    events: tuple[str, ...] = ()
