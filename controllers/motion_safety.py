"""Versioned, simulator-independent motion safety configuration."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a finite number") from exc


@dataclass(frozen=True)
class MotionSafetyConfig:
    version: str = "motion-safety-v3"
    robot_radius_m: float = 0.18
    payload_overhang_m: float = 0.0
    maximum_speed_mps: float = 0.22
    joint_turn_rate_rad_s: float = 1.988
    braking_deceleration_mps2: float = 0.50
    reaction_latency_s: float = 0.20
    joint_time_slot_s: float = 4.75
    hard_collision_distance_m: float = 0.50
    coordinated_stop_distance_m: float = 0.55
    planning_clearance_m: float = 0.70
    wide_planning_clearance_m: float = 0.75
    peer_radial_stop_distance_m: float = 0.80
    prediction_clearance_m: float = 0.85
    static_margin_m: float = 0.55
    lidar_hard_stop_m: float = 0.25
    dwa_critical_distance_m: float = 0.40
    stale_caution_age_s: float = 0.10
    stale_stop_age_s: float = 0.30

    def __post_init__(self) -> None:
        values = asdict(self)
        numeric = {key: value for key, value in values.items()
                   if key != "version"}
        if any(not isinstance(value, (int, float)) or
               not math.isfinite(float(value)) or value <= 0
               for key, value in numeric.items()
               if key not in {"payload_overhang_m"}):
            raise ValueError("motion safety values must be positive")
        if (not math.isfinite(self.payload_overhang_m) or
                self.payload_overhang_m < 0):
            raise ValueError("payload_overhang_m cannot be negative")
        if not (self.hard_collision_distance_m <=
                self.coordinated_stop_distance_m <=
                self.planning_clearance_m <=
                self.wide_planning_clearance_m <=
                self.peer_radial_stop_distance_m <=
                self.prediction_clearance_m):
            raise ValueError("dynamic safety distances must be monotonic")
        if self.stale_stop_age_s <= self.stale_caution_age_s:
            raise ValueError("stale stop age must exceed caution age")
        if self.joint_time_slot_s < self.worst_case_cardinal_slot_s:
            raise ValueError("joint time slot is shorter than motion bound")

    @property
    def stopping_distance_m(self) -> float:
        speed = self.maximum_speed_mps
        return (speed * self.reaction_latency_s +
                speed * speed / (2.0 * self.braking_deceleration_mps2))

    @property
    def footprint_radius_m(self) -> float:
        return self.robot_radius_m + self.payload_overhang_m

    @property
    def worst_case_cardinal_slot_s(self) -> float:
        # A conservative 180-degree realignment followed by one 0.25 m cell.
        return (math.pi / self.joint_turn_rate_rad_s +
                0.25 / self.maximum_speed_mps + self.reaction_latency_s)

    def fingerprint(self) -> str:
        payload = json.dumps(
            asdict(self), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def resolved(self) -> dict:
        result = asdict(self)
        result["stopping_distance_m"] = self.stopping_distance_m
        result["footprint_radius_m"] = self.footprint_radius_m
        result["worst_case_cardinal_slot_s"] = self.worst_case_cardinal_slot_s
        result["fingerprint"] = self.fingerprint()
        return result


def load_motion_safety_config() -> MotionSafetyConfig:
    prefix = "SMART_FACTORY_SAFETY_"
    return MotionSafetyConfig(
        robot_radius_m=_env_float(prefix + "ROBOT_RADIUS_M", 0.18),
        payload_overhang_m=_env_float(prefix + "PAYLOAD_OVERHANG_M", 0.0),
        maximum_speed_mps=_env_float(prefix + "MAXIMUM_SPEED_MPS", 0.22),
        joint_turn_rate_rad_s=_env_float(
            prefix + "JOINT_TURN_RATE_RAD_S", 1.988),
        braking_deceleration_mps2=_env_float(
            prefix + "BRAKING_DECELERATION_MPS2", 0.50),
        reaction_latency_s=_env_float(prefix + "REACTION_LATENCY_S", 0.20),
        joint_time_slot_s=_env_float(prefix + "JOINT_TIME_SLOT_S", 4.75),
        hard_collision_distance_m=_env_float(
            prefix + "HARD_COLLISION_DISTANCE_M", 0.50),
        coordinated_stop_distance_m=_env_float(
            prefix + "COORDINATED_STOP_DISTANCE_M", 0.55),
        planning_clearance_m=_env_float(
            prefix + "PLANNING_CLEARANCE_M", 0.70),
        wide_planning_clearance_m=_env_float(
            prefix + "WIDE_PLANNING_CLEARANCE_M", 0.75),
        peer_radial_stop_distance_m=_env_float(
            prefix + "PEER_RADIAL_STOP_DISTANCE_M", 0.80),
        prediction_clearance_m=_env_float(
            prefix + "PREDICTION_CLEARANCE_M", 0.85),
        static_margin_m=_env_float(prefix + "STATIC_MARGIN_M", 0.55),
        lidar_hard_stop_m=_env_float(prefix + "LIDAR_HARD_STOP_M", 0.25),
        dwa_critical_distance_m=_env_float(
            prefix + "DWA_CRITICAL_DISTANCE_M", 0.40),
        stale_caution_age_s=_env_float(prefix + "STALE_CAUTION_AGE_S", 0.10),
        stale_stop_age_s=_env_float(prefix + "STALE_STOP_AGE_S", 0.30),
    )


MOTION_SAFETY = load_motion_safety_config()
