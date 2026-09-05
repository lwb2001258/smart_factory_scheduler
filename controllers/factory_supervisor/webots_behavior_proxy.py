"""Versioned, scheduler-level proxy for Webots execution behaviour.

The proxy models only effects visible to the dispatcher. It is not a rigid-body
replacement. Parameters are anchored to robot_controller constants and may be
calibrated from measured route traces without changing observation semantics.
"""

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Sequence, Tuple

from config import BATTERY_CHARGE_RATE, BATTERY_DRAIN_RATE, TIMESTEP


Point = Tuple[float, float]


@dataclass(frozen=True)
class BehaviorProxyConfig:
    version: str = "webots-dispatch-proxy-v1"
    timestep_seconds: float = TIMESTEP / 1000.0
    linear_speed: float = 0.22
    angular_speed: float = 2.84
    waypoint_tolerance: float = 0.35
    # Turning and translation overlap in the real controller. 1.0 would add
    # full in-place turn time; 0.0 would ignore heading changes entirely.
    turn_time_weight: float = 0.35
    battery_drain_rate: float = BATTERY_DRAIN_RATE
    battery_charge_rate: float = BATTERY_CHARGE_RATE


@dataclass(frozen=True)
class MotionEstimate:
    path_distance: float
    executable_distance: float
    translation_seconds: float
    turning_seconds: float
    total_seconds: float
    battery_used: float
    turns: int


def _wrapped_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


class WebotsBehaviorProxy:
    def __init__(self, config: BehaviorProxyConfig = None):
        self.config = config or BehaviorProxyConfig()

    def estimate_path(self, start: Point, path: Sequence[Point],
                      initial_heading: float = 0.0) -> MotionEstimate:
        points = [tuple(start)] + [tuple(point) for point in path]
        distance = executable = turning = 0.0
        heading = float(initial_heading)
        turns = 0
        for a, b in zip(points, points[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            segment = math.hypot(dx, dy)
            if segment <= 1e-12:
                continue
            target_heading = math.atan2(dy, dx)
            delta = abs(_wrapped_angle(target_heading - heading))
            if delta > 1e-6:
                turns += 1
                turning += (delta / self.config.angular_speed
                            * self.config.turn_time_weight)
            distance += segment
            # The controller advances a waypoint once inside GOAL_THRESHOLD.
            executable += max(0.0, segment - self.config.waypoint_tolerance)
            heading = target_heading
        translation = executable / self.config.linear_speed
        raw_total = translation + turning
        ticks = math.ceil(raw_total / self.config.timestep_seconds)
        total = ticks * self.config.timestep_seconds
        return MotionEstimate(
            path_distance=distance,
            executable_distance=executable,
            translation_seconds=translation,
            turning_seconds=turning,
            total_seconds=total,
            battery_used=total * self.config.battery_drain_rate,
            turns=turns,
        )

    def charging_seconds(self, current_battery: float,
                         target_battery: float) -> float:
        delta = max(0.0, float(target_battery) - float(current_battery))
        raw = delta / self.config.battery_charge_rate
        return (math.ceil(raw / self.config.timestep_seconds)
                * self.config.timestep_seconds)

    def metadata(self) -> dict:
        return asdict(self.config)

