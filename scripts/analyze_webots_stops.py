"""Correlate Webots animation standstills with scheduler diagnostic events."""

from __future__ import annotations

import argparse
import json
import math
import re
from bisect import bisect_left
from pathlib import Path
from typing import Any


ROBOT_RE = re.compile(r"<Robot\s+id=['\"]n(\d+)['\"][^>]*\bname=['\"]robot_(\d+)['\"]")
ACTIVE_STATES = {"en_route_pickup", "en_route_delivery", "returning_home", "charging"}
INTERVAL_EVENT_KEYS = ("planned_wait_events", "motion_continuity_events")
POINT_EVENT_KEYS = (
    "unplanned_stop_events", "progress_lease_events", "replan_events",
    "replan_request_events", "escape_events", "yield_resume_events",
    "route_dispatch_events", "terminal_handoff_events",
)


def load_robot_nodes(x3d_path: Path) -> dict[int, int]:
    text = x3d_path.read_text(encoding="utf-8", errors="replace")
    return {int(node): int(robot) for node, robot in ROBOT_RE.findall(text)}


def _translation(update: dict[str, Any]) -> tuple[float, float] | None:
    raw = update.get("translation")
    if not raw:
        return None
    values = [float(value) for value in str(raw).split()]
    return (values[0], values[1]) if len(values) >= 2 else None


def _yaw(update: dict[str, Any]) -> float | None:
    """Return planar yaw from a Webots axis-angle rotation."""
    raw = update.get("rotation")
    if not raw:
        return None
    values = [float(value) for value in str(raw).split()]
    if len(values) < 4:
        return None
    x, y, z, angle = values[:4]
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-12:
        return 0.0
    x, y, z = x / norm, y / norm, z / norm
    half = angle / 2.0
    qx, qy, qz = x * math.sin(half), y * math.sin(half), z * math.sin(half)
    qw = math.cos(half)
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _angle_distance(left: float, right: float) -> float:
    return abs((left - right + math.pi) % (2.0 * math.pi) - math.pi)


def detect_stops(
    animation: dict[str, Any], node_map: dict[int, int], min_seconds: float,
    epsilon_m: float, rotation_epsilon_rad: float = 0.10,
    pose_epsilon_m: float = 0.005,
) -> list[dict[str, Any]]:
    active: dict[int, dict[str, Any]] = {}
    stops: list[dict[str, Any]] = []
    last_time = 0.0
    for frame in animation.get("frames", []):
        now = float(frame.get("time", 0.0)) / 1000.0
        last_time = max(last_time, now)
        for update in frame.get("updates", []):
            robot_id = node_map.get(int(update.get("id", -1)))
            position = _translation(update)
            yaw = _yaw(update)
            if robot_id is None:
                continue
            run = active.get(robot_id)
            if run is None:
                if position is None:
                    continue
                active[robot_id] = {
                    "start": now, "anchor": position, "last": position,
                    "last_time": now, "max_translation_change": 0.0,
                }
                if yaw is not None:
                    active[robot_id]["anchor_yaw"] = yaw
                    active[robot_id]["last_yaw"] = yaw
                active[robot_id]["max_yaw_change"] = 0.0
                continue
            if yaw is not None:
                run["last_yaw"] = yaw
                anchor_yaw = run.get("anchor_yaw")
                if anchor_yaw is None:
                    run["anchor_yaw"] = yaw
                else:
                    run["max_yaw_change"] = max(
                        run.get("max_yaw_change", 0.0),
                        _angle_distance(yaw, anchor_yaw),
                    )
            if position is None:
                continue
            translation_change = math.dist(position, run["anchor"])
            if translation_change > epsilon_m:
                if now - run["start"] >= min_seconds:
                    stops.append(_stop(
                        robot_id, run, float(run.get("last_time", now)),
                        rotation_epsilon_rad, pose_epsilon_m))
                active[robot_id] = {
                    "start": now, "anchor": position, "last": position,
                    "last_time": now, "max_yaw_change": 0.0,
                    "max_translation_change": 0.0,
                }
                if yaw is not None:
                    active[robot_id]["anchor_yaw"] = yaw
                    active[robot_id]["last_yaw"] = yaw
            else:
                run["last"] = position
                run["last_time"] = now
                run["max_translation_change"] = max(
                    run.get("max_translation_change", 0.0),
                    translation_change)
    for robot_id, run in active.items():
        if last_time - run["start"] >= min_seconds:
            stops.append(_stop(
                robot_id, run, last_time, rotation_epsilon_rad,
                pose_epsilon_m))
    return sorted(stops, key=lambda item: (item["started_at"], item["robot_id"]))


def _stop(
        robot_id: int, run: dict[str, Any], ended_at: float,
        rotation_epsilon_rad: float, pose_epsilon_m: float) -> dict[str, Any]:
    yaw_change = float(run.get("max_yaw_change", 0.0))
    translation_change = float(run.get("max_translation_change", 0.0))
    return {
        "robot_id": robot_id,
        "started_at": round(run["start"], 3),
        "ended_at": round(ended_at, 3),
        "duration": round(ended_at - run["start"], 3),
        "position": [round(value, 4) for value in run["last"]],
        "max_yaw_change_rad": round(yaw_change, 4),
        "max_translation_change_m": round(translation_change, 5),
        "rotated_in_place": yaw_change > rotation_epsilon_rad,
        "pose_stationary": (
            translation_change <= pose_epsilon_m and
            yaw_change <= rotation_epsilon_rad),
    }


def _nearest_state(samples: list[dict[str, Any]], when: float, robot_id: int) -> dict[str, Any]:
    if not samples:
        return {}
    times = [float(sample.get("sim_time", 0.0)) for sample in samples]
    index = bisect_left(times, when)
    choices = [i for i in (index - 1, index) if 0 <= i < len(samples)]
    sample = samples[min(choices, key=lambda i: abs(times[i] - when))]
    state = dict(sample.get("robot_task_states", {}).get(str(robot_id), {}))
    state["sample_time"] = sample.get("sim_time")
    return state


def _robot_matches(event: dict[str, Any], robot_id: int) -> bool:
    direct = event.get("robot_id")
    involved = event.get("robots_involved", [])
    return direct == robot_id or str(direct) == str(robot_id) or robot_id in involved


def correlate(stops: list[dict[str, Any]], experiment: dict[str, Any]) -> list[dict[str, Any]]:
    samples = experiment.get("time_series", [])
    for stop in stops:
        robot_id = stop["robot_id"]
        start, end = stop["started_at"], stop["ended_at"]
        state = _nearest_state(samples, (start + end) / 2.0, robot_id)
        evidence: list[dict[str, Any]] = []
        for key in INTERVAL_EVENT_KEYS:
            for event in experiment.get(key, []):
                if not _robot_matches(event, robot_id):
                    continue
                event_start = float(event.get("started_at", event.get("sim_time", 0.0)))
                event_end_raw = event.get("ended_at")
                if event_end_raw is None:
                    event_end_raw = event.get("last_seen_at")
                event_end = float(event_start if event_end_raw is None else event_end_raw)
                if event_end >= start - 0.5 and event_start <= end + 0.5:
                    evidence.append({**event, "_event_stream": key})
        for key in POINT_EVENT_KEYS:
            for event in experiment.get(key, []):
                if _robot_matches(event, robot_id) and start - 0.5 <= float(event.get("sim_time", -1)) <= end + 0.5:
                    evidence.append({**event, "_event_stream": key})
        stop["task_state"] = state
        stop["motion_expected"] = state.get("state") in ACTIVE_STATES
        stop["evidence"] = evidence
        stop["classification"] = classify(state, evidence, stop)
    return stops


def classify(
        state: dict[str, Any], evidence: list[dict[str, Any]],
        stop: dict[str, Any] | None = None) -> str:
    if state.get("planned_wait"):
        return f"planned_wait:{state.get('wait_reason') or 'unknown'}"
    sources = {item["_event_stream"] for item in evidence}
    if "unplanned_stop_events" in sources:
        return "unplanned_stop"
    stages = {item.get("stage") for item in evidence if item["_event_stream"] == "progress_lease_events"}
    if stages:
        return "progress_recovery:" + ",".join(sorted(str(stage) for stage in stages))
    if stop and stop.get("pose_stationary"):
        command_state = state.get("controller_motion_state")
        stop_reason = str(state.get("controller_stop_reason") or "")
        if command_state == "moving":
            return "commanded_without_pose_progress"
        if command_state == "zero" and stop_reason == "emergency":
            return "controller_emergency_stop"
        if command_state == "zero" and stop_reason.startswith("advance_hold:"):
            return "controller_advance_hold:" + stop_reason.split(":", 1)[1]
        if command_state == "zero" and stop_reason == "local_planner_zero_replan":
            return "controller_zero_replan"
    if state.get("state") == "idle":
        return "idle"
    if not state:
        return "no_scheduler_sample"
    return "unexplained_active_stop" if state.get("state") in ACTIVE_STATES else f"state:{state.get('state')}"


def concurrent_stop_summary(
        stops: list[dict[str, Any]], threshold: int = 3,
        abnormal_only: bool = False, pose_stationary_only: bool = False) -> dict[str, Any]:
    events = []
    for stop in stops:
        if not stop.get("motion_expected"):
            continue
        classification = str(stop.get("classification", ""))
        if abnormal_only and not (
                classification in {"unexplained_active_stop", "unplanned_stop",
                                   "commanded_without_pose_progress",
                                   "controller_emergency_stop",
                                   "controller_zero_replan"}):
            continue
        if pose_stationary_only and not stop.get("pose_stationary", True):
            continue
        events.append((float(stop["started_at"]), 1, int(stop["robot_id"])))
        events.append((float(stop["ended_at"]), -1, int(stop["robot_id"])))
    events.sort(key=lambda item: (item[0], item[1]))  # End before start.
    active: set[int] = set()
    previous = None
    maximum = 0
    intervals = []
    for when, delta, robot_id in events:
        if previous is not None and when > previous and len(active) >= threshold:
            intervals.append({
                "started_at": round(previous, 3), "ended_at": round(when, 3),
                "duration": round(when - previous, 3),
                "robot_ids": sorted(active),
            })
        if delta < 0:
            active.discard(robot_id)
        else:
            active.add(robot_id)
            maximum = max(maximum, len(active))
        previous = when
    return {
        "max_concurrent_motion_expected_stops": maximum,
        "mass_stop_threshold": threshold,
        "mass_stop_seconds": round(sum(item["duration"] for item in intervals), 3),
        "mass_stop_intervals": intervals,
    }


def markdown_report(report: dict[str, Any]) -> str:
    summary = report["summary"]
    lines = [
        "# Webots 停车关联诊断", "",
        f"- 检测停车段：{summary['stop_intervals']}",
        f"- 有任务停车段：{summary['active_stop_intervals']}",
        f"- 未解释的有任务停车段：{summary['unexplained_active_intervals']}", "",
        "|机器人|开始(s)|结束(s)|时长(s)|任务状态|分类|证据数|", "|---:|---:|---:|---:|---|---|---:|",
    ]
    for stop in report["stops"]:
        state = stop["task_state"].get("state", "-")
        lines.append(
            f"|{stop['robot_id']}|{stop['started_at']:.3f}|{stop['ended_at']:.3f}|"
            f"{stop['duration']:.3f}|{state}|{stop['classification']}|{len(stop['evidence'])}|"
        )
    return "\n".join(lines) + "\n"


def analyze(
        animation_path: Path, experiment_path: Path, x3d_path: Path,
        min_seconds: float, epsilon_m: float,
        rotation_epsilon_rad: float = 0.10,
        pose_epsilon_m: float = 0.005) -> dict[str, Any]:
    animation = json.loads(animation_path.read_text(encoding="utf-8"))
    experiment = json.loads(experiment_path.read_text(encoding="utf-8"))
    stops = correlate(detect_stops(
        animation, load_robot_nodes(x3d_path), min_seconds, epsilon_m,
        rotation_epsilon_rad, pose_epsilon_m), experiment)
    concurrency = concurrent_stop_summary(stops)
    abnormal_concurrency = concurrent_stop_summary(stops, abnormal_only=True)
    stationary_abnormal_concurrency = concurrent_stop_summary(
        stops, abnormal_only=True, pose_stationary_only=True)
    return {
        "inputs": {"animation": str(animation_path), "experiment": str(experiment_path), "x3d": str(x3d_path)},
        "settings": {
            "min_stop_seconds": min_seconds,
            "movement_epsilon_m": epsilon_m,
            "rotation_epsilon_rad": rotation_epsilon_rad,
            "pose_epsilon_m": pose_epsilon_m,
        },
        "summary": {
            "stop_intervals": len(stops),
            "active_stop_intervals": sum(bool(item["motion_expected"]) for item in stops),
            "unexplained_active_intervals": sum(item["classification"] == "unexplained_active_stop" for item in stops),
            **{key: value for key, value in concurrency.items()
               if key != "mass_stop_intervals"},
            "max_concurrent_abnormal_stops": abnormal_concurrency[
                "max_concurrent_motion_expected_stops"],
            "abnormal_mass_stop_seconds": abnormal_concurrency[
                "mass_stop_seconds"],
            "rotating_in_place_intervals": sum(
                bool(item["rotated_in_place"]) for item in stops),
            "max_concurrent_pose_stationary_abnormal_stops":
                stationary_abnormal_concurrency[
                    "max_concurrent_motion_expected_stops"],
            "pose_stationary_abnormal_mass_stop_seconds":
                stationary_abnormal_concurrency["mass_stop_seconds"],
        },
        "mass_stop_intervals": concurrency["mass_stop_intervals"],
        "abnormal_mass_stop_intervals": abnormal_concurrency[
            "mass_stop_intervals"],
        "pose_stationary_abnormal_mass_stop_intervals":
            stationary_abnormal_concurrency["mass_stop_intervals"],
        "stops": stops,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("animation", type=Path)
    parser.add_argument("experiment", type=Path)
    parser.add_argument("--x3d", type=Path)
    parser.add_argument("--min-stop-seconds", type=float, default=2.0)
    parser.add_argument("--movement-epsilon-m", type=float, default=0.03)
    parser.add_argument("--rotation-epsilon-rad", type=float, default=0.10)
    parser.add_argument("--pose-epsilon-m", type=float, default=0.005)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    x3d = args.x3d or args.animation.with_suffix(".x3d")
    output = args.output or args.experiment.with_name(args.experiment.stem + "_stop_analysis.json")
    report = analyze(
        args.animation, args.experiment, x3d, args.min_stop_seconds,
        args.movement_epsilon_m, args.rotation_epsilon_rad,
        args.pose_epsilon_m)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    output.with_suffix(".md").write_text(markdown_report(report), encoding="utf-8")
    print(json.dumps(report["summary"], ensure_ascii=False))


if __name__ == "__main__":
    main()
