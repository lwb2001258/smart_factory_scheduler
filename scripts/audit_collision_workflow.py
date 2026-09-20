"""Capture and audit reproducible collision-workflow Webots evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError


ROOT = Path(__file__).resolve().parents[1]
KEY_FILES = (
    "controllers/factory_supervisor/config.py",
    "controllers/factory_supervisor/grid_planner.py",
    "controllers/factory_supervisor/joint_grid_planner.py",
    "controllers/factory_supervisor/motion_coordinator.py",
    "controllers/factory_supervisor/factory_supervisor.py",
    "controllers/factory_supervisor/collision_safety.py",
    "controllers/robot_controller/robot_controller.py",
    "worlds/smart_factory.wbt",
)
FLAG_NAMES = (
    "SMART_FACTORY_ENABLE_JOINT_RUNTIME",
    "SMART_FACTORY_ENABLE_RHCR",
    "SMART_FACTORY_ENABLE_JOINT_SPEED",
    "SMART_FACTORY_ENABLE_PRIORITY_YIELD_RESUME",
    "SMART_FACTORY_ENABLE_LEGACY_INTERLOCK",
    "SMART_FACTORY_ENABLE_NONPHYSICAL_RECOVERY",
    "SMART_FACTORY_JOINT_WATCHDOG_INTERVAL",
    "SMART_FACTORY_RESERVATION_REFRESH_INTERVAL",
    "SMART_FACTORY_SIM_DURATION",
    "SMART_FACTORY_AUTO_STOP",
)
CHECKPOINT_SCHEDULERS = {
    "PPO_RL", "SARSA", "DQN", "SARSA_LAMBDA", "RAINBOW_DQN",
    "A2C", "DISCRETE_SAC", "QR_DQN",
}


class AuditError(ValueError):
    """Evidence is missing, stale, inconsistent, or unsafe."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git(*args: str) -> str:
    completed = subprocess.run(
        ["git", *args], cwd=ROOT, text=True, capture_output=True,
        encoding="utf-8", errors="replace", check=False)
    if completed.returncode:
        return f"UNAVAILABLE: {completed.stderr.strip()}"
    return completed.stdout.strip()


def _webots_version(executable: Path | None) -> str | None:
    if executable is None:
        return None
    completed = subprocess.run(
        [str(executable), "--version"], text=True, capture_output=True,
        encoding="utf-8", errors="replace", check=False)
    if completed.returncode:
        raise AuditError(f"Webots version failed: {completed.stderr.strip()}")
    return (completed.stdout or completed.stderr).strip()


def _resolved_config() -> dict[str, Any]:
    controller_dir = ROOT / "controllers" / "factory_supervisor"
    sys.path.insert(0, str(controller_dir))
    try:
        import config
    finally:
        sys.path.pop(0)
    names = (
        "ENABLE_JOINT_RUNTIME", "ENABLE_RUNTIME_RHCR",
        "ENABLE_PROACTIVE_JOINT_SPEED", "ENABLE_PRIORITY_YIELD_RESUME",
        "ENABLE_LEGACY_INTERLOCK_RECOVERY", "ENABLE_NONPHYSICAL_RECOVERY",
        "JOINT_WATCHDOG_INTERVAL", "RESERVATION_REFRESH_INTERVAL",
        "SIM_DURATION", "AUTO_STOP_SIMULATION", "TIMESTEP",
    )
    resolved = {name: getattr(config, name) for name in names}
    from motion_safety import MOTION_SAFETY
    resolved["MOTION_SAFETY"] = MOTION_SAFETY.resolved()
    return resolved


def capture_snapshot(webots: Path | None = None) -> dict[str, Any]:
    files = {}
    for relative in KEY_FILES:
        path = ROOT / relative
        if not path.is_file():
            raise AuditError(f"required file missing: {relative}")
        files[relative] = sha256(path)
    status = _git("status", "--porcelain")
    return {
        "schema": "collision-workflow-snapshot-v1",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "root": str(ROOT),
        "git_commit": _git("rev-parse", "HEAD"),
        "git_dirty": bool(status),
        "git_status": status.splitlines(),
        "python": sys.version,
        "platform": platform.platform(),
        "webots": _webots_version(webots),
        "environment": {name: os.environ.get(name) for name in FLAG_NAMES},
        "resolved_config": _resolved_config(),
        "files": files,
        "world_hash": files["worlds/smart_factory.wbt"],
    }


def _assert_finite(value: Any, path: str = "$") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise AuditError(f"non-finite number at {path}")
    if isinstance(value, dict):
        for key, item in value.items():
            _assert_finite(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _assert_finite(item, f"{path}[{index}]")


def _require(mapping: dict[str, Any], key: str, where: str) -> Any:
    if key not in mapping:
        raise AuditError(f"missing {where}.{key}")
    return mapping[key]


def _replan_category(source: str) -> str:
    normalized = source.lower()
    if "priority_yield" in normalized:
        return "traffic_rule"
    if "escape" in normalized or "reverse" in normalized:
        return "escape_triggered"
    if "stall" in normalized or "deadlock" in normalized:
        return "stall_recovery"
    if "emergency" in normalized:
        return "emergency"
    if "transaction" in normalized or "joint" in normalized:
        return "joint_runtime"
    if "business" in normalized or "task" in normalized:
        return "business_transition"
    return "other"


def _route_category(source: str) -> str:
    normalized = source.lower()
    if "priority_yield" in normalized:
        return "traffic_rule"
    if "escape" in normalized or "reverse" in normalized:
        return "escape_recovery"
    if "stall" in normalized or "deadlock" in normalized:
        return "stall_recovery"
    if "business" in normalized or "task" in normalized:
        return "business_transition"
    if "emergency" in normalized:
        return "emergency"
    if "joint_grid" in normalized or "joint" in normalized:
        return "rolling_joint"
    return "other"


def audit_result(data: dict[str, Any], *, scenario: str, scheduler: str,
                 seed: int, minimum_duration: float,
                 allow_unsafe_baseline: bool = False,
                 require_motion_safety: bool = False,
                 expected_motion_safety_fingerprint: str | None = None,
                 require_joint_timing: bool = False,
                 maximum_traversal_p99: float | None = None,
                 require_webots_performance: bool = False,
                 minimum_sim_to_wall_ratio: float | None = None,
                 maximum_scheduling_p99_ms: float | None = None
                 ) -> dict[str, Any]:
    _assert_finite(data)
    info = _require(data, "experiment_info", "result")
    metrics = _require(data, "summary_metrics", "result")
    provenance = _require(info, "provenance", "experiment_info")
    expected = {
        "scenario": scenario,
        "scheduler": scheduler,
        "seed": int(seed),
        "run_mode": "webots",
    }
    actual = {
        "scenario": _require(info, "scenario", "experiment_info"),
        "scheduler": _require(info, "scheduler", "experiment_info"),
        "seed": _require(provenance, "seed", "provenance"),
        "run_mode": _require(provenance, "run_mode", "provenance"),
    }
    if actual != expected:
        raise AuditError(f"result identity mismatch: {actual!r} != {expected!r}")
    duration = float(_require(info, "sim_duration", "experiment_info"))
    if duration + 1e-6 < minimum_duration:
        raise AuditError(
            f"short Webots result: {duration:.3f}s < {minimum_duration:.3f}s")
    required_provenance = ["manifest_fingerprint"]
    if scheduler in CHECKPOINT_SCHEDULERS:
        required_provenance.extend(("sha256", "contract_fingerprint"))
    for key in required_provenance:
        value = _require(provenance, key, "provenance")
        if not isinstance(value, str) or not value.strip():
            raise AuditError(f"invalid provenance.{key}")
    safety_fingerprint = provenance.get("motion_safety_fingerprint")
    safety_config = provenance.get("motion_safety_config")
    if require_motion_safety and (
            safety_fingerprint is None or safety_config is None):
        raise AuditError("motion safety provenance is required")
    if safety_fingerprint is not None or safety_config is not None:
        if not isinstance(safety_fingerprint, str) or len(safety_fingerprint) != 64:
            raise AuditError("invalid provenance.motion_safety_fingerprint")
        if not isinstance(safety_config, dict):
            raise AuditError("invalid provenance.motion_safety_config")
        if safety_config.get("fingerprint") != safety_fingerprint:
            raise AuditError("motion safety config/fingerprint mismatch")
    if (expected_motion_safety_fingerprint is not None and
            safety_fingerprint != expected_motion_safety_fingerprint):
        raise AuditError("unexpected motion safety fingerprint")

    replans = list(_require(data, "replan_events", "result"))
    escapes = list(_require(data, "escape_events", "result"))
    routes = list(_require(data, "route_dispatch_events", "result"))
    nonphysical_events = list(_require(
        data, "nonphysical_recovery_events", "result"))
    unauthorized_events = list(_require(
        data, "unauthorized_route_write_events", "result"))
    traversal_samples = data.get("joint_cell_traversal_seconds")
    window_gap_events = data.get("joint_window_gap_events")
    expired_movement_events = data.get("reservation_expired_movement_events")
    audited_replans = int(_require(metrics, "audited_replans", "summary_metrics"))
    physical_escapes = int(_require(metrics, "physical_escapes", "summary_metrics"))
    if audited_replans != len(replans):
        raise AuditError(
            f"replan count mismatch: metric={audited_replans}, events={len(replans)}")
    if physical_escapes != len(escapes):
        raise AuditError(
            f"escape count mismatch: metric={physical_escapes}, events={len(escapes)}")
    route_dispatches = int(_require(
        metrics, "route_dispatches", "summary_metrics"))
    if route_dispatches != len(routes):
        raise AuditError(
            f"route dispatch count mismatch: metric={route_dispatches}, "
            f"events={len(routes)}")

    violations = int(_require(
        metrics, "pair_distance_violations", "summary_metrics"))
    minimum_distance = float(_require(
        metrics, "min_pair_distance", "summary_metrics"))
    nonphysical = int(_require(
        metrics, "nonphysical_recoveries", "summary_metrics"))
    unauthorized = int(_require(
        metrics, "unauthorized_route_writes", "summary_metrics"))
    if nonphysical != len(nonphysical_events):
        raise AuditError(
            f"nonphysical count mismatch: metric={nonphysical}, "
            f"events={len(nonphysical_events)}")
    if unauthorized != len(unauthorized_events):
        raise AuditError(
            f"unauthorized write count mismatch: metric={unauthorized}, "
            f"events={len(unauthorized_events)}")
    timing = None
    if require_joint_timing:
        if not isinstance(traversal_samples, list) or not traversal_samples:
            raise AuditError("joint traversal samples are required")
        if not isinstance(window_gap_events, list):
            raise AuditError("joint window gap events are required")
        if not isinstance(expired_movement_events, list):
            raise AuditError("expired reservation movement events are required")
        sample_count = int(_require(
            metrics, "joint_cell_traversal_samples", "summary_metrics"))
        gaps = int(_require(metrics, "joint_window_gaps", "summary_metrics"))
        expired = int(_require(
            metrics, "reservation_expired_movements", "summary_metrics"))
        p99 = float(_require(
            metrics, "joint_cell_traversal_p99_seconds", "summary_metrics"))
        if sample_count != len(traversal_samples):
            raise AuditError("joint traversal sample count mismatch")
        if gaps != len(window_gap_events) or expired != len(expired_movement_events):
            raise AuditError("joint timing event count mismatch")
        if gaps or expired:
            raise AuditError(
                f"joint timing safety failure: gaps={gaps}, expired={expired}")
        if maximum_traversal_p99 is not None and p99 > maximum_traversal_p99:
            raise AuditError(
                f"joint traversal P99 {p99:.6f}s exceeds "
                f"{maximum_traversal_p99:.6f}s")
        timing = {"samples": sample_count, "p99_seconds": p99,
                  "window_gaps": gaps, "expired_movements": expired}
    performance = None
    if require_webots_performance:
        execution = _require(data, "execution_metrics", "result")
        ratio = float(_require(
            execution, "sim_to_wall_ratio", "execution_metrics"))
        wall_seconds = float(_require(
            execution, "webots_wall_seconds", "execution_metrics"))
        scheduling_p99 = float(_require(
            metrics, "scheduling_latency_p99_ms", "summary_metrics"))
        if minimum_sim_to_wall_ratio is not None and ratio < minimum_sim_to_wall_ratio:
            raise AuditError(
                f"Webots sim/wall ratio {ratio:.3f} below "
                f"{minimum_sim_to_wall_ratio:.3f}")
        if (maximum_scheduling_p99_ms is not None and
                scheduling_p99 > maximum_scheduling_p99_ms):
            raise AuditError(
                f"scheduling P99 {scheduling_p99:.3f}ms exceeds "
                f"{maximum_scheduling_p99_ms:.3f}ms")
        performance = {"webots_wall_seconds": wall_seconds,
                       "sim_to_wall_ratio": ratio,
                       "scheduling_p99_ms": scheduling_p99}
    unsafe_reasons = []
    if violations:
        unsafe_reasons.append(f"pair_distance_violations={violations}")
    if minimum_distance < 0.50:
        unsafe_reasons.append(f"min_pair_distance={minimum_distance:.6f}<0.50")
    if nonphysical:
        unsafe_reasons.append(f"nonphysical_recoveries={nonphysical}")
    if nonphysical:
        raise AuditError("unsafe Webots result: " + "; ".join(unsafe_reasons))
    if unsafe_reasons and not allow_unsafe_baseline:
        raise AuditError("unsafe Webots result: " + "; ".join(unsafe_reasons))

    sources = Counter(str(event.get("source", "missing")) for event in replans)
    categories = Counter(_replan_category(source) for source in sources.elements())
    route_sources = Counter(
        str(event.get("source", "missing")) for event in routes)
    route_categories = Counter(
        _route_category(source) for source in route_sources.elements())
    return {
        "schema": "collision-workflow-audit-v1",
        "status": "CAPTURED_UNSAFE_BASELINE" if unsafe_reasons else "PASS",
        "identity": actual,
        "motion_safety_fingerprint": safety_fingerprint,
        "duration": duration,
        "safety": {
            "min_pair_distance": minimum_distance,
            "pair_distance_violations": violations,
            "nonphysical_recoveries": nonphysical,
            "unsafe_reasons": unsafe_reasons,
        },
        "replans": {
            "total": audited_replans,
            "by_source": dict(sorted(sources.items())),
            "by_category": dict(sorted(categories.items())),
        },
        "route_dispatches": {
            "total": route_dispatches,
            "by_source": dict(sorted(route_sources.items())),
            "by_category": dict(sorted(route_categories.items())),
            "unauthorized": unauthorized,
        },
        "physical_escapes": physical_escapes,
        "joint_timing": timing,
        "webots_performance": performance,
    }


def _read_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as handle:
            value = json.load(handle, parse_constant=lambda token: (_ for _ in ()).throw(
                AuditError(f"invalid JSON constant: {token}")))
    except json.JSONDecodeError as exc:
        raise AuditError(f"invalid JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise AuditError("top-level JSON must be an object")
    return value


def assert_result_fresh(path: Path, data: dict[str, Any], not_before: datetime,
                        result_timezone: str = "Asia/Shanghai") -> None:
    modified = datetime.fromtimestamp(path.stat().st_mtime, timezone.utc)
    threshold = not_before.astimezone(timezone.utc)
    if modified < threshold:
        raise AuditError(
            f"stale result: mtime {modified.isoformat()} < "
            f"{threshold.isoformat()}")
    raw_timestamp = _require(
        _require(data, "experiment_info", "result"),
        "timestamp", "experiment_info")
    try:
        recorded = datetime.fromisoformat(str(raw_timestamp))
        if recorded.tzinfo is None:
            recorded = recorded.replace(tzinfo=ZoneInfo(result_timezone))
    except (ValueError, ZoneInfoNotFoundError) as exc:
        raise AuditError(f"invalid experiment timestamp/timezone: {exc}") from exc
    if recorded.astimezone(timezone.utc) < threshold:
        raise AuditError(
            f"stale result: experiment timestamp {recorded.isoformat()} < "
            f"{not_before.isoformat()}")


def write_evidence(path: Path, rendered: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8") as handle:
            handle.write(rendered + "\n")
    except FileExistsError as exc:
        raise AuditError(f"refusing to overwrite evidence: {path}") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    capture = subparsers.add_parser("capture")
    capture.add_argument("--output", type=Path, required=True)
    capture.add_argument("--webots", type=Path)
    audit = subparsers.add_parser("audit")
    audit.add_argument("result", type=Path)
    audit.add_argument("--scenario", required=True)
    audit.add_argument("--scheduler", required=True)
    audit.add_argument("--seed", type=int, required=True)
    audit.add_argument("--minimum-duration", type=float, required=True)
    audit.add_argument("--allow-unsafe-baseline", action="store_true")
    audit.add_argument("--require-motion-safety", action="store_true")
    audit.add_argument("--expected-motion-safety-fingerprint")
    audit.add_argument("--require-joint-timing", action="store_true")
    audit.add_argument("--maximum-traversal-p99", type=float)
    audit.add_argument("--require-webots-performance", action="store_true")
    audit.add_argument("--minimum-sim-to-wall-ratio", type=float)
    audit.add_argument("--maximum-scheduling-p99-ms", type=float)
    audit.add_argument(
        "--not-before",
        help="Reject a result whose filesystem mtime is before this ISO time")
    audit.add_argument("--result-timezone", default="Asia/Shanghai")
    audit.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    if args.command == "capture":
        output = capture_snapshot(args.webots)
        destination = args.output
    else:
        result_data = _read_json(args.result)
        if args.not_before:
            try:
                not_before = datetime.fromisoformat(
                    args.not_before.replace("Z", "+00:00"))
            except ValueError as exc:
                raise AuditError(f"invalid --not-before: {args.not_before}") from exc
            if not_before.tzinfo is None:
                raise AuditError("--not-before must include a timezone")
            assert_result_fresh(
                args.result, result_data, not_before, args.result_timezone)
        output = audit_result(
            result_data, scenario=args.scenario,
            scheduler=args.scheduler, seed=args.seed,
            minimum_duration=args.minimum_duration,
            allow_unsafe_baseline=args.allow_unsafe_baseline,
            require_motion_safety=args.require_motion_safety,
            expected_motion_safety_fingerprint=(
                args.expected_motion_safety_fingerprint),
            require_joint_timing=args.require_joint_timing,
            maximum_traversal_p99=args.maximum_traversal_p99,
            require_webots_performance=args.require_webots_performance,
            minimum_sim_to_wall_ratio=args.minimum_sim_to_wall_ratio,
            maximum_scheduling_p99_ms=args.maximum_scheduling_p99_ms)
        destination = args.output
    rendered = json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False)
    if destination:
        write_evidence(destination, rendered)
    else:
        print(rendered)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AuditError as exc:
        print(f"AUDIT FAILED: {exc}", file=sys.stderr)
        raise SystemExit(2)
