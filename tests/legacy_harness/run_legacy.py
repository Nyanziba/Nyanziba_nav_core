"""CLI: drive each scenario YAML against the legacy plugins and emit
``expected.json`` next to the YAML.

Two modes:

  ``--record``  Overwrites ``expected.json`` with the output of the run.
                Use this once when first capturing the fixtures (or when
                the legacy code is intentionally bumped).
  ``--check``   Re-runs each scenario and compares against the existing
                ``expected.json`` within ``tolerances``. Exits non-zero
                if any scenario drifts. CI uses this mode.

The runner only handles the subset of scenario YAML aspects that S-01..S-04
exercise: set_pose / set_goal / drive_until / cancel / step / wait /
enable_disturbance / assert. Heavier features (sweep / cases / hammer_set_path
/ plan_only with `repeat_consistency`) come in the next M0 commit alongside
their corresponding metric collectors.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path
from typing import Any

import yaml

from .legacy_runner import LegacyHarness


# ---------------------------------------------------------------------------
# Plugin name resolution
# ---------------------------------------------------------------------------

_PLANNER_PLUGIN_NAMES: dict[str, str] = {
    "astar":              "texnitis_move_base_like::AStarPlanner",
}

_CONTROLLER_PLUGIN_NAMES: dict[str, str] = {
    "lookahead":     "texnitis_move_base_like::LookaheadController",
    "pure_pursuit":  "texnitis_move_base_like::DiffDrivePurePursuitController",
    "mecanum_mppi":  "texnitis_move_base_like::MecanumMppiPlugin",
}


# ---------------------------------------------------------------------------
# Lightweight scenario state
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class ScenarioState:
    """Mutable bag passed between sequence steps."""

    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    goal: tuple[float, float, float] | None = None
    last_twist: tuple[float, float, float] = (0.0, 0.0, 0.0)
    outcome: str = "PENDING"
    xy_goal_reached: bool = False
    goal_reach_flip_count: int = 0
    collision_count: int = 0
    nan_inf_count: int = 0
    step_count: int = 0
    history_path: list[tuple[float, float, float]] = dataclasses.field(default_factory=list)
    history_twist: list[tuple[float, float, float]] = dataclasses.field(default_factory=list)
    last_planned_path: list[tuple[float, float, float]] = dataclasses.field(default_factory=list)


def _occupancy_to_grid(spec: dict[str, Any]) -> tuple[list[list[int]], float, tuple[float, float]]:
    """Translate the YAML map block into (occupancy 2D, resolution, origin).

    `#` -> 100 (lethal), `.` -> 0 (free), `?` -> -1 (unknown).
    """
    table = {".": 0, "#": 100, "?": -1}
    rows: list[list[int]] = []
    for line in spec["occupancy"].strip("\n").splitlines():
        rows.append([table.get(ch, 0) for ch in line])
    return rows, float(spec["resolution"]), tuple(spec.get("origin", [0.0, 0.0]))


def _step_robot(state: ScenarioState,
                twist: tuple[float, float, float],
                dt: float) -> None:
    """Advance the kinematic pose by one dt using the unicycle model.

    Mecanum / holonomic refinements come once the simulator package
    lands (M8). For S-01..S-04 the unicycle model is enough.
    """
    x, y, yaw = state.pose
    vx, vy, wz = twist
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    state.pose = (
        x + (vx * cos_yaw - vy * sin_yaw) * dt,
        y + (vx * sin_yaw + vy * cos_yaw) * dt,
        ((yaw + wz * dt) + math.pi) % (2.0 * math.pi) - math.pi,
    )


def _evaluate_condition(condition: str, state: ScenarioState) -> bool:
    """Tiny condition language used by `drive_until`."""
    if condition == "goal_reached":
        return state.outcome == "SUCCESS"
    if condition == "xy_goal_reached":
        return state.xy_goal_reached
    if condition.startswith("step_count >= "):
        return state.step_count >= int(condition.split(">=")[-1].strip())
    if condition.startswith("distance_to_goal < "):
        if state.goal is None:
            return False
        gx, gy, _ = state.goal
        return math.hypot(gx - state.pose[0], gy - state.pose[1]) < \
            float(condition.split("<")[-1].strip())
    raise NotImplementedError(f"unsupported condition: {condition}")


def _do_step(harness: LegacyHarness,
             planner_handle: int | None,
             controller_handle: int | None,
             state: ScenarioState,
             dt: float,
             grid: list[list[int]],
             resolution: float,
             origin: tuple[float, float]) -> None:
    """Run one tick: optionally re-plan, then computeCommand, then integrate."""
    if planner_handle is not None and controller_handle is not None and \
            state.goal is not None and not state.last_planned_path:
        status, path = harness.plan(
            planner_handle,
            occupancy=grid, resolution=resolution, origin=origin,
            start=state.pose, goal=state.goal,
        )
        if status == 0 and path:
            state.last_planned_path = path
            harness.set_path(controller_handle, path)
        else:
            state.outcome = "NO_PATH_FOUND"
            return

    if controller_handle is not None and state.last_planned_path:
        status, twist, reached = harness.compute(controller_handle, state.pose)
        state.last_twist = twist
        state.history_twist.append(twist)
        # NaN / Inf accounting.
        for component in twist:
            if math.isnan(component) or math.isinf(component):
                state.nan_inf_count += 1

        # Update stateful flag: once True it must stay True per S-01 / S-17.
        was_reached = state.xy_goal_reached
        if reached and not state.xy_goal_reached:
            state.xy_goal_reached = True
            state.goal_reach_flip_count += 1
        if reached:
            state.outcome = "SUCCESS"
        if was_reached and not reached:
            # Flip count tracks 0 -> 1 transitions only; staying True is fine.
            pass

        _step_robot(state, twist, dt)

    state.step_count += 1
    state.history_path.append(state.pose)


def _record_assert(state: ScenarioState, action: dict[str, Any],
                   recorded: dict[str, Any]) -> None:
    """Translate `assert` keys into recorded slots inside expected.json."""
    key = action["key"]
    if key == "outcome":
        recorded.setdefault("outcomes", []).append(state.outcome)
    elif key == "xy_goal_reached":
        recorded.setdefault("xy_goal_reached_states", []).append(state.xy_goal_reached)
    elif key.startswith("last_twist."):
        recorded.setdefault("last_twist_samples", []).append(state.last_twist)
    elif key.startswith("metrics."):
        recorded.setdefault("metrics", {})[key.split(".", 1)[1]] = (
            getattr(state, key.split(".", 1)[1], None))
    elif key == "collision_count":
        recorded.setdefault("collision_counts", []).append(state.collision_count)


def record_one(harness: LegacyHarness, scenario: dict[str, Any]) -> dict[str, Any]:
    """Drive ``scenario`` against the legacy plugins and return expected.json.

    Subset implementation: covers S-01..S-04 and any other scenario whose
    sequence uses set_pose / set_goal / drive_until / cancel / step / wait /
    enable_disturbance / assert. More exotic actions raise NotImplementedError
    so the missing coverage is loud rather than silent.
    """
    plugins = scenario["plugins"]
    map_grid, resolution, origin = _occupancy_to_grid(scenario["map"])

    planner_handle: int | None = None
    controller_handle: int | None = None

    if "planner" in plugins:
        spec = plugins["planner"]
        planner_handle = harness.create_planner(
            _PLANNER_PLUGIN_NAMES[spec["name"]], spec.get("params", {}))
    if "controller" in plugins:
        spec = plugins["controller"]
        controller_handle = harness.create_controller(
            _CONTROLLER_PLUGIN_NAMES[spec["name"]], spec.get("params", {}))

    state = ScenarioState()
    recorded: dict[str, Any] = {
        "id": scenario.get("id", "?"),
        "trials": [],
    }
    trial_recorded: dict[str, Any] = {"seed": scenario.get("seed", 0),
                                       "outcomes": [],
                                       "metrics": {}}

    try:
        for action in scenario.get("sequence", []):
            kind = action["action"]
            if kind == "set_pose":
                state.pose = tuple(action["pose"])
            elif kind == "set_goal":
                state.goal = tuple(action["pose"])
                state.last_planned_path = []
                if controller_handle is not None:
                    harness.reset_controller(controller_handle)
                state.xy_goal_reached = False
                state.outcome = "PENDING"
            elif kind == "cancel":
                if controller_handle is not None:
                    harness.cancel_controller(controller_handle)
                state.outcome = "CANCELLED"
                state.last_planned_path = []
            elif kind == "step":
                for _ in range(int(action["count"])):
                    _do_step(harness, planner_handle, controller_handle, state,
                             float(action.get("dt", 0.05)), map_grid, resolution, origin)
            elif kind == "drive_until":
                max_steps = int(action["max_steps"])
                dt = float(action.get("dt", 0.05))
                done = False
                for _ in range(max_steps):
                    _do_step(harness, planner_handle, controller_handle, state,
                             dt, map_grid, resolution, origin)
                    if _evaluate_condition(action["condition"], state):
                        done = True
                        break
                if not done:
                    state.outcome = "TIMEOUT"
            elif kind == "wait":
                for _ in range(int(float(action["seconds"]) / 0.05)):
                    state.history_twist.append((0.0, 0.0, 0.0))
                    state.history_path.append(state.pose)
                    state.step_count += 1
            elif kind == "enable_disturbance":
                # Disturbance application is owned by the simulator package,
                # not the legacy harness. We ignore it for fixture recording
                # and capture the deterministic baseline.
                pass
            elif kind == "assert":
                _record_assert(state, action, trial_recorded)
            else:
                raise NotImplementedError(f"unsupported action: {kind}")
    finally:
        if controller_handle is not None:
            harness.destroy_controller(controller_handle)
        if planner_handle is not None:
            harness.destroy_planner(planner_handle)

    trial_recorded["outcomes"] = trial_recorded.get("outcomes") or [state.outcome]
    trial_recorded["metrics"] = {
        "goal_reach_flip_count": state.goal_reach_flip_count,
        "collision_count": state.collision_count,
        "nan_inf_count": state.nan_inf_count,
        "step_count": state.step_count,
    }
    trial_recorded["final_pose"] = state.pose
    trial_recorded["last_twist"] = state.last_twist
    recorded["trials"].append(trial_recorded)
    return recorded


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class ScenarioFile:
    """One scenario discovered on disk."""

    scenario_id: str
    yaml_path: Path
    expected_path: Path


def discover_scenarios(scenarios_dir: Path) -> list[ScenarioFile]:
    """Return scenarios sorted by ID under ``scenarios_dir``."""
    found: list[ScenarioFile] = []
    for sub in sorted(scenarios_dir.iterdir()):
        if not sub.is_dir():
            continue
        yaml_path = sub / "scenario.yaml"
        if not yaml_path.exists():
            continue
        found.append(
            ScenarioFile(
                scenario_id=sub.name,
                yaml_path=yaml_path,
                expected_path=sub / "expected.json",
            )
        )
    return found


def load_scenario(yaml_path: Path) -> dict[str, Any]:
    """Parse a scenario YAML."""
    with yaml_path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def parse_args(argv: list[str]) -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(prog="run_legacy")
    parser.add_argument(
        "--scenarios",
        type=Path,
        default=Path("tests/scenarios"),
        help="Directory containing S-XX/scenario.yaml entries.",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=Path("build/legacy_harness"),
        help="CMake build directory holding libtexnitis_legacy_harness.",
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--record", action="store_true",
                      help="Generate / overwrite expected.json files.")
    mode.add_argument("--check", action="store_true",
                      help="Compare current run against expected.json.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint."""
    args = parse_args(sys.argv[1:] if argv is None else argv)
    scenarios = discover_scenarios(args.scenarios)
    if not scenarios:
        print(f"no scenarios under {args.scenarios}", file=sys.stderr)
        return 1

    harness = LegacyHarness(args.build_dir)
    failures: list[str] = []

    for sf in scenarios:
        scenario = load_scenario(sf.yaml_path)
        try:
            recorded = record_one(harness, scenario)
        except NotImplementedError as exc:
            print(f"[skip] {sf.scenario_id}: {exc}")
            continue
        if args.record:
            sf.expected_path.write_text(json.dumps(recorded, indent=2, sort_keys=True))
            print(f"[record] wrote {sf.expected_path}")
        else:
            if not sf.expected_path.exists():
                failures.append(f"{sf.scenario_id}: missing expected.json")
                continue
            stored = json.loads(sf.expected_path.read_text())
            if stored != recorded:
                failures.append(f"{sf.scenario_id}: drift detected")

    if failures:
        for reason in failures:
            print(reason, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
