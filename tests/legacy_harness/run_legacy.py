"""CLI: drive each scenario YAML against the legacy plugins and emit
``expected.json`` next to the YAML.

Two modes:

  ``--record``  Overwrites ``expected.json`` with the output of the run.
                Use this once when first capturing the fixtures (or when
                the legacy code is intentionally bumped).
  ``--check``   Re-runs each scenario and compares against the existing
                ``expected.json`` within ``tolerances``. Exits non-zero
                if any scenario drifts. CI uses this mode.

The actual recording is implemented in milestone M0's follow-up commit
once ``c_api.cpp`` has filled in its TODOs. For now this script only
verifies that the YAML parses and the harness library loads.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .legacy_runner import LegacyHarness


@dataclass(frozen=True)
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


def record_one(harness: LegacyHarness, scenario: dict[str, Any]) -> dict[str, Any]:
    """Drive ``scenario`` against the legacy plugins and return expected.json.

    NOT YET IMPLEMENTED. Filled in once c_api.cpp's TODO blocks are done.
    """
    raise NotImplementedError(
        "record_one is pending the c_api.cpp implementation: "
        "legacy_planner_plan / legacy_controller_compute are stubs."
    )


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
