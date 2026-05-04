"""Smoke tests for the pybind11 bindings.

Skipped automatically when the native extension hasn't been built (e.g.
the harness CI job that installs only the pure Python sources).
"""

from __future__ import annotations

import importlib

import numpy as np
import pytest


@pytest.fixture(scope="module")
def core() -> object:
    """Return the bound module, skipping the suite when it's missing."""
    nc = importlib.import_module("texnitis_nav_core")
    if nc._core is None:
        pytest.skip("native _core extension not available in this build")
    return nc


@pytest.mark.binding
def test_pose_default(core) -> None:
    """A default Pose2D has all-zero fields."""
    p = core.Pose2D()
    assert p.x == 0.0 and p.y == 0.0 and p.yaw == 0.0


@pytest.mark.binding
def test_normalize_angle(core) -> None:
    """normalize_angle wraps into [-pi, pi]."""
    assert pytest.approx(core.normalize_angle(3.0 * np.pi), abs=1e-9) == np.pi


@pytest.mark.binding
def test_grid_map_view_from_numpy(core) -> None:
    """from_numpy constructs a borrowed view that stays alive while held."""
    grid = np.zeros((5, 5), dtype=np.int8)
    view = core.GridMapView.from_numpy(grid, resolution=0.1, origin_x=0.0, origin_y=0.0)
    assert view.width == 5 and view.height == 5
    assert view.in_bounds(2, 2) and not view.in_bounds(-1, 0)


@pytest.mark.binding
def test_astar_open_grid_succeeds(core) -> None:
    """A* on an empty 5x5 grid returns a path."""
    grid = np.zeros((5, 5), dtype=np.int8)
    view = core.GridMapView.from_numpy(grid, resolution=0.1)
    params = core.AStarParams()
    params.inflation_radius = 0.0
    planner = core.AStarPlanner(params)
    status, path = planner.plan_path(view, core.Pose2D(0.05, 0.05, 0.0),
                                     core.Pose2D(0.45, 0.45, 0.0))
    assert status == core.PlanResult.Success
    assert len(path) >= 2


@pytest.mark.binding
def test_lookahead_empty_path_returns_empty(core) -> None:
    """LookaheadController with no plan returns EmptyPath."""
    controller = core.LookaheadController()
    status, twist = controller.compute_command(core.Pose2D())
    assert status == core.ControllerResult.EmptyPath
    assert twist.vx == 0.0 and twist.wz == 0.0
