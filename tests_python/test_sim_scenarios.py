"""End-to-end sim scenarios.

Each test sets up an OccupancyMap, plans with the bound A* and drives
the bound Lookahead controller through ``run_episode``, then asserts
the metric thresholds defined for the corresponding S-XX scenario.
"""

from __future__ import annotations

import importlib

import pytest


@pytest.fixture(scope="module")
def core() -> object:
    """Skip the suite if the native bindings are missing."""
    nc = importlib.import_module("texnitis_nav_core")
    if nc._core is None:
        pytest.skip("native _core extension not available")
    return nc


@pytest.fixture(scope="module")
def sim(core) -> object:
    """Provide the sim subpackage."""
    return importlib.import_module("texnitis_nav_core.sim")


def _plan_path(core, occupancy_map, start_xy, goal_xy):
    """Run A* on the provided OccupancyMap and return Path2D poses."""
    view = occupancy_map.to_grid_map_view()
    params = core.AStarParams()
    params.inflation_radius = 0.0
    planner = core.AStarPlanner(params)
    status, path = planner.plan_path(
        view,
        core.Pose2D(start_xy[0], start_xy[1], 0.0),
        core.Pose2D(goal_xy[0], goal_xy[1], 0.0),
    )
    assert status == core.PlanResult.Success, f"planner returned {status}"
    return path.poses


def _make_lookahead(core):
    """Make a holonomic Lookahead controller with sane defaults."""
    params = core.LookaheadParams()
    params.kp_xy = 1.0
    params.kp_yaw = 1.2
    params.max_speed_xy = 0.30
    params.max_speed_yaw = 1.0
    params.lookahead_dist = 0.30
    params.use_diff_drive = False
    params.goal_checker.xy_tolerance = 0.10
    params.goal_checker.yaw_tolerance = 0.30
    params.goal_checker.stateful = True
    return core.LookaheadController(params)


@pytest.mark.sim
def test_open_grid_reaches_goal(core, sim):
    """Driving across an empty 10x10 grid reaches the goal."""
    occ = sim.OccupancyMap.from_strings(
        [".........."] * 10,
        resolution=0.10,
    )
    world = sim.World(occupancy=occ, robot=sim.Robot2D(pose=(0.10, 0.10, 0.0)))
    plan = _plan_path(core, occ, (0.10, 0.10), (0.85, 0.85))

    history = sim.run_episode(
        world=world,
        controller=_make_lookahead(core),
        plan=plan,
        config=sim.EpisodeConfig(dt=0.05, max_steps=400, xy_tolerance=0.10),
    )

    assert history.reach_step is not None
    assert sim.collision_count(history) == 0
    assert sim.nan_inf_count(history) == 0
    assert sim.goal_reach_flip_count(history) == 1


@pytest.mark.sim
def test_corridor_navigates_around_wall(core, sim):
    """A short corridor with one wall section is navigated successfully."""
    rows = [
        "..........",
        "..........",
        "..########",
        "..........",
        "..........",
    ]
    occ = sim.OccupancyMap.from_strings(rows, resolution=0.10)
    world = sim.World(occupancy=occ, robot=sim.Robot2D(pose=(0.05, 0.05, 0.0)))
    plan = _plan_path(core, occ, (0.05, 0.05), (0.05, 0.45))

    history = sim.run_episode(
        world=world,
        controller=_make_lookahead(core),
        plan=plan,
        config=sim.EpisodeConfig(dt=0.05, max_steps=600, xy_tolerance=0.10),
    )

    assert history.reach_step is not None
    assert sim.collision_count(history) == 0


@pytest.mark.sim
def test_disturbance_does_not_blow_up(core, sim):
    """Pose noise within 50% of xy_tolerance does not destabilise the run."""
    occ = sim.OccupancyMap.from_strings(
        [".........."] * 10,
        resolution=0.10,
    )
    world = sim.World(occupancy=occ, robot=sim.Robot2D(pose=(0.10, 0.10, 0.0)))
    plan = _plan_path(core, occ, (0.10, 0.10), (0.85, 0.50))

    disturbance = sim.Disturbance(
        sim.DisturbanceConfig(pose_noise_xy=0.02, pose_noise_yaw=0.05, seed=42)
    )
    history = sim.run_episode(
        world=world,
        controller=_make_lookahead(core),
        plan=plan,
        config=sim.EpisodeConfig(dt=0.05, max_steps=600, xy_tolerance=0.10),
        disturbance=disturbance,
    )

    assert history.reach_step is not None
    assert sim.nan_inf_count(history) == 0
    # Stateful flag must NOT flip back; cumulative goal-reach transitions == 1.
    assert sim.goal_reach_flip_count(history) == 1
