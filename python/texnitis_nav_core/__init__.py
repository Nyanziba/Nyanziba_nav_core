"""texnitis_nav_core: ROS-independent navigation algorithms with a 2D simulator.

The native ``_core`` module is built and installed by scikit-build-core when
``NAV_CORE_BUILD_PYTHON=ON``. If it is missing (e.g. you ran a pure setuptools
install before the M7 commit landed), this module still imports and only
exposes ``__version__`` and a stub-friendly placeholder.
"""

from __future__ import annotations

__version__ = "0.0.1"

try:
    from . import _core  # noqa: F401  - re-exported below
    from ._core import (  # noqa: F401
        AStarParams,
        AStarPlanner,
        ControllerResult,
        DiffDrivePurePursuitController,
        DiffDrivePurePursuitParams,
        GoalCheckerParams,
        GridMapView,
        HeightAwareAStarParams,
        HeightAwareAStarPlanner,
        HeightGridView,
        HeightProvider,
        LookaheadController,
        LookaheadParams,
        Path2D,
        PlanResult,
        Pose2D,
        Twist2D,
        clamp,
        distance_xy,
        normalize_angle,
        shortest_angular_diff,
    )

    try:
        from ._core import mppi  # noqa: F401  - optional MPPI submodule
    except ImportError:  # pragma: no cover - MPPI build disabled at compile time
        mppi = None  # type: ignore
except ImportError:  # pragma: no cover - native extension not yet built
    _core = None  # type: ignore
    mppi = None  # type: ignore

__all__ = [
    "__version__",
    "_core",
    "mppi",
]
