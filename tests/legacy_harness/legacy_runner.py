"""ctypes wrapper around tests/legacy_harness/c_api.hpp.

Loads ``libtexnitis_legacy_harness.{so,dylib}`` from the build tree and exposes
a typed Python facade. ``run_legacy.py`` uses this facade to drive each
scenario YAML and emit ``expected.json``.

This module is intentionally self-contained: importing it on a machine
without ROS 2 is harmless as long as you do not call :func:`load_library`.
"""

from __future__ import annotations

import ctypes
import dataclasses
import json
import os
import platform
import sys
from pathlib import Path
from typing import Any, Optional

_ERR_BUF_LEN = 4096


@dataclasses.dataclass(frozen=True)
class PlanResult:
    """Status enum mirrored from c_api.hpp."""

    SUCCESS: int = 0
    NO_PATH_FOUND: int = 50
    INVALID_START: int = 52
    INVALID_GOAL: int = 53
    START_IN_COLLISION: int = 54
    GOAL_IN_COLLISION: int = 55
    NOT_INITIALIZED: int = 59
    ARG_ERROR: int = -1
    INTERNAL_ERROR: int = -2


@dataclasses.dataclass(frozen=True)
class ControllerResult:
    """Status enum mirrored from c_api.hpp."""

    SUCCESS: int = 0
    COMPUTATION_FAILED: int = 100
    INVALID_PATH: int = 104
    CANCELLED: int = 107
    NOT_INITIALIZED: int = 59
    ARG_ERROR: int = -1
    INTERNAL_ERROR: int = -2


def _resolve_lib_path(build_dir: Optional[Path]) -> Path:
    """Locate the harness shared library under a build directory.

    Args:
        build_dir: Path to the cmake build tree (e.g. ``build/legacy_harness``).
            If None, fall back to ``$LEGACY_HARNESS_LIB`` from the environment.
    """
    candidates: list[Path] = []
    if build_dir is not None:
        suffix = ".dylib" if platform.system() == "Darwin" else ".so"
        candidates.append(build_dir / f"libtexnitis_legacy_harness{suffix}")
    env = os.environ.get("LEGACY_HARNESS_LIB")
    if env:
        candidates.append(Path(env))
    for path in candidates:
        if path.exists():
            return path
    raise FileNotFoundError(
        "Could not locate libtexnitis_legacy_harness. Pass --build-dir or "
        "set LEGACY_HARNESS_LIB. Searched: "
        + ", ".join(str(c) for c in candidates)
    )


class _CSignatures:
    """Bundle of the function pointers we declare on ``ctypes.CDLL``."""

    @staticmethod
    def configure(lib: ctypes.CDLL) -> None:
        c_char_p = ctypes.c_char_p
        c_int32 = ctypes.c_int32
        c_size_t = ctypes.c_size_t
        c_double = ctypes.c_double
        c_int8_p = ctypes.POINTER(ctypes.c_int8)
        c_double_p = ctypes.POINTER(ctypes.c_double)
        c_int32_p = ctypes.POINTER(ctypes.c_int32)
        c_char_p_buf = ctypes.c_char_p

        lib.legacy_init.restype = c_int32
        lib.legacy_init.argtypes = []
        lib.legacy_shutdown.restype = c_int32
        lib.legacy_shutdown.argtypes = []

        lib.legacy_planner_create.restype = c_int32
        lib.legacy_planner_create.argtypes = [c_char_p, c_char_p, c_char_p_buf, c_size_t]

        lib.legacy_planner_plan.restype = c_int32
        lib.legacy_planner_plan.argtypes = [
            c_int32, c_int8_p, c_int32, c_int32, c_double, c_double, c_double,
            c_double_p, c_double_p, c_double_p, c_int32, c_int32_p,
            c_char_p_buf, c_size_t,
        ]
        lib.legacy_planner_cancel.restype = c_int32
        lib.legacy_planner_cancel.argtypes = [c_int32]
        lib.legacy_planner_destroy.restype = c_int32
        lib.legacy_planner_destroy.argtypes = [c_int32]

        lib.legacy_controller_create.restype = c_int32
        lib.legacy_controller_create.argtypes = [c_char_p, c_char_p, c_char_p_buf, c_size_t]

        lib.legacy_controller_set_path.restype = c_int32
        lib.legacy_controller_set_path.argtypes = [c_int32, c_double_p, c_int32, c_char_p_buf, c_size_t]

        lib.legacy_controller_compute.restype = c_int32
        lib.legacy_controller_compute.argtypes = [
            c_int32, c_double_p, c_double_p, c_double_p, c_int32_p, c_char_p_buf, c_size_t,
        ]
        lib.legacy_controller_reset.restype = c_int32
        lib.legacy_controller_reset.argtypes = [c_int32]
        lib.legacy_controller_cancel.restype = c_int32
        lib.legacy_controller_cancel.argtypes = [c_int32]
        lib.legacy_controller_destroy.restype = c_int32
        lib.legacy_controller_destroy.argtypes = [c_int32]

        lib.legacy_planner_set_height_grid.restype = c_int32
        lib.legacy_planner_set_height_grid.argtypes = [
            c_int32, c_int8_p, c_int32, c_int32, c_double, c_double, c_double,
            c_char_p_buf, c_size_t,
        ]


class LegacyHarness:
    """Typed wrapper around the harness shared library."""

    def __init__(self, build_dir: Optional[Path] = None) -> None:
        path = _resolve_lib_path(build_dir)
        self._lib = ctypes.CDLL(str(path))
        _CSignatures.configure(self._lib)
        if self._lib.legacy_init() != 0:
            raise RuntimeError("legacy_init failed")

    def __del__(self) -> None:
        try:
            self._lib.legacy_shutdown()
        except Exception:  # pragma: no cover - best effort during teardown
            pass

    # ------------------------------------------------------------------
    # Planner
    # ------------------------------------------------------------------

    def create_planner(self, plugin_name: str, params: dict[str, Any]) -> int:
        err = ctypes.create_string_buffer(_ERR_BUF_LEN)
        handle = self._lib.legacy_planner_create(
            plugin_name.encode("utf-8"),
            json.dumps(params).encode("utf-8"),
            err,
            _ERR_BUF_LEN,
        )
        if handle < 0:
            raise RuntimeError(f"legacy_planner_create({plugin_name}) failed: "
                               f"{err.value.decode('utf-8', errors='replace')}")
        return int(handle)

    def destroy_planner(self, handle: int) -> None:
        self._lib.legacy_planner_destroy(handle)

    # plan() will be filled in alongside the c_api.cpp TODO blocks.
    # Stub kept here so importing the module is still valid.

    # ------------------------------------------------------------------
    # Controller
    # ------------------------------------------------------------------

    def create_controller(self, plugin_name: str, params: dict[str, Any]) -> int:
        err = ctypes.create_string_buffer(_ERR_BUF_LEN)
        handle = self._lib.legacy_controller_create(
            plugin_name.encode("utf-8"),
            json.dumps(params).encode("utf-8"),
            err,
            _ERR_BUF_LEN,
        )
        if handle < 0:
            raise RuntimeError(f"legacy_controller_create({plugin_name}) failed: "
                               f"{err.value.decode('utf-8', errors='replace')}")
        return int(handle)

    def reset_controller(self, handle: int) -> None:
        self._lib.legacy_controller_reset(handle)

    def cancel_controller(self, handle: int) -> None:
        self._lib.legacy_controller_cancel(handle)

    def destroy_controller(self, handle: int) -> None:
        self._lib.legacy_controller_destroy(handle)


def main() -> int:
    """Sanity entrypoint: print the resolved library path and exit."""
    if len(sys.argv) > 1:
        build_dir = Path(sys.argv[1])
    else:
        build_dir = Path("build/legacy_harness")
    try:
        harness = LegacyHarness(build_dir)
    except Exception as exc:  # pragma: no cover - depends on local build
        print(f"failed to load harness: {exc}", file=sys.stderr)
        return 1
    print(f"loaded {harness}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
