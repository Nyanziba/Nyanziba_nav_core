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
    """Locate the harness shared library under a build directory."""
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
        """Bind argtypes / restype on every extern "C" function."""
        c_char_p = ctypes.c_char_p
        c_int32 = ctypes.c_int32
        c_size_t = ctypes.c_size_t
        c_double = ctypes.c_double
        c_int8_p = ctypes.POINTER(ctypes.c_int8)
        c_double_p = ctypes.POINTER(ctypes.c_double)
        c_int32_p = ctypes.POINTER(ctypes.c_int32)

        lib.legacy_init.restype = c_int32
        lib.legacy_init.argtypes = []
        lib.legacy_shutdown.restype = c_int32
        lib.legacy_shutdown.argtypes = []

        lib.legacy_planner_create.restype = c_int32
        lib.legacy_planner_create.argtypes = [c_char_p, c_char_p, c_char_p, c_size_t]

        lib.legacy_planner_plan.restype = c_int32
        lib.legacy_planner_plan.argtypes = [
            c_int32, c_int8_p, c_int32, c_int32, c_double, c_double, c_double,
            c_double_p, c_double_p, c_double_p, c_int32, c_int32_p,
            c_char_p, c_size_t,
        ]
        lib.legacy_planner_cancel.restype = c_int32
        lib.legacy_planner_cancel.argtypes = [c_int32]
        lib.legacy_planner_destroy.restype = c_int32
        lib.legacy_planner_destroy.argtypes = [c_int32]

        lib.legacy_controller_create.restype = c_int32
        lib.legacy_controller_create.argtypes = [c_char_p, c_char_p, c_char_p, c_size_t]

        lib.legacy_controller_set_path.restype = c_int32
        lib.legacy_controller_set_path.argtypes = [c_int32, c_double_p, c_int32, c_char_p, c_size_t]

        lib.legacy_controller_compute.restype = c_int32
        lib.legacy_controller_compute.argtypes = [
            c_int32, c_double_p, c_double_p, c_double_p, c_int32_p, c_char_p, c_size_t,
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
            c_char_p, c_size_t,
        ]


class LegacyHarness:
    """Typed wrapper around the harness shared library."""

    _PATH_CAPACITY = 4096   # max poses per planner output

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
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _err_buf() -> ctypes.Array[ctypes.c_char]:
        return ctypes.create_string_buffer(_ERR_BUF_LEN)

    @staticmethod
    def _err_text(buf: ctypes.Array[ctypes.c_char]) -> str:
        return buf.value.decode("utf-8", errors="replace")

    # ------------------------------------------------------------------
    # Planner
    # ------------------------------------------------------------------

    def create_planner(self, plugin_name: str, params: dict[str, Any]) -> int:
        """Construct a planner plugin instance. Returns its handle."""
        err = self._err_buf()
        handle = self._lib.legacy_planner_create(
            plugin_name.encode("utf-8"),
            json.dumps(params).encode("utf-8"),
            err,
            _ERR_BUF_LEN,
        )
        if handle < 0:
            raise RuntimeError(f"legacy_planner_create({plugin_name}) failed: "
                               f"{self._err_text(err)}")
        return int(handle)

    def plan(self, handle: int,
             *,
             occupancy: list[list[int]],
             resolution: float,
             origin: tuple[float, float],
             start: tuple[float, float, float],
             goal: tuple[float, float, float]) -> tuple[int, list[tuple[float, float, float]]]:
        """Run planPath() and return (status, path)."""
        height = len(occupancy)
        width = len(occupancy[0]) if height > 0 else 0
        flat: list[int] = []
        for row in occupancy:
            flat.extend(row)

        c_int8 = ctypes.c_int8
        c_double = ctypes.c_double
        map_arr = (c_int8 * len(flat))(*flat)
        start_arr = (c_double * 3)(*start)
        goal_arr = (c_double * 3)(*goal)
        out_path = (c_double * (self._PATH_CAPACITY * 3))()
        out_len = ctypes.c_int32(0)
        err = self._err_buf()

        status = self._lib.legacy_planner_plan(
            ctypes.c_int32(handle),
            ctypes.cast(map_arr, ctypes.POINTER(c_int8)),
            ctypes.c_int32(width),
            ctypes.c_int32(height),
            ctypes.c_double(resolution),
            ctypes.c_double(origin[0]),
            ctypes.c_double(origin[1]),
            ctypes.cast(start_arr, ctypes.POINTER(c_double)),
            ctypes.cast(goal_arr, ctypes.POINTER(c_double)),
            ctypes.cast(out_path, ctypes.POINTER(c_double)),
            ctypes.c_int32(self._PATH_CAPACITY),
            ctypes.byref(out_len),
            err,
            ctypes.c_size_t(_ERR_BUF_LEN),
        )
        if status < 0:
            raise RuntimeError(f"legacy_planner_plan failed: {self._err_text(err)}")
        path: list[tuple[float, float, float]] = []
        for i in range(out_len.value):
            base = i * 3
            path.append((out_path[base], out_path[base + 1], out_path[base + 2]))
        return int(status), path

    def destroy_planner(self, handle: int) -> None:
        """Free the planner instance."""
        self._lib.legacy_planner_destroy(handle)

    # ------------------------------------------------------------------
    # Controller
    # ------------------------------------------------------------------

    def create_controller(self, plugin_name: str, params: dict[str, Any]) -> int:
        """Construct a controller plugin instance. Returns its handle."""
        err = self._err_buf()
        handle = self._lib.legacy_controller_create(
            plugin_name.encode("utf-8"),
            json.dumps(params).encode("utf-8"),
            err,
            _ERR_BUF_LEN,
        )
        if handle < 0:
            raise RuntimeError(f"legacy_controller_create({plugin_name}) failed: "
                               f"{self._err_text(err)}")
        return int(handle)

    def set_path(self, handle: int, path: list[tuple[float, float, float]]) -> None:
        """Hand a path to the controller (resets it as a side-effect)."""
        c_double = ctypes.c_double
        flat: list[float] = []
        for x, y, yaw in path:
            flat.extend([x, y, yaw])
        arr = (c_double * len(flat))(*flat) if flat else None
        err = self._err_buf()
        status = self._lib.legacy_controller_set_path(
            ctypes.c_int32(handle),
            ctypes.cast(arr, ctypes.POINTER(c_double)) if arr else None,
            ctypes.c_int32(len(path)),
            err,
            ctypes.c_size_t(_ERR_BUF_LEN),
        )
        if status < 0:
            raise RuntimeError(f"legacy_controller_set_path failed: {self._err_text(err)}")

    def compute(self, handle: int,
                pose: tuple[float, float, float],
                vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
                ) -> tuple[int, tuple[float, float, float], bool]:
        """Run computeCommand() and return (status, twist, goal_reached)."""
        c_double = ctypes.c_double
        pose_arr = (c_double * 3)(*pose)
        vel_arr = (c_double * 3)(*vel)
        out_cmd = (c_double * 3)()
        out_reached = ctypes.c_int32(0)
        err = self._err_buf()
        status = self._lib.legacy_controller_compute(
            ctypes.c_int32(handle),
            ctypes.cast(pose_arr, ctypes.POINTER(c_double)),
            ctypes.cast(vel_arr, ctypes.POINTER(c_double)),
            ctypes.cast(out_cmd, ctypes.POINTER(c_double)),
            ctypes.byref(out_reached),
            err,
            ctypes.c_size_t(_ERR_BUF_LEN),
        )
        if status < 0:
            raise RuntimeError(f"legacy_controller_compute failed: {self._err_text(err)}")
        return (int(status),
                (out_cmd[0], out_cmd[1], out_cmd[2]),
                bool(out_reached.value))

    def reset_controller(self, handle: int) -> None:
        """Drop cached path and reset internal state."""
        self._lib.legacy_controller_reset(handle)

    def cancel_controller(self, handle: int) -> None:
        """Same as reset; kept as a separate verb for clarity."""
        self._lib.legacy_controller_cancel(handle)

    def destroy_controller(self, handle: int) -> None:
        """Free the controller instance."""
        self._lib.legacy_controller_destroy(handle)

    # ------------------------------------------------------------------
    # Height grid (HeightAware planner only)
    # ------------------------------------------------------------------

    def set_height_grid(self, handle: int, *,
                        height_data: list[list[int]],
                        resolution: float,
                        origin: tuple[float, float]) -> None:
        """Publish a height grid into the planner via its subscription."""
        height = len(height_data)
        width = len(height_data[0]) if height > 0 else 0
        flat: list[int] = []
        for row in height_data:
            flat.extend(row)
        c_int8 = ctypes.c_int8
        arr = (c_int8 * len(flat))(*flat) if flat else None
        err = self._err_buf()
        status = self._lib.legacy_planner_set_height_grid(
            ctypes.c_int32(handle),
            ctypes.cast(arr, ctypes.POINTER(c_int8)) if arr else None,
            ctypes.c_int32(width),
            ctypes.c_int32(height),
            ctypes.c_double(resolution),
            ctypes.c_double(origin[0]),
            ctypes.c_double(origin[1]),
            err,
            ctypes.c_size_t(_ERR_BUF_LEN),
        )
        if status < 0:
            raise RuntimeError(f"legacy_planner_set_height_grid failed: {self._err_text(err)}")
