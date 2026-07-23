// extern "C" surface that Python (legacy_runner.py via ctypes) calls into.
//
// Design notes:
//   - All buffers are caller-allocated. The harness writes into them and
//     returns a count. There are no raw allocations crossing the FFI
//     boundary, no opaque ownership transfer.
//   - Strings are passed as fixed-size char buffers. Errors return a
//     negative status and write a human-readable message into err_buf.
//   - Each plugin instance is referenced by an int32_t handle. The handle
//     must be released with `legacy_*_destroy` to free the underlying
//     plugin (the C++ side keeps a small std::unordered_map<int32_t, ...>).
//   - Calls are NOT thread-safe. Python invokes them from a single thread.
//
// The corresponding cpp file (c_api.cpp) only delegates; the actual
// plugin lifetime, parameter wiring and ROS message conversion live there.
#pragma once

#include <cstddef>
#include <cstdint>

#if defined(_WIN32)
#define LEGACY_HARNESS_API __declspec(dllexport)
#else
#define LEGACY_HARNESS_API __attribute__ ((visibility ("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Process-wide lifecycle
// -----------------------------------------------------------------------------

LEGACY_HARNESS_API int32_t legacy_init (void);
LEGACY_HARNESS_API int32_t legacy_shutdown (void);

// -----------------------------------------------------------------------------
// Planner instances
// -----------------------------------------------------------------------------

/**
 * @brief Load a planner plugin by registered class name.
 *
 * @param plugin_name  e.g. "texnitis_move_base_like::AStarPlanner".
 * @param param_json   UTF-8 JSON object with the parameter overrides.
 *                     The harness translates this to ROS parameter overrides
 *                     before constructing the plugin's host Node.
 * @param err_buf      Caller-owned char buffer for the error message.
 * @param err_buf_len  Size of err_buf in bytes.
 * @return Non-negative handle on success. Negative on failure.
 */
LEGACY_HARNESS_API int32_t
legacy_planner_create (const char *plugin_name,
                       const char *param_json,
                       char       *err_buf,
                       size_t      err_buf_len);

/**
 * @brief Run planPath() on the given handle.
 *
 * @param handle       Planner handle returned by legacy_planner_create.
 * @param map_data     Pointer to occupancy data (int8). Length width*height.
 * @param width        Map width in cells.
 * @param height       Map height in cells.
 * @param resolution   Cell size [m].
 * @param origin_x     Map origin x [m].
 * @param origin_y     Map origin y [m].
 * @param start_pose   [x, y, yaw], length 3.
 * @param goal_pose    [x, y, yaw], length 3.
 * @param out_path     Caller buffer of doubles, length out_path_capacity*3.
 * @param out_path_capacity Max number of poses out_path can hold.
 * @param out_path_len Number of poses actually written. Set on return.
 * @param err_buf      Error buffer.
 * @param err_buf_len  Size of err_buf.
 * @return PlanResult-equivalent status:
 *           0  SUCCESS
 *          50  NO_PATH_FOUND
 *          52  INVALID_START
 *          53  INVALID_GOAL
 *          54  START_IN_COLLISION
 *          55  GOAL_IN_COLLISION
 *          59  NOT_INITIALIZED
 *          -1  argument error
 *          -2  internal C++ exception
 */
LEGACY_HARNESS_API int32_t
legacy_planner_plan (int32_t       handle,
                     const int8_t *map_data,
                     int32_t       width,
                     int32_t       height,
                     double        resolution,
                     double        origin_x,
                     double        origin_y,
                     const double *start_pose,
                     const double *goal_pose,
                     double       *out_path,
                     int32_t       out_path_capacity,
                     int32_t      *out_path_len,
                     char         *err_buf,
                     size_t        err_buf_len);

LEGACY_HARNESS_API int32_t legacy_planner_cancel (int32_t handle);
LEGACY_HARNESS_API int32_t legacy_planner_destroy (int32_t handle);

// -----------------------------------------------------------------------------
// Controller instances
// -----------------------------------------------------------------------------

LEGACY_HARNESS_API int32_t
legacy_controller_create (const char *plugin_name,
                          const char *param_json,
                          char       *err_buf,
                          size_t      err_buf_len);

LEGACY_HARNESS_API int32_t
legacy_controller_set_path (int32_t       handle,
                            const double *path,
                            int32_t       path_len,
                            char         *err_buf,
                            size_t        err_buf_len);

/**
 * @brief Run computeCommand() once.
 *
 * @param handle       Controller handle.
 * @param current_pose [x, y, yaw], length 3.
 * @param current_vel  [vx, vy, wz], length 3.
 * @param out_cmd      [vx, vy, wz], length 3, set on return.
 * @param out_goal_reached 0/1 flag, set on return.
 * @return ControllerResult-equivalent status:
 *           0  SUCCESS / GOAL_REACHED
 *         100  COMPUTATION_FAILED
 *         104  INVALID_PATH (empty path)
 *         107  CANCELLED
 *          -1  argument error
 *          -2  internal C++ exception
 */
LEGACY_HARNESS_API int32_t
legacy_controller_compute (int32_t       handle,
                           const double *current_pose,
                           const double *current_vel,
                           double       *out_cmd,
                           int32_t      *out_goal_reached,
                           char         *err_buf,
                           size_t        err_buf_len);

LEGACY_HARNESS_API int32_t legacy_controller_reset (int32_t handle);
LEGACY_HARNESS_API int32_t legacy_controller_cancel (int32_t handle);
LEGACY_HARNESS_API int32_t legacy_controller_destroy (int32_t handle);

#ifdef __cplusplus
}  // extern "C"
#endif
