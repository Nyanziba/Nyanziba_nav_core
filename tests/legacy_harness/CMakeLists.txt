cmake_minimum_required(VERSION 3.20)
project(texnitis_nav_core_legacy_harness CXX)

# This CMakeLists is configured stand-alone (not via the parent project) so
# the harness can be built only when ROS 2 is available. It produces a single
# shared library `legacy_harness.so` that exposes a small extern "C" surface
# Python can drive via ctypes (see legacy_runner.py).
#
# Required environment:
#   - ROS 2 Jazzy.
#     - Linux  (Ubuntu 24.04): `source /opt/ros/jazzy/setup.bash`
#     - macOS  (pixi global):  `source /Users/<you>/.pixi/envs/default/setup.zsh`
#       (the `default` env in pixi-global.toml has ros-jazzy-desktop installed)
#   - Then source the `install/setup.bash` of a colcon workspace that has
#     built `texnitis_move_base_like` (and optionally
#     `mecanum_mppi_controller`) first, so pluginlib can resolve the
#     legacy plugin classes.
#
# Usage from the texnitis_nav_core repo root:
#   cmake -S tests/legacy_harness -B build/legacy_harness
#   cmake --build build/legacy_harness -j

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------

find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND)
    message(WARNING
        "ament_cmake not found. The legacy harness requires a sourced ROS 2 "
        "Jazzy environment. Skipping target generation.")
    return()
endif()

# nlohmann_json for parsing the JSON parameter blobs that legacy_runner.py
# hands over the C ABI. Pulled in via FetchContent so the harness does not
# require a system install.
include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
)
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(nlohmann_json)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(texnitis_move_base_like REQUIRED)

# Optional: MPPI engine. If absent we silently drop S-10 from the run set.
find_package(casadi QUIET)
find_package(mecanum_mppi_controller QUIET)

# -----------------------------------------------------------------------------
# Library
# -----------------------------------------------------------------------------

add_library(legacy_harness SHARED
    fake_node/fake_node.cpp
    c_api.cpp
)

target_include_directories(legacy_harness
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/fake_node
)

ament_target_dependencies(legacy_harness
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    pluginlib
    texnitis_move_base_like
)

target_link_libraries(legacy_harness PRIVATE nlohmann_json::nlohmann_json)

if(casadi_FOUND AND mecanum_mppi_controller_FOUND)
    target_compile_definitions(legacy_harness PRIVATE LEGACY_HARNESS_HAS_MPPI=1)
    ament_target_dependencies(legacy_harness mecanum_mppi_controller)
    target_link_libraries(legacy_harness casadi)
endif()

# Python loads the library via ctypes from a known path. Stamp it next to the
# build tree so legacy_runner.py can locate it without an install step.
set_target_properties(legacy_harness PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    OUTPUT_NAME texnitis_legacy_harness
    POSITION_INDEPENDENT_CODE ON
)
