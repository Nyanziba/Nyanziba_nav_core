cmake_minimum_required(VERSION 3.20)
project(texnitis_nav_core
    VERSION 0.0.1
    DESCRIPTION "ROS-independent C++ navigation core (A* / Pure Pursuit / MPPI)"
    LANGUAGES CXX
)

# -----------------------------------------------------------------------------
# This file is a scaffold. Implementation is added incrementally per the plan in
# docs/architecture.md and docs/contributing/release_process.md. The build is
# intentionally a no-op until M1 lands; downstream tooling (pip / colcon) can
# still discover the project and verify configure-time wiring.
# -----------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(NAV_CORE_WITH_MPPI "Build CasADi-backed mecanum MPPI controller" ON)
option(NAV_CORE_BUILD_TESTS "Build GoogleTest unit tests" ON)
option(NAV_CORE_BUILD_EXAMPLES "Build C++ example executables" OFF)
option(NAV_CORE_BUILD_PYTHON "Build pybind11 Python bindings" OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Sources, install rules, tests and Python bindings will be wired up in
# subsequent milestones. This scaffold purposely stops here so a fresh clone
# can configure cleanly without requiring Eigen / CasADi / pybind11 to be
# installed yet.

message(STATUS "texnitis_nav_core: scaffold configure complete (no targets yet)")
message(STATUS "  NAV_CORE_WITH_MPPI    = ${NAV_CORE_WITH_MPPI}")
message(STATUS "  NAV_CORE_BUILD_TESTS  = ${NAV_CORE_BUILD_TESTS}")
message(STATUS "  NAV_CORE_BUILD_PYTHON = ${NAV_CORE_BUILD_PYTHON}")
