cmake_minimum_required(VERSION 3.20)
project(texnitis_nav_core
    VERSION 0.0.1
    DESCRIPTION "ROS-independent C++ navigation core (A* / Pure Pursuit / MPPI)"
    LANGUAGES CXX
)

# -----------------------------------------------------------------------------
# Milestones:
#   M1 added the header-only types / grid view / result / logger.
#   M2 (this revision) adds PlannerBase / ControllerBase / GoalChecker plus
#       the AStarPlanner and LookaheadController implementations, switching
#       the library from INTERFACE to STATIC.
#   Later milestones will append DiffDrivePurePursuit and
#   MPPI sources to the same target.
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

# -----------------------------------------------------------------------------
# Library target
# -----------------------------------------------------------------------------

find_package(Eigen3 QUIET NO_MODULE)

add_library(texnitis_nav_core STATIC
    src/map_processing/map_preprocessor.cpp
    src/motion_models/motion_model.cpp
    src/motion_models/holonomic_motion_model.cpp
    src/motion_models/differential_drive_motion_model.cpp
    src/terrain/terrain_traversal_evaluator.cpp
    src/planners/astar_planner.cpp
    src/planners/kinematic_time_astar_planner.cpp
    src/controllers/lookahead_controller.cpp
    src/controllers/diff_drive_pure_pursuit_controller.cpp
)
add_library(texnitis_nav_core::texnitis_nav_core ALIAS texnitis_nav_core)

target_include_directories(texnitis_nav_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(texnitis_nav_core PUBLIC cxx_std_20)
set_target_properties(texnitis_nav_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

if(NAV_CORE_WITH_MPPI)
    if(NOT TARGET Eigen3::Eigen)
        find_package(Eigen3 REQUIRED NO_MODULE)
    endif()
    target_sources(texnitis_nav_core PRIVATE
        src/mppi/mecanum_kinematics.cpp
        src/mppi/mecanum_mppi_controller.cpp
    )
    target_link_libraries(texnitis_nav_core PUBLIC Eigen3::Eigen)
    target_compile_definitions(texnitis_nav_core PUBLIC TEXNITIS_NAV_CORE_WITH_MPPI=1)
endif()

# -----------------------------------------------------------------------------
# Python bindings (pybind11)
# -----------------------------------------------------------------------------

if(NAV_CORE_BUILD_PYTHON)
    find_package(pybind11 CONFIG REQUIRED)
    set(_binding_sources
        bindings/python/module.cpp
        bindings/python/bind_types.cpp
        bindings/python/bind_grid_map.cpp
        bindings/python/bind_planners.cpp
        bindings/python/bind_controllers.cpp
    )
    if(NAV_CORE_WITH_MPPI)
        list(APPEND _binding_sources bindings/python/bind_mppi.cpp)
    endif()
    pybind11_add_module(_core MODULE ${_binding_sources})
    target_link_libraries(_core PRIVATE texnitis_nav_core)
    if(NAV_CORE_WITH_MPPI)
        target_compile_definitions(_core PRIVATE TEXNITIS_NAV_CORE_WITH_MPPI=1)
    endif()
    install(TARGETS _core
        LIBRARY DESTINATION texnitis_nav_core
    )
endif()

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------

if(NAV_CORE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# -----------------------------------------------------------------------------
# Install / export
# -----------------------------------------------------------------------------

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING
        PATTERN "*.hpp"
)

install(TARGETS texnitis_nav_core
    EXPORT texnitis_nav_coreTargets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT texnitis_nav_coreTargets
    FILE texnitis_nav_coreTargets.cmake
    NAMESPACE texnitis_nav_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/texnitis_nav_core
)

set(_config_in ${CMAKE_CURRENT_LIST_DIR}/cmake/texnitis_nav_coreConfig.cmake.in)
if(NOT EXISTS ${_config_in})
    file(WRITE ${_config_in} [[
@PACKAGE_INIT@
include("${CMAKE_CURRENT_LIST_DIR}/texnitis_nav_coreTargets.cmake")
check_required_components(texnitis_nav_core)
]])
endif()

configure_package_config_file(
    ${_config_in}
    ${CMAKE_CURRENT_BINARY_DIR}/texnitis_nav_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/texnitis_nav_core
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/texnitis_nav_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/texnitis_nav_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/texnitis_nav_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/texnitis_nav_core
)

message(STATUS "texnitis_nav_core: M1 INTERFACE library configured")
message(STATUS "  NAV_CORE_WITH_MPPI    = ${NAV_CORE_WITH_MPPI}")
message(STATUS "  NAV_CORE_BUILD_TESTS  = ${NAV_CORE_BUILD_TESTS}")
message(STATUS "  NAV_CORE_BUILD_PYTHON = ${NAV_CORE_BUILD_PYTHON}")
