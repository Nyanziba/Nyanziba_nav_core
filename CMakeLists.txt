cmake_minimum_required(VERSION 3.20)
project(texnitis_nav_core
    VERSION 0.0.1
    DESCRIPTION "ROS-independent C++ navigation core (A* / Pure Pursuit / MPPI)"
    LANGUAGES CXX
)

# -----------------------------------------------------------------------------
# Milestone M1: header-only INTERFACE library exposing the core POD types,
# logger and result enums. Implementation sources (planners / controllers /
# MPPI) are added in later milestones.
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

add_library(texnitis_nav_core INTERFACE)
add_library(texnitis_nav_core::texnitis_nav_core ALIAS texnitis_nav_core)

target_include_directories(texnitis_nav_core
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(texnitis_nav_core INTERFACE cxx_std_20)

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
