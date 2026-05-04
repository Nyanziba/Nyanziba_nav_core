cmake_minimum_required(VERSION 3.20)
project(texnitis_nav_core_demos CXX)

# Stand-alone CMake project. Configure with
#
#   cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/install
#
# where /path/to/install is the prefix where `cmake --install` placed
# texnitis_nav_core. Or build from a parent project that already
# exposes texnitis_nav_core::texnitis_nav_core in scope.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT TARGET texnitis_nav_core::texnitis_nav_core)
    find_package(texnitis_nav_core REQUIRED CONFIG)
endif()

add_executable(astar_lookahead astar_lookahead/main.cpp)
target_link_libraries(astar_lookahead PRIVATE texnitis_nav_core::texnitis_nav_core)

add_executable(mppi mppi/main.cpp)
target_link_libraries(mppi PRIVATE texnitis_nav_core::texnitis_nav_core)
