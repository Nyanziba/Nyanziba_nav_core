# GoogleTest entry point for the core. Pulled in via FetchContent so a fresh
# clone does not need a system-installed gtest. Linked against
# texnitis_nav_core (the INTERFACE target) to verify header-only consumption
# from a downstream perspective.

include(FetchContent)

set(_gtest_tag v1.14.0)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG ${_gtest_tag}
    GIT_SHALLOW TRUE
)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

add_executable(test_core_smoke
    test_core_smoke.cpp
)

target_link_libraries(test_core_smoke
    PRIVATE
        texnitis_nav_core
        GTest::gtest_main
)

gtest_discover_tests(test_core_smoke)

add_executable(test_m2_planning
    test_m2_planning.cpp
)

target_link_libraries(test_m2_planning
    PRIVATE
        texnitis_nav_core
        GTest::gtest_main
)

gtest_discover_tests(test_m2_planning)

add_executable(test_map_preprocessor test_map_preprocessor.cpp)
target_link_libraries(test_map_preprocessor PRIVATE texnitis_nav_core GTest::gtest_main)
gtest_discover_tests(test_map_preprocessor)

add_executable(test_motion_models test_motion_models.cpp)
target_link_libraries(test_motion_models PRIVATE texnitis_nav_core GTest::gtest_main)
gtest_discover_tests(test_motion_models)

add_executable(test_kinematic_time_astar test_kinematic_time_astar.cpp)
target_link_libraries(test_kinematic_time_astar PRIVATE texnitis_nav_core GTest::gtest_main)
gtest_discover_tests(test_kinematic_time_astar)

add_executable(test_m4_purepursuit
    test_m4_purepursuit.cpp
)

target_link_libraries(test_m4_purepursuit
    PRIVATE
        texnitis_nav_core
        GTest::gtest_main
)

gtest_discover_tests(test_m4_purepursuit)

if(NAV_CORE_WITH_MPPI)
    add_executable(test_m5_mppi
        test_m5_mppi.cpp
    )

    target_link_libraries(test_m5_mppi
        PRIVATE
            texnitis_nav_core
            GTest::gtest_main
    )

    gtest_discover_tests(test_m5_mppi)
endif()
