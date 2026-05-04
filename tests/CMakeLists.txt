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
