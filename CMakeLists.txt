cmake_minimum_required(VERSION 3.16)
project(sensor_fusion_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)

# --- Eigen (header-only linear algebra) ----------------------------------
# Pulled at configure time so the repo has no vendored third-party code and
# no manual install step. See docs/adr/0003.
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)  # don't build Eigen's own test suite
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(Eigen)

# --- Fusion library -------------------------------------------------------
add_library(sft STATIC cpp/src/tracker.cpp)
target_include_directories(sft PUBLIC cpp/include)
target_link_libraries(sft PUBLIC Eigen3::Eigen)
target_compile_options(sft PRIVATE -Wall -Wextra)

# --- CLI application ------------------------------------------------------
add_executable(sft_app cpp/src/main.cpp)
target_link_libraries(sft_app PRIVATE sft)

# --- Unit tests -----------------------------------------------------------
option(SFT_BUILD_TESTS "Build the GoogleTest suite" ON)
if(SFT_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_executable(sft_tests cpp/tests/test_kalman.cpp)
    target_link_libraries(sft_tests PRIVATE sft GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(sft_tests)
endif()
