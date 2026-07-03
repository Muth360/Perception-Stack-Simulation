cmake_minimum_required(VERSION 3.16)
project(perception_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ---------------------------------------------------------------------------
# Core library (everything except the executable entry point)
# ---------------------------------------------------------------------------
add_library(perception_sim_core
    src/eigen_ops.cpp
    src/csv_logger.cpp
    src/point_cloud_io.cpp
    src/filtering.cpp
    src/clustering.cpp
    src/pca_analysis.cpp
    src/bounding_box.cpp
    src/visualization.cpp
    src/logger.cpp
    src/timing.cpp
    src/pipeline.cpp
)

target_include_directories(perception_sim_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(perception_sim_core
    PUBLIC
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        Eigen3::Eigen
)

# ---------------------------------------------------------------------------
# Main executable
# ---------------------------------------------------------------------------
add_executable(perception_sim_app src/main.cpp)
target_link_libraries(perception_sim_app PRIVATE perception_sim_core)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
enable_testing()
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        include(FetchContent)
        FetchContent_Declare(
            googletest
            URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        )
        # Windows-specific option kept off; harmless on Linux.
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
    endif()

    add_executable(perception_sim_tests
        tests/test_eigen_ops.cpp
        tests/test_filtering.cpp
        tests/test_clustering.cpp
        tests/test_pca_analysis.cpp
        tests/test_bounding_box.cpp
    )
    target_link_libraries(perception_sim_tests
        PRIVATE perception_sim_core gtest gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(perception_sim_tests)
endif()
