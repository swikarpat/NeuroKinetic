cmake_minimum_required(VERSION 3.28)
project(NeuroKineticKernel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -Wpedantic -march=native")

# Locate Eigen3
find_package(Eigen3 3.4 QUIET CONFIG)
if(NOT Eigen3_FOUND)
    find_package(Eigen3 3.4 REQUIRED)
endif()

include_directories(
    src
    ${EIGEN3_INCLUDE_DIR}
)

# 1. Standalone Verification Benchmark
add_executable(cbf_kernel_benchmark
    src/kernel/cbf_solver.cpp
    tests/benchmark_cbf.cpp
)
target_link_libraries(cbf_kernel_benchmark PRIVATE Eigen3::Eigen)

# 2. 500 Hz Real-Time Safety Daemon with POSIX SHM
add_executable(neurokinetic_daemon
    src/kernel/cbf_solver.cpp
    src/kernel/realtime_daemon.cpp
)
target_link_libraries(neurokinetic_daemon PRIVATE Eigen3::Eigen)
