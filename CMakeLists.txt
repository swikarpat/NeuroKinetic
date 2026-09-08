cmake_minimum_required(VERSION 3.28)
project(NeuroKineticKernel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -Wpedantic -march=native")

include_directories(
    /opt/homebrew/include
    src
)
link_directories(/opt/homebrew/lib)

find_package(Eigen3 3.4 QUIET CONFIG)
if(NOT Eigen3_FOUND)
    find_package(Eigen3 3.4 REQUIRED)
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

find_library(ROCKSDB_LIB rocksdb PATHS /opt/homebrew/lib REQUIRED)

# 1. Standalone Verification Benchmark
add_executable(cbf_kernel_benchmark
    src/kernel/cbf_solver.cpp
    tests/benchmark_cbf.cpp
)
target_link_libraries(cbf_kernel_benchmark PRIVATE Eigen3::Eigen)

# 2. 500 Hz Real-Time Safety Daemon
add_executable(neurokinetic_daemon
    src/kernel/cbf_solver.cpp
    src/storage/flight_recorder.cpp
    src/kernel/realtime_daemon.cpp
)
target_link_libraries(neurokinetic_daemon PRIVATE 
    Eigen3::Eigen 
    ${ROCKSDB_LIB}
)

# 3. RocksDB Incident Reader Binary
add_executable(read_incidents
    src/storage/read_incidents.cpp
)
target_link_libraries(read_incidents PRIVATE ${ROCKSDB_LIB})
