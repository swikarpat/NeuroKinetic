#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>

namespace neurokinetic::ipc {

inline constexpr const char* SHM_NAME = "/tmp/neurokinetic_ipc.shm";
inline constexpr size_t DOF = 7;

// Sequence Lock Slot for Actuator Telemetry (500 Hz C++ -> Python)
struct alignas(64) TelemetrySlot {
    std::atomic<uint64_t> seq{0};
    uint64_t timestamp_ns{0};
    float joint_positions[DOF]{0.0f};
    float joint_velocities[DOF]{0.0f};
    float applied_torques[DOF]{0.0f};
    float barrier_margin{0.0f};
    uint32_t cbf_intervention{0};
    uint32_t error_code{0};
    uint8_t padding[8]{0};
};

// Command Slot for High-Level VLA Directives (Python -> 500 Hz C++)
struct alignas(64) CommandSlot {
    std::atomic<uint64_t> seq{0};
    uint64_t timestamp_ns{0};
    float target_positions[DOF]{0.0f};
    float nominal_torques[DOF]{0.0f};
    uint32_t command_flags{0};
    uint8_t padding[20]{0};
};

// Global Memory-Mapped Segment
struct alignas(64) SharedMemorySegment {
    std::atomic<uint64_t> heartbeat_cpp{0};
    std::atomic<uint64_t> heartbeat_py{0};
    TelemetrySlot telemetry;
    CommandSlot command;
};

} // namespace neurokinetic::ipc
