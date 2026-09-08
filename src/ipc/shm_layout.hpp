#pragma once

#include <cstdint>
#include <atomic>
#include <cstddef>
#include "kernel/cbf_solver.hpp"

namespace neurokinetic::ipc {

inline constexpr const char* SHM_NAME = "/tmp/neurokinetic_ipc.shm";

// 64-byte cache-line aligned telemetry frame (C++ -> Python / UI)
struct alignas(64) TelemetrySlot {
    std::atomic<uint64_t> seq{0};
    uint64_t timestamp_ns{0};
    float joint_positions[neurokinetic::DOF]{0.0f};
    float joint_velocities[neurokinetic::DOF]{0.0f};
    float applied_torques[neurokinetic::DOF]{0.0f};
    float barrier_margin{0.0f};
    uint32_t cbf_intervention{0};
    uint32_t error_code{0};
    uint32_t watchdog_tripped{0};
    uint8_t padding[16]{0};
};

// 64-byte cache-line aligned command frame (Python / UI -> C++)
struct alignas(64) CommandSlot {
    std::atomic<uint64_t> seq{0};
    float nominal_torques[neurokinetic::DOF]{0.0f};
    float target_positions[neurokinetic::DOF]{0.0f};
    float obstacle_safety_margin{0.08f}; // Dynamic barrier margin (m)
    float barrier_gamma{15.0f};          // Dynamic barrier stiffness
    uint32_t command_flags{0};           // Bit 0: Active, Bit 1: E-Stop, Bit 2: Hazard
    uint8_t padding[16]{0};
};

// Top-level memory segment
struct alignas(64) SharedMemorySegment {
    std::atomic<uint32_t> magic{0x4E455552}; // 'NEUR'
    std::atomic<uint64_t> heartbeat_cpp{0};
    std::atomic<uint64_t> heartbeat_python{0};
    TelemetrySlot telemetry;
    CommandSlot command;
};

} // namespace neurokinetic::ipc
