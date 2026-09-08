#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include "kernel/cbf_solver.hpp"

namespace neurokinetic::ipc {

#if defined(__APPLE__)
constexpr const char* SHM_PATH = "/tmp/neurokinetic_ipc.shm";
#else
constexpr const char* SHM_PATH = "/dev/shm/neurokinetic_ipc.shm";
#endif

// Slot 1: Actuator Telemetry (500 Hz C++ -> Python)
// 8 (seq) + 8 (ts) + 28 (q) + 28 (q_dot) + 28 (torques) + 4 (margin) + 4 (intervention) + 4 (err) + 16 (pad) = 128 B
struct alignas(64) TelemetrySlot {
    std::atomic<uint64_t> seq{0};
    uint64_t timestamp_ns{0};
    float joint_positions[neurokinetic::DOF]{0.0f};
    float joint_velocities[neurokinetic::DOF]{0.0f};
    float applied_torques[neurokinetic::DOF]{0.0f};
    float barrier_margin{0.0f};
    uint32_t cbf_intervention{0};
    uint32_t error_code{0};
    uint8_t padding[16]{0};
};
static_assert(sizeof(TelemetrySlot) == 128, "TelemetrySlot must be exactly 128 bytes (2 cache lines)");

// Slot 2: High-Level VLA Commands (Python -> 500 Hz C++)
// 8 (seq) + 8 (ts) + 28 (target_pos) + 28 (nominal_torques) + 4 (flags) + 52 (pad) = 128 B
struct alignas(64) CommandSlot {
    std::atomic<uint64_t> seq{0};
    uint64_t timestamp_ns{0};
    float target_positions[neurokinetic::DOF]{0.0f};
    float nominal_torques[neurokinetic::DOF]{0.0f};
    uint32_t command_flags{0};
    uint8_t padding[52]{0};
};
static_assert(sizeof(CommandSlot) == 128, "CommandSlot must be exactly 128 bytes (2 cache lines)");

// Global 320-Byte Shared Memory Segment (5 x 64-byte cache lines)
// Header (64 B) + Telemetry (128 B) + Command (128 B)
struct alignas(64) SharedMemorySegment {
    std::atomic<uint64_t> heartbeat_cpp{0}; // 8 B
    std::atomic<uint64_t> heartbeat_py{0};  // 8 B
    uint8_t header_padding[48]{0};          // 48 B (pads header to 64 B)
    TelemetrySlot telemetry;                // Offset 64
    CommandSlot command;                    // Offset 192
};
static_assert(sizeof(SharedMemorySegment) == 320, "SharedMemorySegment must be exactly 320 bytes");

} // namespace neurokinetic::ipc
