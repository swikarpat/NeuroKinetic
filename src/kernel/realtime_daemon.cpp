#include "kernel/cbf_solver.hpp"
#include "ipc/shm_layout.hpp"
#include "storage/flight_recorder.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace neurokinetic;
using namespace neurokinetic::kernel;
using namespace neurokinetic::ipc;
using namespace neurokinetic::storage;

std::atomic<bool> g_running{true};

void handle_signal([[maybe_unused]] int sig) {
    g_running.store(false);
}

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::cout << "========================================================\n";
    std::cout << "  NeuroKinetic 500 Hz Real-Time C++23 Safety Daemon     \n";
    std::cout << "  Conduit Path: " << neurokinetic::ipc::SHM_NAME << "\n";
    std::cout << "  Flight Recorder: RocksDB (/tmp/neurokinetic_flight_recorder)\n";
    std::cout << "========================================================\n";

    // 1. Initialize POSIX Shared Memory
    int shm_fd = open(neurokinetic::ipc::SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        std::cerr << "[Fatal Error] Failed to open shared memory file\n";
        return 1;
    }

    if (ftruncate(shm_fd, sizeof(SharedMemorySegment)) != 0) {
        std::cerr << "[Fatal Error] Failed to set SHM segment size\n";
        return 1;
    }

    auto* shm = static_cast<SharedMemorySegment*>(
        mmap(nullptr, sizeof(SharedMemorySegment), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0)
    );

    if (shm == MAP_FAILED) {
        std::cerr << "[Fatal Error] Failed to mmap shared memory\n";
        return 1;
    }

    new (shm) SharedMemorySegment();

    // 2. Initialize Embedded RocksDB Flight Recorder
    FlightRecorder flight_recorder("/tmp/neurokinetic_flight_recorder");
    std::cout << "✓ Embedded RocksDB Flight Recorder active across 2 Column Families\n";

    // 3. Initialize Kinematic Envelopes & CBF Kernel
    KinematicLimits limits{
        .q_min = Eigen::Vector<double, neurokinetic::DOF>::Constant(-2.89),
        .q_max = Eigen::Vector<double, neurokinetic::DOF>::Constant(2.89),
        .q_dot_max = Eigen::Vector<double, neurokinetic::DOF>::Constant(2.17),
        .torque_max = Eigen::Vector<double, neurokinetic::DOF>::Constant(87.0),
        .min_obstacle_distance = 0.08
    };
    ControlBarrierKernel kernel(limits);

    ManipulatorState state{
        .q = (Eigen::Vector<double, neurokinetic::DOF>() << 0.0, 0.2, 0.0, -1.2, 0.0, 2.80, 0.0).finished(),
        .q_dot = Eigen::Vector<double, neurokinetic::DOF>::Zero(),
        .end_effector_pos = Eigen::Vector3d(0.45, 0.10, 0.32),
        .end_effector_vel = Eigen::Vector3d::Zero()
    };

    std::vector<Obstacle> obstacles = {
        Obstacle{.position = Eigen::Vector3d(0.50, 0.10, 0.30), .radius = 0.05}
    };

    constexpr auto TICK_DURATION = std::chrono::microseconds(2000); // 500 Hz = 2000 µs
    uint64_t tick_count = 0;

    std::cout << "[Kernel Online] Real-time 500 Hz loop active. Awaiting VLA commands...\n";

    while (g_running.load(std::memory_order_relaxed)) {
        const auto tick_start = std::chrono::steady_clock::now();

        shm->heartbeat_cpp.fetch_add(1, std::memory_order_relaxed);

        // Read command from Python via Seqlock
        Eigen::Vector<double, neurokinetic::DOF> nominal_torque = Eigen::Vector<double, neurokinetic::DOF>::Zero();
        uint32_t flags = 0;

        uint64_t cmd_seq1 = shm->command.seq.load(std::memory_order_acquire);
        if ((cmd_seq1 & 1) == 0 && cmd_seq1 != 0) {
            for (size_t i = 0; i < neurokinetic::DOF; ++i) {
                nominal_torque(i) = shm->command.nominal_torques[i];
            }
            flags = shm->command.command_flags;
            uint64_t cmd_seq2 = shm->command.seq.load(std::memory_order_acquire);
            if (cmd_seq1 != cmd_seq2) {
                nominal_torque.setZero();
            }
        }

        if (flags & 2) {
            nominal_torque.setZero();
        }

        // Evaluate Control Barrier Function
        auto result = kernel.filter_torques(state, nominal_torque, obstacles);
        SafetyOutput safety = result.value_or(SafetyOutput{
            .safe_torque = Eigen::Vector<double, neurokinetic::DOF>::Zero(),
            .intervention_triggered = true,
            .solve_time_us = 0.0,
            .barrier_margin = 0.0
        });

        // Integrate simulated dynamics
        for (size_t i = 0; i < neurokinetic::DOF; ++i) {
            double acceleration = safety.safe_torque(i) / 10.0;
            state.q_dot(i) += acceleration * 0.002;
            state.q_dot(i) *= 0.98;
            state.q(i) += state.q_dot(i) * 0.002;
        }

        uint64_t current_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        // RocksDB Flight Recording: Persist intervention events to local NVMe
        if (safety.intervention_triggered) {
            InterventionRecord record{
                .sequence_number = tick_count,
                .timestamp_ns = current_ts,
                .barrier_margin = static_cast<float>(safety.barrier_margin),
                .commanded_joint5_torque = static_cast<float>(nominal_torque(5)),
                .safe_joint5_torque = static_cast<float>(safety.safe_torque(5)),
                .active_mask = 1
            };
            flight_recorder.log_cbf_intervention(record);
        }

        // Write Telemetry via Seqlock
        uint64_t current_seq = shm->telemetry.seq.load(std::memory_order_relaxed);
        shm->telemetry.seq.store(current_seq + 1, std::memory_order_release);

        shm->telemetry.timestamp_ns = current_ts;

        for (size_t i = 0; i < neurokinetic::DOF; ++i) {
            shm->telemetry.joint_positions[i] = static_cast<float>(state.q(i));
            shm->telemetry.joint_velocities[i] = static_cast<float>(state.q_dot(i));
            shm->telemetry.applied_torques[i] = static_cast<float>(safety.safe_torque(i));
        }
        shm->telemetry.barrier_margin = static_cast<float>(safety.barrier_margin);
        shm->telemetry.cbf_intervention = safety.intervention_triggered ? 1 : 0;
        shm->telemetry.error_code = result.has_value() ? 0 : 1;

        shm->telemetry.seq.store(current_seq + 2, std::memory_order_release);

        tick_count++;
        if (tick_count % 1000 == 0) {
            std::cout << "[500 Hz Tick: " << tick_count << "] Margin: " 
                      << safety.barrier_margin << " m | Safe T[5]: " 
                      << safety.safe_torque(5) << " Nm | Clamped: " 
                      << (safety.intervention_triggered ? "YES" : "NO") 
                      << " | RocksDB: Active\n";
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - tick_start
        );
        if (elapsed < TICK_DURATION) {
            std::this_thread::sleep_for(TICK_DURATION - elapsed);
        }
    }

    std::cout << "\n[Shutdown] Flushing RocksDB and cleaning shared memory...\n";
    flight_recorder.flush();
    munmap(shm, sizeof(SharedMemorySegment));
    close(shm_fd);
    return 0;
}
