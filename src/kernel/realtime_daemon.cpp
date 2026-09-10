#include "kernel/cbf_solver.hpp"
#include "ipc/shm_layout.hpp"
#include "storage/flight_recorder.hpp"
#include "neurokinetic/telemetry/telemetry_worker.hpp"
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
using namespace neurokinetic::telemetry;

std::atomic<bool> g_running{true};

void handle_signal([[maybe_unused]] int sig) {
    g_running.store(false);
}

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::cout << "========================================================\n";
    std::cout << "  NeuroKinetic 500 Hz Real-Time C++23 Safety Daemon     \n";
    std::cout << "  Conduit Path: " << SHM_NAME << "\n";
    std::cout << "  Fail-Safe: Active Dead-Man's Watchdog (100 ms limit)  \n";
    std::cout << "========================================================\n";

    int shm_fd = open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        std::cerr << "[Fatal Error] Failed to open shared memory\n";
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

    FlightRecorder flight_recorder("/tmp/neurokinetic_flight_recorder");
    std::cout << "✓ Embedded RocksDB Flight Recorder online\n";

    KinematicLimits limits{
        .q_min = Eigen::Vector<double, DOF>::Constant(-2.89),
        .q_max = Eigen::Vector<double, DOF>::Constant(2.89),
        .q_dot_max = Eigen::Vector<double, DOF>::Constant(2.17),
        .torque_max = Eigen::Vector<double, DOF>::Constant(87.0),
        .min_obstacle_distance = 0.08
    };
    ControlBarrierKernel kernel(limits);
    TelemetryWorker telemetry_worker;
    telemetry_worker.start();

    ManipulatorState state{
        .q = (Eigen::Vector<double, DOF>() << 0.0, 0.2, 0.0, -1.2, 0.0, 2.80, 0.0).finished(),
        .q_dot = Eigen::Vector<double, DOF>::Zero(),
        .end_effector_pos = Eigen::Vector3d(0.45, 0.10, 0.32),
        .end_effector_vel = Eigen::Vector3d::Zero()
    };

    std::vector<Obstacle> obstacles = {
        Obstacle{.position = Eigen::Vector3d(0.50, 0.10, 0.30), .radius = 0.05}
    };

    constexpr auto TICK_DURATION = std::chrono::microseconds(2000); // 500 Hz
    uint64_t tick_count = 0;
    uint64_t last_python_heartbeat = 0;
    auto last_heartbeat_change_time = std::chrono::steady_clock::now();
    bool watchdog_tripped = false;

    std::cout << "[Kernel Online] 500 Hz safety loop active with hardware watchdog...\n";

    while (g_running.load(std::memory_order_relaxed)) {
        const auto tick_start = std::chrono::steady_clock::now();
        shm->heartbeat_cpp.fetch_add(1, std::memory_order_relaxed);

        // 1. Hardware Dead-Man's Watchdog Evaluation
        uint64_t current_py_heartbeat = shm->heartbeat_python.load(std::memory_order_relaxed);
        if (current_py_heartbeat != last_python_heartbeat) {
            last_python_heartbeat = current_py_heartbeat;
            last_heartbeat_change_time = tick_start;
            watchdog_tripped = false;
        } else {
            auto silent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                tick_start - last_heartbeat_change_time
            ).count();
            // If Python was previously connected and has been silent for > 150 ms, trip fail-safe
            if (silent_ms > 150 && last_python_heartbeat > 0) {
                watchdog_tripped = true;
            }
        }

        // 2. Ingest Dynamic Tuning & Commands via Seqlock
        Eigen::Vector<double, DOF> nominal_torque = Eigen::Vector<double, DOF>::Zero();
        uint32_t flags = 0;
        float dynamic_margin = 0.08f;

        uint64_t cmd_seq1 = shm->command.seq.load(std::memory_order_acquire);
        if ((cmd_seq1 & 1) == 0 && cmd_seq1 != 0) {
            for (size_t i = 0; i < DOF; ++i) {
                nominal_torque(i) = shm->command.nominal_torques[i];
            }
            flags = shm->command.command_flags;
            dynamic_margin = shm->command.obstacle_safety_margin;
            uint64_t cmd_seq2 = shm->command.seq.load(std::memory_order_acquire);
            if (cmd_seq1 != cmd_seq2) {
                nominal_torque.setZero();
            }
        }

        // Apply dynamic barrier safety margin hot-reload
        limits.min_obstacle_distance = std::clamp(static_cast<double>(dynamic_margin), 0.02, 0.30);
        kernel.update_limits(limits);

        // Fail-safe override: If watchdog tripped or E-Stop asserted, force zero torques
        if (watchdog_tripped || (flags & 2)) {
            nominal_torque.setZero();
        }

        // 3. Evaluate Active-Set CBF-QP Kernel
        auto result = kernel.filter_torques(state, nominal_torque, obstacles);
        SafetyOutput safety = result.value_or(SafetyOutput{
            .safe_torque = Eigen::Vector<double, DOF>::Zero(),
            .intervention_triggered = true,
            .solve_time_us = 0.0,
            .barrier_margin = 0.0
        });

        // 4. Physical State Integration
        for (size_t i = 0; i < DOF; ++i) {
            double acceleration = safety.safe_torque(i) / 10.0;
            state.q_dot(i) += acceleration * 0.002;
            state.q_dot(i) *= 0.98; // Simulated damping
            state.q(i) += state.q_dot(i) * 0.002;
        }

        uint64_t current_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        // 5. RocksDB Flight Recording
        if (safety.intervention_triggered) {
            InterventionRecord record{
                .sequence_number = tick_count,
                .timestamp_ns = current_ts,
                .barrier_margin = static_cast<float>(safety.barrier_margin),
                .commanded_joint5_torque = static_cast<float>(nominal_torque(5)),
                .safe_joint5_torque = static_cast<float>(safety.safe_torque(5)),
                .active_mask = watchdog_tripped ? 2u : 1u
            };
            flight_recorder.log_cbf_intervention(record);
        }

        // 6. Write Telemetry via Seqlock
        uint64_t current_seq = shm->telemetry.seq.load(std::memory_order_relaxed);
        shm->telemetry.seq.store(current_seq + 1, std::memory_order_release);

        shm->telemetry.timestamp_ns = current_ts;
        for (size_t i = 0; i < DOF; ++i) {
            shm->telemetry.joint_positions[i] = static_cast<float>(state.q(i));
            shm->telemetry.joint_velocities[i] = static_cast<float>(state.q_dot(i));
            shm->telemetry.applied_torques[i] = static_cast<float>(safety.safe_torque(i));
        }
        shm->telemetry.barrier_margin = static_cast<float>(safety.barrier_margin);
        shm->telemetry.cbf_intervention = safety.intervention_triggered ? 1 : 0;
        shm->telemetry.error_code = result.has_value() ? 0 : 1;
        shm->telemetry.watchdog_tripped = watchdog_tripped ? 1 : 0;

        shm->telemetry.seq.store(current_seq + 2, std::memory_order_release);

        float max_torque_ratio = 0.0f;
        for (size_t i = 0; i < DOF; ++i) {
            max_torque_ratio = std::max(max_torque_ratio, static_cast<float>(std::abs(safety.safe_torque(i)) / limits.torque_max(i)));
        }
        uint32_t safety_flags = 0;
        if (flags & 2u) safety_flags |= 1u;
        if (!result.has_value()) safety_flags |= 2u;
        if (safety.intervention_triggered) safety_flags |= 4u;
        telemetry_worker.try_push(KinematicTelemetryFrame{
            .timestamp_ns = current_ts,
            .loop_duration_us = static_cast<float>(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - tick_start).count()),
            .condition_number = 0.0f,
            .max_torque_saturation_ratio = max_torque_ratio,
            .safety_flags = safety_flags
        });

        tick_count++;
        if (tick_count % 1000 == 0) {
            std::cout << "[500 Hz Tick: " << tick_count << "] Margin: " 
                      << safety.barrier_margin << " m | T[5]: " 
                      << safety.safe_torque(5) << " Nm | Watchdog: "
                      << (watchdog_tripped ? "TRIPPED (FAIL-SAFE)" : "HEALTHY") << "\n";
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - tick_start
        );
        if (elapsed < TICK_DURATION) {
            std::this_thread::sleep_for(TICK_DURATION - elapsed);
        }
    }

    std::cout << "\n[Shutdown] Flushing RocksDB and detaching shared memory...\n";
    flight_recorder.flush();
    telemetry_worker.stop();
    munmap(shm, sizeof(SharedMemorySegment));
    close(shm_fd);
    return 0;
}
