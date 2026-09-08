#include "kernel/cbf_solver.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

using namespace neurokinetic;
using namespace neurokinetic::kernel;

int main() {
    std::cout << "========================================================\n";
    std::cout << "  NeuroKinetic C++23 Control Barrier Function Benchmark \n";
    std::cout << "========================================================\n";

    KinematicLimits limits{
        .q_min = Eigen::Vector<double, DOF>::Constant(-2.89),
        .q_max = Eigen::Vector<double, DOF>::Constant(2.89),
        .q_dot_max = Eigen::Vector<double, DOF>::Constant(2.17),
        .torque_max = Eigen::Vector<double, DOF>::Constant(87.0),
        .min_obstacle_distance = 0.08
    };

    ControlBarrierKernel kernel(limits);

    // Simulated State: Arm nearing joint upper bound & obstacle
    ManipulatorState state{
        .q = (Eigen::Vector<double, DOF>() << 0.1, 0.2, 0.0, -1.5, 0.0, 2.82, 0.0).finished(),
        .q_dot = (Eigen::Vector<double, DOF>() << 0.0, 0.1, 0.0, 0.2, 0.0, 1.8, 0.0).finished(),
        .end_effector_pos = Eigen::Vector3d(0.45, 0.10, 0.32),
        .end_effector_vel = Eigen::Vector3d(0.80, 0.00, -0.20)
    };

    Eigen::Vector<double, DOF> hazardous_torque = 
        (Eigen::Vector<double, DOF>() << 10.0, 5.0, 0.0, 15.0, 2.0, 45.0, 5.0).finished();

    std::vector<Obstacle> obstacles = {
        Obstacle{.position = Eigen::Vector3d(0.50, 0.10, 0.30), .radius = 0.05}
    };

    constexpr size_t RUNS = 10000;
    std::vector<double> latencies;
    latencies.reserve(RUNS);

    SafetyOutput last_output;

    for (size_t i = 0; i < RUNS; ++i) {
        auto res = kernel.filter_torques(state, hazardous_torque, obstacles);
        if (!res.has_value()) {
            std::cerr << "Kernel returned error code: " << static_cast<int>(res.error()) << "\n";
            return 1;
        }
        last_output = *res;
        latencies.push_back(last_output.solve_time_us);
    }

    std::sort(latencies.begin(), latencies.end());
    double p50 = latencies[RUNS * 0.50];
    double p99 = latencies[RUNS * 0.99];
    double max_lat = latencies.back();

    std::cout << "\n[Execution Verification Results]\n";
    std::cout << " • Nominal Commanded Torque Joint 5: " << hazardous_torque(5) << " Nm\n";
    std::cout << " • Safe Overridden Torque Joint 5:   " << last_output.safe_torque(5) << " Nm\n";
    std::cout << " • Active CBF Intervention:         " << (last_output.intervention_triggered ? "YES (CLAMPED)" : "NO") << "\n";
    std::cout << " • Barrier Distance Margin:         " << last_output.barrier_margin << " m\n\n";

    std::cout << "[Deterministic 500 Hz Timing Profiles (" << RUNS << " ticks)]\n";
    std::cout << " • P50 Latency: " << p50 << " µs\n";
    std::cout << " • P99 Latency: " << p99 << " µs  (Target SLA: < 800 µs)\n";
    std::cout << " • Max Latency: " << max_lat << " µs\n";
    std::cout << "========================================================\n";

    return 0;
}
