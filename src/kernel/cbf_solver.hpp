#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <expected>
#include <span>
#include <string_view>
#include <vector>

namespace neurokinetic {

constexpr size_t DOF = 7; // 7-DOF Manipulator (Franka Emika / KUKA iiwa / Humanoid Arm)

namespace kernel {

enum class KernelErrorCode : uint8_t {
    OK = 0,
    QP_INFEASIBLE = 1,
    KINEMATIC_SINGULARITY = 2,
    HARD_EMERGENCY_STOP = 3
};

struct ManipulatorState {
    Eigen::Vector<double, DOF> q;       // Joint angles (rad)
    Eigen::Vector<double, DOF> q_dot;   // Joint velocities (rad/s)
    Eigen::Vector3d end_effector_pos;   // Cartesian position [X, Y, Z] (m)
    Eigen::Vector3d end_effector_vel;   // Cartesian velocity [Vx, Vy, Vz] (m/s)
};

struct KinematicLimits {
    Eigen::Vector<double, DOF> q_min;
    Eigen::Vector<double, DOF> q_max;
    Eigen::Vector<double, DOF> q_dot_max;
    Eigen::Vector<double, DOF> torque_max;
    double min_obstacle_distance = 0.05; // 5 cm safety bubble
};

struct Obstacle {
    Eigen::Vector3d position;
    double radius;
};

struct SafetyOutput {
    Eigen::Vector<double, DOF> safe_torque;
    bool intervention_triggered;
    double solve_time_us;
    double barrier_margin;
};

class alignas(64) ControlBarrierKernel {
public:
    explicit ControlBarrierKernel(KinematicLimits limits) noexcept;

    // Fast-path 500 Hz tick: Filters nominal policy torque through CBF-QP
    [[nodiscard]] std::expected<SafetyOutput, KernelErrorCode> filter_torques(
        const ManipulatorState& state,
        const Eigen::Vector<double, DOF>& nominal_torque,
        std::span<const Obstacle> obstacles,
        double dt = 0.002
    ) noexcept;

private:
    KinematicLimits limits_;
    double gamma_joint_ = 10.0;
    double gamma_obs_ = 15.0;

    bool solve_active_set_qp(
        const Eigen::Matrix<double, DOF, DOF>& H,
        const Eigen::Vector<double, DOF>& f,
        const Eigen::Matrix<double, Eigen::Dynamic, DOF>& A_cons,
        const Eigen::VectorXd& b_cons,
        Eigen::Vector<double, DOF>& u_optimal
    ) noexcept;
};

} // namespace kernel
} // namespace neurokinetic
