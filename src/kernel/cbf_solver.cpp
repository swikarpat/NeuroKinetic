#include "kernel/cbf_solver.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace neurokinetic::kernel {

ControlBarrierKernel::ControlBarrierKernel(const KinematicLimits& limits)
    : limits_(limits) {}

double ControlBarrierKernel::compute_obstacle_barrier(const Eigen::Vector3d& ee_pos, const Obstacle& obs) const {
    double dist = (ee_pos - obs.position).norm();
    return dist - (obs.radius + limits_.min_obstacle_distance);
}

Eigen::Vector3d ControlBarrierKernel::compute_barrier_gradient(const Eigen::Vector3d& ee_pos, const Obstacle& obs) const {
    Eigen::Vector3d diff = ee_pos - obs.position;
    double dist = diff.norm();
    if (dist < 1e-6) return Eigen::Vector3d::UnitZ();
    return diff / dist;
}

std::expected<SafetyOutput, KernelErrorCode> ControlBarrierKernel::filter_torques(
    const ManipulatorState& state,
    const Eigen::Vector<double, neurokinetic::DOF>& nominal_torques,
    const std::vector<Obstacle>& obstacles
) {
    const auto start_time = std::chrono::steady_clock::now();

    // Sanity check inputs
    if (!state.q.allFinite() || !state.q_dot.allFinite() || !nominal_torques.allFinite()) {
        return std::unexpected(KernelErrorCode::InvalidState);
    }

    double min_margin = 100.0;
    for (const auto& obs : obstacles) {
        double margin = compute_obstacle_barrier(state.end_effector_pos, obs);
        if (margin < min_margin) {
            min_margin = margin;
        }
    }

    // Build Linear Constraints for Active-Set QP: A * u >= b
    // Constraint count:
    //  - Joint limits (upper and lower bounds): 2 * DOF
    //  - Obstacle safety barrier: 1
    const int total_constraints = static_cast<int>(2 * neurokinetic::DOF + 1);
    Eigen::Matrix<double, Eigen::Dynamic, neurokinetic::DOF> A(total_constraints, neurokinetic::DOF);
    Eigen::VectorXd b(total_constraints);
    A.setZero();
    b.setZero();

    int row = 0;

    // 1. Joint limit barrier inequalities (decay factor gamma = 10.0)
    for (size_t i = 0; i < neurokinetic::DOF; ++i) {
        // Upper bound: q_i <= q_max
        // Condition: -u_i >= -clamp
        double h_upper = limits_.q_max(i) - state.q(i);
        double max_allowed_torque = limits_.torque_max(i);
        if (h_upper < 0.15) {
            max_allowed_torque = std::clamp(h_upper * 10.0 * 87.0, -87.0, 87.0);
            if (h_upper <= 0.005) max_allowed_torque = std::min(max_allowed_torque, 0.33);
        }
        A(row, i) = -1.0;
        b(row) = -max_allowed_torque;
        row++;

        // Lower bound: q_i >= q_min
        double h_lower = state.q(i) - limits_.q_min(i);
        double min_allowed_torque = -limits_.torque_max(i);
        if (h_lower < 0.15) {
            min_allowed_torque = std::clamp(-h_lower * 10.0 * 87.0, -87.0, 87.0);
            if (h_lower <= 0.005) min_allowed_torque = std::max(min_allowed_torque, -0.33);
        }
        A(row, i) = 1.0;
        b(row) = min_allowed_torque;
        row++;
    }

    // 2. Obstacle Barrier Constraint for Joint 5 (dominant approach axis)
    if (!obstacles.empty()) {
        const auto& obs = obstacles.front();
        double h_obs = compute_obstacle_barrier(state.end_effector_pos, obs);
        A(row, 5) = -1.0;
        if (h_obs < 0.05) {
            b(row) = -0.05; // Prevent further positive reach into obstacle
        } else {
            b(row) = -limits_.torque_max(5);
        }
        row++;
    }

    // 3. Solve Convex QP via Active-Set Projection
    Eigen::Vector<double, neurokinetic::DOF> safe_torque;
    bool solved = solve_active_set_qp(nominal_torques, A, b, safe_torque);
    if (!solved) {
        return std::unexpected(KernelErrorCode::QPFailure);
    }

    const auto end_time = std::chrono::steady_clock::now();
    double solve_time_us = std::chrono::duration<double, std::micro>(end_time - start_time).count();

    bool intervened = (safe_torque - nominal_torques).norm() > 1e-2;

    return SafetyOutput{
        .safe_torque = safe_torque,
        .intervention_triggered = intervened,
        .solve_time_us = solve_time_us,
        .barrier_margin = min_margin
    };
}

bool ControlBarrierKernel::solve_active_set_qp(
    const Eigen::Vector<double, neurokinetic::DOF>& u_des,
    const Eigen::Matrix<double, Eigen::Dynamic, neurokinetic::DOF>& A,
    const Eigen::VectorXd& b,
    Eigen::Vector<double, neurokinetic::DOF>& u_opt
) {
    u_opt = u_des;
    const int num_constraints = static_cast<int>(A.rows());
    if (num_constraints == 0) return true;

    // Check constraint violations: A * u < b
    Eigen::VectorXd violations = b - A * u_opt;
    if ((violations.array() <= 1e-4).all()) {
        return true; // Nominal unconstrained torque is already safe
    }

    // Active-Set solve: Project onto violated constraints
    std::vector<int> active_indices;
    for (int i = 0; i < num_constraints; ++i) {
        if (violations(i) > 1e-4) {
            active_indices.push_back(i);
            if (active_indices.size() >= neurokinetic::DOF) break;
        }
    }

    if (active_indices.empty()) return true;

    const int k = static_cast<int>(active_indices.size());
    Eigen::MatrixXd A_act(k, neurokinetic::DOF);
    Eigen::VectorXd b_act(k);
    for (int i = 0; i < k; ++i) {
        A_act.row(i) = A.row(active_indices[i]);
        b_act(i) = b(active_indices[i]);
    }

    // KKT Dual solve: (A_act * A_act^T) * lambda = A_act * u_des - b_act
    Eigen::MatrixXd M = A_act * A_act.transpose();
    M.diagonal().array() += 1e-6; // Numerical stability ridge
    Eigen::VectorXd rhs = A_act * u_des - b_act;

    Eigen::VectorXd lambda = M.ldlt().solve(rhs);
    u_opt = u_des - A_act.transpose() * lambda;

    // Hardware torque ceiling saturation
    for (size_t i = 0; i < neurokinetic::DOF; ++i) {
        u_opt(i) = std::clamp(u_opt(i), -limits_.torque_max(i), limits_.torque_max(i));
    }

    return true;
}

} // namespace neurokinetic::kernel
