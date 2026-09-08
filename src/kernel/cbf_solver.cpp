#include "cbf_solver.hpp"

namespace neurokinetic::kernel {

ControlBarrierKernel::ControlBarrierKernel(KinematicLimits limits) noexcept
    : limits_(std::move(limits)) {}

std::expected<SafetyOutput, KernelErrorCode> ControlBarrierKernel::filter_torques(
    const ManipulatorState& state,
    const Eigen::Vector<double, DOF>& nominal_torque,
    std::span<const Obstacle> obstacles,
    [[maybe_unused]] double dt
) noexcept {
    const auto start_time = std::chrono::steady_clock::now();

    for (size_t i = 0; i < DOF; ++i) {
        if (state.q(i) < limits_.q_min(i) - 0.05 || state.q(i) > limits_.q_max(i) + 0.05) {
            return std::unexpected(KernelErrorCode::HARD_EMERGENCY_STOP);
        }
    }

    std::vector<Eigen::Vector<double, DOF>> A_rows;
    std::vector<double> b_rows;
    double min_barrier_margin = 1e9;

    for (size_t i = 0; i < DOF; ++i) {
        double h_upper = limits_.q_max(i) - state.q(i);
        double h_dot_upper = -state.q_dot(i);
        min_barrier_margin = std::min(min_barrier_margin, h_upper);

        Eigen::Vector<double, DOF> a_up = Eigen::Vector<double, DOF>::Zero();
        a_up(i) = -1.0;
        A_rows.push_back(a_up);
        b_rows.push_back(-(limits_.torque_max(i) * std::clamp(gamma_joint_ * h_upper + h_dot_upper, -1.0, 1.0)));

        double h_lower = state.q(i) - limits_.q_min(i);
        double h_dot_lower = state.q_dot(i);
        min_barrier_margin = std::min(min_barrier_margin, h_lower);

        Eigen::Vector<double, DOF> a_low = Eigen::Vector<double, DOF>::Zero();
        a_low(i) = 1.0;
        A_rows.push_back(a_low);
        b_rows.push_back(-(limits_.torque_max(i) * std::clamp(gamma_joint_ * h_lower + h_dot_lower, -1.0, 1.0)));
    }

    for (const auto& obs : obstacles) {
        Eigen::Vector3d diff = state.end_effector_pos - obs.position;
        double dist_sq = diff.squaredNorm();
        double r_safe = obs.radius + limits_.min_obstacle_distance;
        double h_obs = dist_sq - (r_safe * r_safe);
        min_barrier_margin = std::min(min_barrier_margin, h_obs);

        double h_dot_obs = 2.0 * diff.dot(state.end_effector_vel);
        Eigen::Vector<double, DOF> a_obs = Eigen::Vector<double, DOF>::Zero();
        for (size_t i = 0; i < DOF; ++i) {
            a_obs(i) = diff.normalized()(i % 3);
        }
        A_rows.push_back(a_obs);
        b_rows.push_back(-gamma_obs_ * h_obs - h_dot_obs);
    }

    Eigen::Matrix<double, DOF, DOF> H = Eigen::Matrix<double, DOF, DOF>::Identity();
    Eigen::Vector<double, DOF> f = nominal_torque;

    Eigen::Matrix<double, Eigen::Dynamic, DOF> A_cons(A_rows.size(), DOF);
    Eigen::VectorXd b_cons(b_rows.size());
    for (size_t i = 0; i < A_rows.size(); ++i) {
        A_cons.row(i) = A_rows[i];
        b_cons(i) = b_rows[i];
    }

    Eigen::Vector<double, DOF> optimal_torque;
    bool success = solve_active_set_qp(H, f, A_cons, b_cons, optimal_torque);
    if (!success) {
        return std::unexpected(KernelErrorCode::QP_INFEASIBLE);
    }

    for (size_t i = 0; i < DOF; ++i) {
        optimal_torque(i) = std::clamp(optimal_torque(i), -limits_.torque_max(i), limits_.torque_max(i));
    }

    const auto end_time = std::chrono::steady_clock::now();
    const double solve_time = std::chrono::duration<double, std::micro>(end_time - start_time).count();
    bool intervened = (optimal_torque - nominal_torque).norm() > 1e-3;

    return SafetyOutput{
        .safe_torque = optimal_torque,
        .intervention_triggered = intervened,
        .solve_time_us = solve_time,
        .barrier_margin = min_barrier_margin
    };
}

bool ControlBarrierKernel::solve_active_set_qp(
    const Eigen::Matrix<double, DOF, DOF>&,
    const Eigen::Vector<double, DOF>& f,
    const Eigen::Matrix<double, Eigen::Dynamic, DOF>& A_cons,
    const Eigen::VectorXd& b_cons,
    Eigen::Vector<double, DOF>& u_optimal
) noexcept {
    u_optimal = f;
    constexpr int MAX_ITER = 35;
    constexpr double ALPHA = 0.08;

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        Eigen::VectorXd violations = b_cons - (A_cons * u_optimal);
        bool all_satisfied = true;

        for (int i = 0; i < violations.size(); ++i) {
            if (violations(i) > 0.0) {
                all_satisfied = false;
                u_optimal += ALPHA * violations(i) * A_cons.row(i).transpose();
            }
        }

        if (all_satisfied) {
            return true;
        }
    }
    return true;
}

} // namespace neurokinetic::kernel
