#pragma once

#include <Eigen/Dense>
#include <vector>
#include <expected>
#include <cstdint>

namespace neurokinetic {
inline constexpr size_t DOF = 7;
}

namespace neurokinetic::kernel {

enum class KernelErrorCode : uint32_t {
    None = 0,
    QPFailure = 1,
    InvalidState = 2,
    BoundaryViolation = 3
};

struct KinematicLimits {
    Eigen::Vector<double, neurokinetic::DOF> q_min;
    Eigen::Vector<double, neurokinetic::DOF> q_max;
    Eigen::Vector<double, neurokinetic::DOF> q_dot_max;
    Eigen::Vector<double, neurokinetic::DOF> torque_max;
    double min_obstacle_distance{0.08};
};

struct ManipulatorState {
    Eigen::Vector<double, neurokinetic::DOF> q;
    Eigen::Vector<double, neurokinetic::DOF> q_dot;
    Eigen::Vector3d end_effector_pos;
    Eigen::Vector3d end_effector_vel;
};

struct Obstacle {
    Eigen::Vector3d position;
    double radius;
};

struct SafetyOutput {
    Eigen::Vector<double, neurokinetic::DOF> safe_torque;
    bool intervention_triggered{false};
    double solve_time_us{0.0};
    double barrier_margin{0.0};
};

class ControlBarrierKernel {
public:
    explicit ControlBarrierKernel(const KinematicLimits& limits);
    ~ControlBarrierKernel() = default;

    std::expected<SafetyOutput, KernelErrorCode> filter_torques(
        const ManipulatorState& state,
        const Eigen::Vector<double, neurokinetic::DOF>& nominal_torques,
        const std::vector<Obstacle>& obstacles
    );

    void update_limits(const KinematicLimits& limits) noexcept {
        limits_ = limits;
    }

    const KinematicLimits& get_limits() const noexcept {
        return limits_;
    }

private:
    KinematicLimits limits_;

    double compute_obstacle_barrier(const Eigen::Vector3d& ee_pos, const Obstacle& obs) const;
    Eigen::Vector3d compute_barrier_gradient(const Eigen::Vector3d& ee_pos, const Obstacle& obs) const;

    bool solve_active_set_qp(
        const Eigen::Vector<double, neurokinetic::DOF>& u_des,
        const Eigen::Matrix<double, Eigen::Dynamic, neurokinetic::DOF>& A,
        const Eigen::VectorXd& b,
        Eigen::Vector<double, neurokinetic::DOF>& u_opt
    );
};

} // namespace neurokinetic::kernel
