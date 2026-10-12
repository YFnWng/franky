#include "franky/motion/joint_velocity_impedance_tracking_motion.hpp"

#include <algorithm>
#include <stdexcept>

namespace franky {

JointVelocityImpedanceTrackingMotion::JointVelocityImpedanceTrackingMotion(
    const Params &params, double gains_time_constant)
    : JointImpedanceBase(Vector7d::Zero(), Vector7d::Zero(), params, gains_time_constant) {
  if (!params.safety.lower_joint_limits.has_value() || !params.safety.upper_joint_limits.has_value()) {
    throw std::invalid_argument(
        "JointVelocityImpedanceTrackingMotion requires lower_joint_limits and upper_joint_limits");
  }
}

void JointVelocityImpedanceTrackingMotion::setInitialReference(const Vector7d &reference) {
  if (has_started()) throw std::logic_error("initial reference can only be set before motion start");
  validateFinite(reference, "initial_reference");
  const auto &lower = *params().safety.lower_joint_limits;
  const auto &upper = *params().safety.upper_joint_limits;
  if ((reference.array() < lower.array()).any() || (reference.array() > upper.array()).any()) {
    throw std::invalid_argument("initial_reference exceeds configured joint limits");
  }
  initial_reference_handle_.set(reference);
}

void JointVelocityImpedanceTrackingMotion::initImpl(
    const RobotState &robot_state, const std::optional<franka::Torques> &previous_command) {
  (void)previous_command;
  const auto initial = initial_reference_handle_.getUnsafe();
  target_ = initial.value_or(robot_state.q);
  target_velocity_.setZero();
  JointReference reference{target_, target_velocity_, Vector7d::Zero()};
  applied_reference_handle_.setUnsafe(reference);
}

franka::Torques JointVelocityImpedanceTrackingMotion::nextCommandImpl(
    const RobotState &robot_state, franka::Duration time_step, franka::Duration rel_time, franka::Duration abs_time,
    const std::optional<franka::Torques> &previous_command) {
  (void)rel_time;
  (void)abs_time;
  (void)previous_command;
  const double dt = time_step.toSec();
  target_velocity_ = velocity_handle_.getUnsafe();
  const auto &lower = *params().safety.lower_joint_limits;
  const auto &upper = *params().safety.upper_joint_limits;
  // libfranka supplies a zero-period first control callback. Publish the seeded
  // reference and compute the first bounded torque command without integrating.
  if (dt > 0.0) {
    for (Eigen::Index i = 0; i < target_.size(); ++i) {
      const double candidate = target_[i] + target_velocity_[i] * dt;
      target_[i] = std::clamp(candidate, lower[i], upper[i]);
      if ((target_[i] <= lower[i] && target_velocity_[i] < 0.0) ||
          (target_[i] >= upper[i] && target_velocity_[i] > 0.0)) {
        target_velocity_[i] = 0.0;
      }
    }
  }

  JointReference reference{target_, target_velocity_, Vector7d::Zero()};
  applied_reference_handle_.setUnsafe(reference);
  return computeCommand(robot_state, reference, dt);
}

}  // namespace franky
