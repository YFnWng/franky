#pragma once

#include <optional>

#include "franky/motion/joint_impedance_base.hpp"
#include "franky/wait_free_triple_buffer.hpp"

namespace franky {

using JointVelocityHandle = WaitFreeTripleBuffer<Vector7d>;

/**
 * Explicit velocity-reference joint impedance controller.
 *
 * A user thread publishes dq_ref. The 1 kHz control thread integrates q_ref
 * with the measured libfranka time step, projects q_ref at the configured soft
 * limits, and evaluates the ordinary joint impedance law using both q_ref and
 * dq_ref. No Python code runs in the real-time integration path.
 */
class JointVelocityImpedanceTrackingMotion : public JointImpedanceBase {
 public:
  using Params = JointImpedanceParams;

  explicit JointVelocityImpedanceTrackingMotion(
      const Params &params = Params{}, double gains_time_constant = 0.1);

  void setVelocity(const Vector7d &velocity) {
    validateFinite(velocity, "velocity");
    velocity_handle_.set(velocity);
  }

  [[nodiscard]] Vector7d getVelocity() const { return velocity_handle_.getLastWritten(); }

  /** Set the integration origin before the motion starts. */
  void setInitialReference(const Vector7d &reference);

  /** Latest reference actually evaluated by the 1 kHz controller. */
  [[nodiscard]] std::optional<JointReference> getAppliedReference() {
    return applied_reference_handle_.get();
  }

 protected:
  void initImpl(const RobotState &robot_state, const std::optional<franka::Torques> &previous_command) override;
  franka::Torques nextCommandImpl(
      const RobotState &robot_state, franka::Duration time_step, franka::Duration rel_time,
      franka::Duration abs_time, const std::optional<franka::Torques> &previous_command) override;

 private:
  JointVelocityHandle velocity_handle_{Vector7d::Zero()};
  WaitFreeTripleBuffer<std::optional<Vector7d>> initial_reference_handle_{std::nullopt};
  WaitFreeTripleBuffer<std::optional<JointReference>> applied_reference_handle_{std::nullopt};
};

}  // namespace franky
