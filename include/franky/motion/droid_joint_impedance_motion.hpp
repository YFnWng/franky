#pragma once
#include <atomic>
#include <mutex>
#include <vector>
#include "franky/motion/droid_joint_runtime.hpp"
#include "franky/motion/stop_motion.hpp"

namespace franky {
struct DroidJointSample {
  uint64_t tick{0}, publication_sequence{0}, dropped{0};
  int64_t monotonic_ns{0};
  DroidConsumedReference consumed;
  Vector7d q{Vector7d::Zero()}, dq{Vector7d::Zero()}, reference{Vector7d::Zero()};
  Vector7d tracking_raw{Vector7d::Zero()}, raw{Vector7d::Zero()}, outgoing{Vector7d::Zero()};
  int fault{0}, stop_fault{0};
  bool stopping{false}, stop_finished{false}, stop_at_rest{false};
};
// Separate candidate; does not change the existing position/PPO tracking API.
// Clock is CLOCK_MONOTONIC (not RAW). A consumed receipt is callback evidence,
// never a robot acknowledgement that the command reached the actuators.
class DroidJointImpedanceMotion : public JointImpedanceBase {
 public:
  DroidJointImpedanceMotion(const JointImpedanceParams &, const DroidJointLimits &,
                           const TorqueStopParams &, size_t log_capacity, double gains_time_constant);
  void publish(const Vector7d &q, uint64_t row_sequence, int64_t expires_ns);
  void requestStop() { stop_requested_.store(true); }
  DroidJointSample status() { return status_.get(); }
  std::vector<DroidJointSample> drain(); // Single non-RT consumer.
 protected:
  virtual int64_t clockNs() const; // CLOCK_MONOTONIC; injectable only in C++ offline tests.
  virtual franka::Torques trackingCommand(const RobotState &s, const JointReference &r, double dt) {
    return computeCommand(s,r,dt);
  }
  void initImpl(const RobotState &, const std::optional<franka::Torques> &) override;
  franka::Torques nextCommandImpl(const RobotState &, franka::Duration, franka::Duration,
      franka::Duration, const std::optional<franka::Torques> &) override;
  void observeRawJointTorque(const Vector7d &raw) override { raw_=raw; tracking_raw_=raw; runtime_.checkCommand(raw); }
 private:
  DroidJointRuntime runtime_;
  TorqueStopParams stop_;
  WaitFreeTripleBuffer<DroidJointPublication> mailbox_;
  WaitFreeTripleBuffer<DroidJointSample> status_;
  std::mutex publisher_mutex_;
  uint64_t published_{0}, last_row_{0};
  int64_t last_expiry_{0}, last_callback_ns_{0};
  Vector7d last_q_{Vector7d::Zero()}, initial_stop_{Vector7d::Zero()}, raw_{Vector7d::Zero()}, tracking_raw_{Vector7d::Zero()};
  std::atomic<bool> stop_requested_{false};
  bool initialized_{false}, stopping_{false}, finished_{false}, at_rest_{false};
  int stop_fault_{0};
  int64_t stop_start_{0}, rest_since_{0};
  std::vector<DroidJointSample> log_;
  std::atomic<size_t> read_{0}, write_{0};
  uint64_t dropped_{0};
};
}  // namespace franky
