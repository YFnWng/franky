#pragma once
#include <atomic>
#include <chrono>
#include <vector>
#include "franky/motion/cartesian_curve_impedance_motion.hpp"
#include "franky/motion/stop_motion.hpp"

namespace franky {
// Limits are explicit: no hardware-authorized defaults are implied.
struct M4RuntimeLimits {
  Vector7d joint_lower{Vector7d::Zero()};
  Vector7d joint_upper{Vector7d::Zero()};
  Vector7d joint_velocity{Vector7d::Zero()};
  Vector7d external_torque{Vector7d::Zero()};
  Vector7d command_torque{Vector7d::Zero()};
  Eigen::Vector3d workspace_lower{Eigen::Vector3d::Zero()}, workspace_upper{Eigen::Vector3d::Zero()};
  double reference_timeout_s{0.1}, callback_gap_s{0.01}, position_error_m{0.03};
  double orientation_error_rad{0.2}, linear_speed_m_s{0.1}, angular_speed_rad_s{0.5};
  double minimum_jacobian_singular_value{0.02}, minimum_command_success_rate{0.95};
  bool abort_on_contact{true};
  void validate() const;
};
struct M4DlsParams {
  Vector7d stiffness{Vector7d::Constant(100)}, damping{Vector7d::Constant(20)};
  Vector7d reference_error_limit{Vector7d::Constant(.15)};
  Vector6d pose_gain{Vector6d::Constant(2)}, task_scale{Vector6d::Ones()};
  double lambda{.01}, rank_tolerance{1e-6}, posture_gain{.5};
  void validate() const;
};
struct M4DlsStep {
  Vector7d velocity{Vector7d::Zero()}, secondary_velocity{Vector7d::Zero()};
  double velocity_scale{1};
  int rank{0};
};
bool validM4DlsPeriod(double dt_s, uint64_t tick, double maximum_s);
M4DlsStep computeM4DlsStep(const Jacobian &, const Vector6d &pose_error,
                          const Vector6d &desired_twist, const Vector7d &posture_error,
                          const Vector7d &velocity_limit, const M4DlsParams &);
struct M4Reference {
  CartesianReference cartesian;
  Vector7d posture{Vector7d::Zero()};
  bool has_posture{false};
  double posture_transition_s{1.0};
  uint64_t sequence{0};
  int64_t published_ns{0};
};
struct M4RuntimeSample {
  uint64_t tick{0}, reference_sequence{0};
  int64_t monotonic_ns{0};
  double robot_time_s{0}, dt_s{0}, reference_age_s{0}, min_singular_value{0};
  Vector7d q{Vector7d::Zero()};
  Vector7d dq{Vector7d::Zero()};
  Vector7d torque_measured{Vector7d::Zero()};
  Vector7d torque_previous{Vector7d::Zero()};
  Vector7d external_torque{Vector7d::Zero()};
  Affine pose{Affine::Identity()}, reference_pose{Affine::Identity()};
  Vector6d twist{Vector6d::Zero()}, reference_twist{Vector6d::Zero()};
  Vector7d posture{Vector7d::Zero()}, raw{Vector7d::Zero()}, task{Vector7d::Zero()};
  Vector7d nullspace{Vector7d::Zero()}, limit{Vector7d::Zero()}, coriolis{Vector7d::Zero()};
  Vector7d slew{Vector7d::Zero()}, filtered_prediction{Vector7d::Zero()};
  Vector7d joint_reference{Vector7d::Zero()}, joint_velocity_reference{Vector7d::Zero()};
  Vector7d secondary_velocity{Vector7d::Zero()};
  Vector6d outer_correction{Vector6d::Zero()};
  double dls_velocity_scale{1};
  int dls_rank{0};
  bool dls_active{false};
  double command_success_rate{0};
  int fault{0}, stop_fault{0};
  bool stopping{false}, stop_finished{false}, stop_at_rest{false};
};
// Fault IDs: 1 timeout, 2 sequence, 3 callback, 4 nonfinite, 5 joint bounds,
// 6 joint velocity, 7 workspace, 8 pose error, 9 speed, 10 external torque,
// 11 contact/collision, 12 singularity, 13 robot error, 14 command success,
// 15 command torque, 16 native logger overflow, 17 controller/projection, 18 stop timeout.
int evaluateM4Limits(const M4RuntimeLimits &, const M4RuntimeSample &, bool contact, bool robot_error);
int evaluateM4StopLimits(const M4RuntimeLimits &, const M4RuntimeSample &, const Affine &anchor);

// Deterministic servo-time stop completion: a transient low speed is insufficient.
struct M4StopProgress {
  double rest_since{-1};
  // 0 continuing, 1 sustained rest, 18 deadline without sustained rest.
  int update(double elapsed, const Vector7d &dq, const TorqueStopParams &params);
};
Vector7d blendM4StopTorque(const Vector7d &initial, const Vector7d &hold,
                         double elapsed, const TorqueStopParams &params,
                         const Vector7d &previous, const Vector7d &bounds);
Vector6d computeM4PoseHoldWrench(const Matrix6d &stiffness, const Matrix6d &damping,
                               const Vector6d &error, const Vector6d &twist);

class M4RuntimeMotion : public CartesianImpedanceBase {
 public:
  M4RuntimeMotion(const Params &, const M4RuntimeLimits &, const TorqueStopParams &,
                  std::optional<CurveFixtureParams> fixture = std::nullopt, size_t log_capacity = 4096,
                  std::optional<M4DlsParams> dls = std::nullopt,
                  std::optional<Matrix6d> stop_stiffness = std::nullopt,
                  std::optional<Matrix6d> stop_damping = std::nullopt);
  void publish(M4Reference reference);
  void requestStop() { stop_requested_.store(true); }
  int fault() const { return fault_.load(); }
  int stopFault() const { return stop_fault_.load(); }
  static int stopBehaviorVersion() { return 2; }
  uint64_t droppedSamples() const { return dropped_.load(); }
  std::vector<M4RuntimeSample> drain(); // One non-real-time consumer only.
 protected:
  void initImpl(const RobotState &, const std::optional<franka::Torques> &) override;
  franka::Torques nextCommandImpl(const RobotState &, franka::Duration, franka::Duration,
      franka::Duration, const std::optional<franka::Torques> &) override;
  Vector6d computeCartesianWrench(const RobotState &, const CartesianReference &, const Vector6d &,
      const Vector6d &, const Vector6d &) override;
  Vector7d postureTarget(const Vector7d &) override { return applied_posture_; }
  void observeTorque(const Vector7d &, const Vector7d &, const Vector7d &, const Vector7d &, const Vector7d &) override;
 private:
  std::optional<Matrix6d> stop_stiffness_, stop_damping_;
  std::optional<M4DlsParams> dls_;
  Vector7d integrated_reference_{Vector7d::Zero()};
  void latch(int code);
  bool push(const M4RuntimeSample &);
  M4RuntimeLimits limits_;
  TorqueStopParams stop_params_;
  Affine stop_pose_{Affine::Identity()};
  Vector7d stop_initial_torque_{Vector7d::Zero()};
  M4StopProgress stop_progress_;
  std::optional<CurveFixtureParams> fixture_;
  WaitFreeTripleBuffer<M4Reference> references_;
  M4Reference current_;
  Vector7d applied_posture_, posture_start_, posture_goal_;
  double posture_start_s_{0}, elapsed_s_{0}, stop_start_s_{0};
  uint64_t tick_{0};
  int64_t last_tick_ns_{0};
  bool stopping_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<int> fault_{0};
  std::atomic<int> stop_fault_{0};
  std::vector<M4RuntimeSample> log_;
  std::atomic<size_t> read_{0}, write_{0};
  std::atomic<uint64_t> dropped_{0};
  M4RuntimeSample sample_;
};
} // namespace franky
