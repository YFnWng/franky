#pragma once

#include <cstdint>
#include <optional>
#include "franky/types.hpp"

namespace franky {
// Pure time-injected state machine shared by the native motion and offline tests.
// No commissioned defaults: every positive threshold must be supplied explicitly.
enum class DroidFault { None=0, PublisherTimeout=1, Sequence=2, CallbackGap=3,
  Nonfinite=4, JointBounds=5, JointVelocity=6, CommandTorque=7,
  RobotError=8, Contact=9, LogOverflow=10, StopTimeout=11, StopState=12 };
struct DroidJointLimits {
  Vector7d lower{Vector7d::Zero()}, upper{Vector7d::Zero()};
  Vector7d velocity{Vector7d::Zero()}, command_torque{Vector7d::Zero()};
  int64_t publisher_timeout_ns{0}, callback_gap_ns{0};
  void validate() const;
};
struct DroidJointPublication {
  Vector7d q{Vector7d::Zero()};
  uint64_t publication_sequence{0}, row_sequence{0};
  int64_t published_ns{0}, expires_ns{0};
};
struct DroidConsumedReference {
  uint64_t row_sequence{0}, publication_sequence{0}, tick{0};
  int64_t monotonic_ns{0};
};
class DroidJointRuntime {
 public:
  explicit DroidJointRuntime(const DroidJointLimits &limits) : limits(limits) { limits.validate(); }
  void advance(int64_t now, double dt, const Vector7d &q, const Vector7d &dq,
               const Vector7d &previous, const std::optional<DroidJointPublication> &publication,
               bool robot_error=false, bool contact=false, bool stopping=false);
  void checkCommand(const Vector7d &raw);
  // Call ONLY after computing/checking the actual command for this callback.
  void consumed(int64_t now);
  void latch(DroidFault code) { if (fault==DroidFault::None) fault=code; }
  DroidJointLimits limits;
  DroidFault fault{DroidFault::None};
  uint64_t tick{0};
  int64_t last_tick_ns{0};
  std::optional<DroidJointPublication> active;
  DroidConsumedReference receipt;
};
}  // namespace franky
