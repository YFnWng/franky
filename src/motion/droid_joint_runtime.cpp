#include "franky/motion/droid_joint_runtime.hpp"
#include <cmath>
#include <stdexcept>
namespace franky {
void DroidJointLimits::validate() const {
  if (!lower.allFinite() || !upper.allFinite() || (lower.array()>=upper.array()).any() ||
      !velocity.allFinite() || (velocity.array()<=0).any() || !command_torque.allFinite() ||
      (command_torque.array()<=0).any() || publisher_timeout_ns<=0 || callback_gap_ns<=0)
    throw std::invalid_argument("Explicit finite DROID joint limits and watchdog periods required");
}
void DroidJointRuntime::advance(int64_t now, double dt, const Vector7d &q, const Vector7d &dq,
                               const Vector7d &previous, const std::optional<DroidJointPublication> &p,
                               bool robot_error, bool contact, bool stopping) {
  ++tick;
  if (fault!=DroidFault::None) return;
  if (now<=0 || !std::isfinite(dt) || dt<0 || (tick>1 && (dt==0 || now<=last_tick_ns)) ||
      dt>limits.callback_gap_ns*1e-9 || (last_tick_ns && now-last_tick_ns>limits.callback_gap_ns))
    latch(DroidFault::CallbackGap);
  last_tick_ns=now;
  if (!q.allFinite() || !dq.allFinite() || !previous.allFinite()) latch(DroidFault::Nonfinite);
  if (robot_error) latch(DroidFault::RobotError);
  if (contact) latch(DroidFault::Contact);
  if ((q.array()<limits.lower.array()).any() || (q.array()>limits.upper.array()).any()) latch(DroidFault::JointBounds);
  if ((dq.cwiseAbs().array()>limits.velocity.array()).any()) latch(DroidFault::JointVelocity);
  if ((previous.cwiseAbs().array()>limits.command_torque.array()).any()) latch(DroidFault::CommandTorque);
  if (stopping) return; // Producer expiry cannot turn an orderly stop into a new tracking fault.
  if (p) {
    if (!p->q.allFinite()) latch(DroidFault::Nonfinite);
    if (!p->publication_sequence || !p->row_sequence ||
        (active && (p->publication_sequence<=active->publication_sequence || p->row_sequence<active->row_sequence ||
         (p->row_sequence==active->row_sequence && ((p->q.array()!=active->q.array()).any() || p->expires_ns!=active->expires_ns)))))
      latch(DroidFault::Sequence);
    if ((p->q.array()<limits.lower.array()).any() || (p->q.array()>limits.upper.array()).any()) latch(DroidFault::JointBounds);
    if (p->published_ns<=0 || p->published_ns>now || p->expires_ns<=p->published_ns)
      latch(DroidFault::PublisherTimeout);
    if (fault==DroidFault::None) active=p;
  }
  // A native callback NEVER refreshes these producer timestamps.
  if (!active || now<active->published_ns || now-active->published_ns>=limits.publisher_timeout_ns || now>=active->expires_ns)
    latch(DroidFault::PublisherTimeout);
}
void DroidJointRuntime::checkCommand(const Vector7d &raw) {
  if (!raw.allFinite()) latch(DroidFault::Nonfinite);
  else if ((raw.cwiseAbs().array()>limits.command_torque.array()).any()) latch(DroidFault::CommandTorque);
}
void DroidJointRuntime::consumed(int64_t now) {
  if (fault==DroidFault::None && active && receipt.row_sequence!=active->row_sequence)
    receipt={active->row_sequence,active->publication_sequence,tick,now};
}
}  // namespace franky
