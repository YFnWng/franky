#include "franky/motion/droid_joint_impedance_motion.hpp"
#include "franky/model.hpp"
#include <time.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace franky {
namespace {
int64_t monotonicNs() {
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC,&ts)!=0) throw std::runtime_error("CLOCK_MONOTONIC failed");
  return static_cast<int64_t>(ts.tv_sec)*1000000000LL+ts.tv_nsec;
}
size_t checkedCapacity(size_t n) {
  if(n<2 || n>1000000) throw std::invalid_argument("Invalid DROID log capacity");
  return n;
}
franka::Torques torques(const Vector7d &x) {
  std::array<double,7> a{}; Eigen::Map<Vector7d>(a.data())=x;return franka::Torques(a);
}
}
DroidJointImpedanceMotion::DroidJointImpedanceMotion(const JointImpedanceParams &p,
    const DroidJointLimits &l, const TorqueStopParams &s, size_t capacity, double gain_tau)
    : JointImpedanceBase(Vector7d::Zero(),Vector7d::Zero(),p,gain_tau), runtime_(l),stop_(s),log_(checkedCapacity(capacity)) {
  if (!p.damping || !p.cartesian_gains || !p.cartesian_gains->damping)
    throw std::invalid_argument("DROID requires explicit joint/Cartesian gains and damping");
  if (p.safety.max_delta_tau<=0 || capacity<2 || capacity>1000000 || !s.damping.allFinite() || (s.damping.array()<=0).any() ||
      !std::isfinite(s.ramp_duration) || s.ramp_duration<0 || !std::isfinite(s.max_duration) ||
      s.max_duration<=s.ramp_duration+.1 || !std::isfinite(s.velocity_epsilon) || s.velocity_epsilon<=0 ||
      !std::isfinite(s.max_delta_tau) || s.max_delta_tau<=0)
    throw std::invalid_argument("Invalid explicit DROID stop/log parameters");
}
int64_t DroidJointImpedanceMotion::clockNs() const {return monotonicNs();}
void DroidJointImpedanceMotion::publish(const Vector7d &q, uint64_t row, int64_t expires) {
  std::lock_guard<std::mutex> lock(publisher_mutex_);
  const auto now=clockNs();
  if (!q.allFinite() || (q.array()<runtime_.limits.lower.array()).any() ||
      (q.array()>runtime_.limits.upper.array()).any() || row==0 || row<last_row_ ||
      (row==last_row_ && ((q.array()!=last_q_.array()).any() || expires!=last_expiry_)) || expires<=now || published_==UINT64_MAX)
    throw std::invalid_argument("Invalid DROID publication/row/deadline");
  mailbox_.set({q,++published_,row,now,expires});last_row_=row;last_q_=q;last_expiry_=expires;
}
void DroidJointImpedanceMotion::initImpl(const RobotState &state, const std::optional<franka::Torques> &) {
  if (initialized_) throw std::logic_error("DROID motion is single-session; construct a fresh motion");
  initialized_=true;target_=state.q;target_velocity_.setZero();
}
std::vector<DroidJointSample> DroidJointImpedanceMotion::drain() {
  std::vector<DroidJointSample> result;
  auto r=read_.load(std::memory_order_relaxed);const auto w=write_.load(std::memory_order_acquire);
  while(r!=w) {result.push_back(log_[r]);r=(r+1)%log_.size();}
  read_.store(r,std::memory_order_release);return result;
}
franka::Torques DroidJointImpedanceMotion::nextCommandImpl(const RobotState &s, franka::Duration dt,
    franka::Duration, franka::Duration, const std::optional<franka::Torques> &) {
  if (finished_) throw std::logic_error("DROID callback after completion");
  std::optional<DroidJointPublication> p;
  if (mailbox_.hasNewData()) p=mailbox_.getUnsafe();
  // Sample after acquiring the mailbox: a concurrent producer may publish after
  // callback entry. Its timestamp must not appear to be in the future.
  const int64_t now=clockNs();
  const bool contact=(s.joint_collision.array()>0).any() || (s.cartesian_collision.array()>0).any() ||
                     (s.joint_contact.array()>0).any() || (s.cartesian_contact.array()>0).any();
  runtime_.advance(now,dt.toSec(),s.q,s.dq,s.tau_J_d,p,bool(s.current_errors),contact,stopping_ || stop_requested_.load());
  const auto w=write_.load(std::memory_order_relaxed),next=(w+1)%log_.size();
  const bool full=next==read_.load(std::memory_order_acquire);
  if (full) {++dropped_;runtime_.latch(DroidFault::LogOverflow);}
  raw_.setZero();tracking_raw_.setZero();Vector7d out=Vector7d::Zero();
  bool abort=false;
  const bool stop_gap=stopping_ && (now<=last_callback_ns_ || now-last_callback_ns_>runtime_.limits.callback_gap_ns ||
      !std::isfinite(dt.toSec()) || dt.toSec()<=0 || dt.toSec()>runtime_.limits.callback_gap_ns*1e-9);
  last_callback_ns_=now;
  try {
    if (stop_gap) throw std::runtime_error("DROID callback gap during stop");
    if (!stopping_ && runtime_.fault==DroidFault::None && !stop_requested_.load()) {
      JointReference reference;reference.q=runtime_.active->q;
      target_=reference.q;target_velocity_.setZero();
      const auto cmd=trackingCommand(s,reference,dt.toSec());
      out=Eigen::Map<const Vector7d>(cmd.tau_J.data());
    }
    if (runtime_.fault!=DroidFault::None || stop_requested_.load()) {
      if (!stopping_) {stopping_=true;stop_start_=now;initial_stop_=s.tau_J_d;}
    }
    if (stopping_) {
      // Candidate follows the historical damping-stop law; NOT a universal safe stop.
      // Invalid feedback/robot reflex/contact cannot qualify a software damping stop.
      if (!s.q.allFinite() || !s.dq.allFinite() || !s.tau_J_d.allFinite() || s.current_errors || contact)
        throw std::runtime_error("DROID cannot continue software stop with invalid/contact/reflex state");
      const double elapsed=(now-stop_start_)*1e-9;
      Vector7d damping=-stop_.damping.cwiseProduct(s.dq);
      if (stop_.compensate_coriolis) damping+=robot()->model()->coriolis(s);
      const double u=stop_.ramp_duration>0?std::clamp(elapsed/stop_.ramp_duration,0.,1.):1.;
      raw_=(1-u)*initial_stop_+u*damping;
      if (!raw_.allFinite()) throw std::runtime_error("Nonfinite DROID stop torque");
      out=saturateTorqueRate(raw_,s.tau_J_d,stop_.max_delta_tau);
      if ((out.cwiseAbs().array()>runtime_.limits.command_torque.array()).any())
        throw std::runtime_error("DROID stop torque exceeds guard");
      const bool rest=u>=1 && s.dq.cwiseAbs().maxCoeff()<stop_.velocity_epsilon;
      if (!rest) rest_since_=0;else if (!rest_since_) rest_since_=now;
      at_rest_=rest_since_ && now-rest_since_>=100000000LL;
      if (at_rest_) finished_=true;
      else if (elapsed>=stop_.max_duration) {stop_fault_=static_cast<int>(DroidFault::StopTimeout);finished_=true;}
    }
    if (!stopping_) runtime_.consumed(now); // Only a command selected for callback return is consumed.
  } catch (...) {
    runtime_.latch(DroidFault::StopState);stop_fault_=static_cast<int>(DroidFault::StopState);abort=true;
  }
  DroidJointSample sample;
  sample.tick=runtime_.tick;sample.monotonic_ns=now;sample.consumed=runtime_.receipt;
  if (runtime_.active) {sample.reference=runtime_.active->q;sample.publication_sequence=runtime_.active->publication_sequence;}
  sample.q=s.q;sample.dq=s.dq;sample.tracking_raw=tracking_raw_;sample.raw=raw_;sample.outgoing=out;
  sample.fault=static_cast<int>(runtime_.fault);sample.stop_fault=stop_fault_;sample.dropped=dropped_;
  sample.stopping=stopping_;sample.stop_finished=finished_ && !abort;sample.stop_at_rest=at_rest_;
  if (!full) {log_[w]=sample;write_.store(next,std::memory_order_release);}
  status_.setUnsafe(sample); // Terminal evidence survives a full telemetry ring.
  if (abort) throw std::runtime_error("DROID native control/stop failed; inspect native status");
  auto command=torques(out);return finished_?franka::MotionFinished(command):command;
}
}  // namespace franky
