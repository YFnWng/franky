#include "franky/motion/m4_runtime_motion.hpp"
#include "franky/model.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace franky {
namespace {
int64_t nowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool exceeds(const Vector7d &x, const Vector7d &bound) { return (x.cwiseAbs().array() > bound.array()).any(); }
}
bool validM4DlsPeriod(double dt_s, uint64_t tick, double maximum_s) {
  // libfranka's first callback has no elapsed interval. Do not invent an
  // integration step; zero is allowed only for that initialization callback.
  return std::isfinite(dt_s) && dt_s <= maximum_s &&
      (dt_s > 0 || (tick == 1 && dt_s == 0));
}
void M4DlsParams::validate() const {
  for (const auto *v : {&stiffness, &damping, &reference_error_limit})
    if (!v->allFinite() || (v->array() <= 0).any()) throw std::invalid_argument("Invalid DLS joint parameters");
  for (const auto *v : {&pose_gain, &task_scale})
    if (!v->allFinite() || (v->array() <= 0).any()) throw std::invalid_argument("Invalid DLS task parameters");
  if (!std::isfinite(lambda) || lambda<=0 || !std::isfinite(rank_tolerance) || rank_tolerance<=0 || rank_tolerance>=1 ||
      !std::isfinite(posture_gain) || posture_gain<0) throw std::invalid_argument("Invalid DLS solver parameters");
}
M4DlsStep computeM4DlsStep(const Jacobian &j, const Vector6d &error,
                         const Vector6d &desired, const Vector7d &posture,
                         const Vector7d &bound, const M4DlsParams &p) {
  if (!j.allFinite() || !error.allFinite() || !desired.allFinite() || !posture.allFinite() ||
      !bound.allFinite() || (bound.array()<=0).any()) throw std::invalid_argument("Invalid DLS input");
  const Jacobian scaled=p.task_scale.asDiagonal()*j;
  Eigen::JacobiSVD<Jacobian> svd(scaled, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Vector6d command=p.task_scale.cwiseProduct(desired+p.pose_gain.cwiseProduct(error));
  const double threshold=p.rank_tolerance*svd.singularValues()[0];
  M4DlsStep result;
  Eigen::Matrix<double,7,7> projector=Eigen::Matrix<double,7,7>::Identity();
  for (int i=0;i<6;++i) {
    const double sigma=svd.singularValues()[i];
    result.velocity+=svd.matrixV().col(i)*(sigma/(sigma*sigma+p.lambda*p.lambda))*svd.matrixU().col(i).dot(command);
    if (sigma>threshold) { projector-=svd.matrixV().col(i)*svd.matrixV().col(i).transpose(); ++result.rank; }
  }
  result.secondary_velocity=projector*(p.posture_gain*posture);
  result.velocity+=result.secondary_velocity;
  result.velocity_scale=std::min(1.,1./std::max(1.,(result.velocity.cwiseAbs().array()/bound.array()).maxCoeff()));
  result.velocity*=result.velocity_scale; result.secondary_velocity*=result.velocity_scale;
  return result;
}
int M4StopProgress::update(double elapsed, const Vector7d &dq, const TorqueStopParams &params) {
  if (elapsed >= params.ramp_duration && dq.allFinite() &&
      dq.cwiseAbs().maxCoeff() < params.velocity_epsilon) {
    if (rest_since < 0) rest_since=elapsed;
    if (elapsed-rest_since >= .1) return 1;
  } else rest_since=-1;
  return elapsed >= params.max_duration ? 18 : 0;
}
Vector7d blendM4StopTorque(const Vector7d &initial, const Vector7d &hold,
                         double elapsed, const TorqueStopParams &params,
                         const Vector7d &previous, const Vector7d &bounds) {
  const double u=params.ramp_duration > 0 ? std::clamp(elapsed/params.ramp_duration,0.,1.) : 1.;
  const double s=u*u*(3.-2.*u);
  Vector7d desired=((1.-s)*initial+s*hold).cwiseMax(-bounds).cwiseMin(bounds);
  return saturateTorqueRate(desired,previous,params.max_delta_tau);
}
Vector6d computeM4PoseHoldWrench(const Matrix6d &stiffness, const Matrix6d &damping,
                               const Vector6d &error, const Vector6d &twist) {
  return -stiffness*error-damping*twist;
}
void M4RuntimeLimits::validate() const {
  if (!joint_lower.allFinite() || !joint_upper.allFinite() || !workspace_lower.allFinite() ||
      !workspace_upper.allFinite() || (joint_lower.array() >= joint_upper.array()).any() ||
      (workspace_lower.array() >= workspace_upper.array()).any()) throw std::invalid_argument("Invalid M4 bounds");
  for (const auto *v : {&joint_velocity, &external_torque, &command_torque})
    if (!v->allFinite() || (v->array() <= 0).any()) throw std::invalid_argument("Invalid M4 joint thresholds");
  for (double x : {reference_timeout_s, callback_gap_s, position_error_m, orientation_error_rad,
                  linear_speed_m_s, angular_speed_rad_s, minimum_jacobian_singular_value,
                  minimum_command_success_rate})
    if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("Invalid M4 thresholds");
  if (minimum_command_success_rate > 1) throw std::invalid_argument("Invalid success-rate threshold");
}
int evaluateM4Limits(const M4RuntimeLimits &l, const M4RuntimeSample &s, bool contact, bool robot_error) {
  if (!s.q.allFinite() || !s.dq.allFinite() || !s.pose.matrix().allFinite() || !s.twist.allFinite() ||
      !s.reference_pose.matrix().allFinite() || !s.reference_twist.allFinite() ||
      !s.external_torque.allFinite() || !s.torque_measured.allFinite() || !s.torque_previous.allFinite() ||
      !std::isfinite(s.reference_age_s) || !std::isfinite(s.dt_s) || !std::isfinite(s.min_singular_value) ||
      !std::isfinite(s.command_success_rate)) return 4;
  if (s.reference_age_s < 0 || s.reference_age_s > l.reference_timeout_s) return 1;
  if (s.dt_s > l.callback_gap_s || s.dt_s < 0) return 3;
  if ((s.q.array() < l.joint_lower.array()).any() || (s.q.array() > l.joint_upper.array()).any()) return 5;
  if (exceeds(s.dq, l.joint_velocity)) return 6;
  for (const auto &p : {s.pose.translation().eval(), s.reference_pose.translation().eval()})
    if ((p.array() < l.workspace_lower.array()).any() || (p.array() > l.workspace_upper.array()).any()) return 7;
  if ((s.pose.translation()-s.reference_pose.translation()).norm() > l.position_error_m ||
      Eigen::AngleAxisd(s.pose.linear().transpose()*s.reference_pose.linear()).angle() > l.orientation_error_rad) return 8;
  if (s.twist.head<3>().norm() > l.linear_speed_m_s || s.twist.tail<3>().norm() > l.angular_speed_rad_s ||
      s.reference_twist.head<3>().norm() > l.linear_speed_m_s || s.reference_twist.tail<3>().norm() > l.angular_speed_rad_s) return 9;
  if (exceeds(s.external_torque,l.external_torque)) return 10;
  if (contact && l.abort_on_contact) return 11;
  if (s.min_singular_value < l.minimum_jacobian_singular_value) return 12;
  if (robot_error) return 13;
  if (s.tick > 100 && s.command_success_rate < l.minimum_command_success_rate) return 14;
  return 0;
}
int evaluateM4StopLimits(const M4RuntimeLimits &l, const M4RuntimeSample &s, const Affine &anchor) {
  if (!s.q.allFinite() || !s.dq.allFinite() || !s.pose.matrix().allFinite() ||
      !anchor.matrix().allFinite() || s.min_singular_value <= 0) return 17;
  if ((s.q.array()<l.joint_lower.array()).any() || (s.q.array()>l.joint_upper.array()).any()) return 5;
  const auto &p=s.pose.translation();
  if ((p.array()<l.workspace_lower.array()).any() || (p.array()>l.workspace_upper.array()).any()) return 7;
  if ((p-anchor.translation()).norm()>l.position_error_m ||
      Eigen::AngleAxisd(s.pose.linear().transpose()*anchor.linear()).angle()>l.orientation_error_rad) return 8;
  return 0;
}
M4RuntimeMotion::M4RuntimeMotion(const Params &params, const M4RuntimeLimits &limits, const TorqueStopParams &stop,
    std::optional<CurveFixtureParams> fixture, size_t capacity, std::optional<M4DlsParams> dls,
    std::optional<Matrix6d> stop_stiffness, std::optional<Matrix6d> stop_damping)
    : CartesianImpedanceBase(Affine::Identity(),params), limits_(limits), stop_params_(stop),
      stop_stiffness_(stop_stiffness), stop_damping_(stop_damping), dls_(std::move(dls)), fixture_(std::move(fixture)), log_(capacity) {
  limits_.validate();
  if (stop_stiffness_.has_value()!=stop_damping_.has_value()) throw std::invalid_argument("Both stop gains required");
  if (stop_stiffness_) { Params hold=params; hold.stiffness=*stop_stiffness_; hold.damping=*stop_damping_; hold.validate(); }
  if (dls_) { dls_->validate(); if (fixture_) throw std::invalid_argument("DLS cannot use curve fixture"); }
  if (capacity < 2) throw std::invalid_argument("M4 log capacity too small");
  if (fixture_) fixture_->validate();
  if (!stop.damping.allFinite() || (stop.damping.array() <= 0).any() ||
      !std::isfinite(stop.ramp_duration) || stop.ramp_duration < 0 ||
      !std::isfinite(stop.max_duration) || stop.max_duration <= stop.ramp_duration ||
      !std::isfinite(stop.velocity_epsilon) || stop.velocity_epsilon <= 0 ||
      !std::isfinite(stop.max_delta_tau) || stop.max_delta_tau <= 0) throw std::invalid_argument("Invalid M4 stop");
}
void M4RuntimeMotion::publish(M4Reference ref) {
  ref.cartesian.validate();
  if (ref.has_posture && (!ref.posture.allFinite() || (ref.posture.array()<limits_.joint_lower.array()).any() ||
      (ref.posture.array()>limits_.joint_upper.array()).any() || !std::isfinite(ref.posture_transition_s) ||
      ref.posture_transition_s <= 0)) throw std::invalid_argument("Invalid M4 posture reference");
  ref.published_ns=nowNs(); references_.set(ref);
}
void M4RuntimeMotion::latch(int code) { int expected=0; fault_.compare_exchange_strong(expected,code); }
void M4RuntimeMotion::initImpl(const RobotState &state, const std::optional<franka::Torques> &) {
  current_.cartesian.target = robot()->model()->pose(franka::Frame::kEndEffector,state);
  current_.published_ns=nowNs();
  applied_posture_=posture_start_=posture_goal_=state.q;
  integrated_reference_=state.q;
  if (fixture_ && projectCurveFixture(*fixture_,current_.cartesian.target.translation()).distance > fixture_->initialization_tolerance)
    latch(17);
}
void M4RuntimeMotion::observeTorque(const Vector7d &raw, const Vector7d &task, const Vector7d &posture,
    const Vector7d &limit, const Vector7d &coriolis) {
  sample_.raw=raw; sample_.task=task; sample_.nullspace=posture; sample_.limit=limit; sample_.coriolis=coriolis;
}
Vector6d M4RuntimeMotion::computeCartesianWrench(const RobotState &state, const CartesianReference &ref,
    const Vector6d &error, const Vector6d &measured, const Vector6d &desired) {
  if (stopping_) return computeM4PoseHoldWrench(stop_stiffness_.value_or(currentStiffness()),stop_damping_.value_or(currentDamping()),error,measured);
  Vector6d wrench=CartesianImpedanceBase::computeCartesianWrench(state,ref,error,measured,desired);
  // A fixture frees its tangent during guidance, but a stop must hold all axes.
  if (fixture_ && !stopping_) wrench.head<3>()=computeCurveFixtureForce(projectCurveFixture(*fixture_,state.O_T_EE.translation()),
      measured.head<3>(),fixture_->normal_stiffness,fixture_->normal_damping,fixture_->tangent_damping);
  return wrench;
}
bool M4RuntimeMotion::push(const M4RuntimeSample &sample) {
  size_t w=write_.load(std::memory_order_relaxed), next=(w+1)%log_.size();
  if (next==read_.load(std::memory_order_acquire)) { dropped_.fetch_add(1); return false; }
  log_[w]=sample; write_.store(next,std::memory_order_release); return true;
}
std::vector<M4RuntimeSample> M4RuntimeMotion::drain() {
  std::vector<M4RuntimeSample> result;
  size_t r=read_.load(std::memory_order_relaxed), w=write_.load(std::memory_order_acquire);
  while (r!=w) { result.push_back(log_[r]); r=(r+1)%log_.size(); }
  read_.store(r,std::memory_order_release); return result;
}
franka::Torques M4RuntimeMotion::nextCommandImpl(const RobotState &state, franka::Duration dt,
    franka::Duration rel, franka::Duration /*abs*/, const std::optional<franka::Torques> & /*previous*/) {
  elapsed_s_+=dt.toSec();
  if (references_.hasNewData() && !stopping_) {
    auto ref=references_.getUnsafe();
    if (ref.sequence<=current_.sequence) latch(2);
    else {
      if (ref.has_posture && (ref.posture-posture_goal_).norm()>0) {
        posture_start_=applied_posture_; posture_goal_=ref.posture; posture_start_s_=elapsed_s_;
      }
      current_=ref;
    }
  }
  double u=std::clamp((elapsed_s_-posture_start_s_)/current_.posture_transition_s,0.0,1.0);
  applied_posture_=posture_start_+(10*u*u*u-15*u*u*u*u+6*u*u*u*u*u)*(posture_goal_-posture_start_);
  sample_=M4RuntimeSample{}; sample_.tick=++tick_; sample_.monotonic_ns=nowNs();
  if (last_tick_ns_ && (sample_.monotonic_ns-last_tick_ns_)*1e-9 > limits_.callback_gap_s) latch(3);
  last_tick_ns_=sample_.monotonic_ns;
  sample_.reference_sequence=current_.sequence; sample_.reference_age_s=(sample_.monotonic_ns-current_.published_ns)*1e-9;
  sample_.robot_time_s=state.time.toSec(); sample_.dt_s=dt.toSec(); sample_.q=state.q; sample_.dq=state.dq;
  sample_.torque_measured=state.tau_J; sample_.torque_previous=state.tau_J_d; sample_.external_torque=state.tau_ext_hat_filtered;
  sample_.command_success_rate=state.control_command_success_rate; sample_.posture=applied_posture_;
  franka::Torques command(std::array<double,7>{});
  try {
    sample_.pose=robot()->model()->pose(franka::Frame::kEndEffector,state);
    const Jacobian j=robot()->model()->zeroJacobian(franka::Frame::kEndEffector,state);
    if (!j.allFinite()) throw std::runtime_error("Nonfinite M4 Jacobian");
    sample_.twist=j*state.dq;
    sample_.min_singular_value=Eigen::JacobiSVD<Jacobian>(j).singularValues().minCoeff();
    auto reference=current_.cartesian;
    if (stopping_) {
      reference.target=stop_pose_; reference.target_twist=Twist();
    } else if (fixture_) {
      reference.target.translation()=projectCurveFixture(*fixture_,sample_.pose.translation()).projected_point;
      reference.target.linear()=fixture_->orientation.toRotationMatrix(); reference.target_twist=Twist();
    }
    sample_.reference_pose=reference.target;
    sample_.reference_twist=reference.target_twist ? reference.target_twist->vector_repr() : Vector6d::Zero();
    if (!stopping_) {
      // Collision always faults, including hand-guided fixture sessions.
      if ((state.joint_collision.array()>0).any() || (state.cartesian_collision.array()>0).any()) latch(11);
      int code=evaluateM4Limits(limits_,sample_,(state.joint_contact.array()>0).any() ||
          (state.cartesian_contact.array()>0).any(),bool(state.current_errors));
      if (code) latch(code);
      if (!fault() && !stop_requested_.load()) {
        // Use encoder-model pose consistently for feedback in the inherited torque law.
        RobotState measured=state; measured.O_T_EE=sample_.pose;
        if (dls_) {
          if (!validM4DlsPeriod(dt.toSec(),sample_.tick,limits_.callback_gap_s))
            throw std::runtime_error("DLS dt outside limits");
          Vector6d error;
          error.head<3>()=reference.target.translation()-sample_.pose.translation();
          Eigen::AngleAxisd aa(reference.target.linear()*sample_.pose.linear().transpose());
          error.tail<3>()=aa.angle()*aa.axis();
          const auto step=computeM4DlsStep(j,error,sample_.reference_twist,applied_posture_-state.q,limits_.joint_velocity,*dls_);
          const Vector7d next=integrated_reference_+dt.toSec()*step.velocity;
          if ((next.array()<limits_.joint_lower.array()).any() || (next.array()>limits_.joint_upper.array()).any() ||
              exceeds(next-state.q,dls_->reference_error_limit)) latch(19);
          else {
            integrated_reference_=next;
            sample_.dls_active=true; sample_.joint_reference=next; sample_.joint_velocity_reference=step.velocity;
            sample_.secondary_velocity=step.secondary_velocity; sample_.outer_correction=dls_->pose_gain.cwiseProduct(error);
            sample_.dls_velocity_scale=step.velocity_scale; sample_.dls_rank=step.rank;
            sample_.task=computeJointPdTorque(dls_->stiffness,dls_->damping,next-state.q,step.velocity-state.dq);
            const auto &safety=base_params().safety;
            sample_.limit=computeJointLimitTorque(state.q,state.dq,limits_.joint_lower,limits_.joint_upper,
                safety.joint_limit_activation_distance,safety.joint_limit_stiffness,safety.joint_limit_damping,safety.joint_limit_max_torque);
            sample_.coriolis=robot()->model()->coriolis(state);
            sample_.raw=sample_.task+sample_.limit+sample_.coriolis;
            const Vector7d torque=saturateTorqueRate(sample_.raw,state.tau_J_d,1.);
            Eigen::Map<Vector7d>(command.tau_J.data())=torque;
          }
        } else command=computeCommand(measured,reference,dt.toSec());
        if (!sample_.raw.allFinite()) latch(4);
        else if (exceeds(sample_.raw,limits_.command_torque)) latch(15);
      }
    }
  } catch (const std::exception &) { latch(17); }
  if (fault() || stop_requested_.load()) {
    if (!stopping_) {
      stopping_=true; stop_start_s_=rel.toSec(); stop_pose_=sample_.pose;
      stop_initial_torque_=state.tau_J_d;
      applied_posture_=posture_start_=posture_goal_=state.q;
    }
    try {
      const int guard=evaluateM4StopLimits(limits_,sample_,stop_pose_);
      if (guard) { stop_fault_.store(guard); throw std::runtime_error("M4 stop containment failure"); }
      if ((state.joint_collision.array()>0).any() || (state.cartesian_collision.array()>0).any() || state.current_errors) {
        stop_fault_.store(11); throw std::runtime_error("M4 stop robot collision/error");
      }
      if (!sample_.pose.matrix().allFinite() || sample_.min_singular_value <= 0 ||
          !state.dq.allFinite()) throw std::runtime_error("Invalid feedback during M4 stop");
      CartesianReference hold;
      hold.target=stop_pose_; hold.target_twist=Twist();
      sample_.reference_pose=stop_pose_; sample_.reference_twist=Vector6d::Zero();
      RobotState measured=state; measured.O_T_EE=sample_.pose;
      // Inherited law retains tested Cartesian gains, Coriolis and joint limits.
      // observeTorque supplies the unslewed result; blend and slew only once below.
      (void)computeCommand(measured,hold,dt.toSec());
      Vector7d hold_torque=sample_.raw-stop_params_.damping.cwiseProduct(state.dq);
      if (!stop_params_.compensate_coriolis) { hold_torque-=sample_.coriolis; sample_.coriolis.setZero(); }
      if (!hold_torque.allFinite()) throw std::runtime_error("Nonfinite M4 stop torque");
      sample_.raw=hold_torque;
      const double elapsed=rel.toSec()-stop_start_s_;
      const auto torque=blendM4StopTorque(stop_initial_torque_,hold_torque,elapsed,
                                        stop_params_,state.tau_J_d,limits_.command_torque);
      command=franka::Torques(std::array<double,7>{});
      Eigen::Map<Vector7d>(command.tau_J.data())=torque;
      const int progress=stop_progress_.update(elapsed,state.dq,stop_params_);
      if (progress==1) command.motion_finished=true;
      if (progress==18) stop_fault_.store(18);
    } catch (const std::exception &) { if (!stopFault()) stop_fault_.store(17); }
  }
  sample_.fault=fault(); sample_.stop_fault=stopFault(); sample_.stopping=stopping_; sample_.stop_finished=command.motion_finished;
  sample_.stop_at_rest=state.dq.cwiseAbs().maxCoeff()<stop_params_.velocity_epsilon;
  sample_.slew=Eigen::Map<const Vector7d>(command.tau_J.data());
  constexpr double alpha=.001/(.001+1.0/(2.0*3.141592653589793*100.0));
  sample_.filtered_prediction=state.tau_J_d+alpha*(sample_.slew-state.tau_J_d);
  if (!push(sample_)) latch(16); // Latched stop begins on the next servo tick.
  // Deadline/model failure is an explicit control abort, never MotionFinished
  // while moving. Record it first; libfranka unwinds control on this exception.
  if (stopFault()) throw std::runtime_error(stopFault()==18 ? "M4 stop timeout: not at sustained rest" : "M4 stop containment/feedback/controller failure");
  return command;
}
} // namespace franky
