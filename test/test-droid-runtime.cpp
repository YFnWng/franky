#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "franky/motion/droid_joint_impedance_motion.hpp"
#include <limits>
using namespace franky;
namespace {
DroidJointLimits limits() {
  DroidJointLimits l;l.lower.setConstant(-3);l.upper.setConstant(3);l.velocity.setConstant(1);
  l.command_torque.setConstant(10);l.publisher_timeout_ns=50000000;l.callback_gap_ns=5000000;return l;
}
JointImpedanceParams gains() {
  JointImpedanceParams p;p.damping=Vector7d::Constant(5);
  p.cartesian_gains=CartesianImpedanceGains::diagonal(Vector6d::Constant(1),Vector6d::Constant(1));return p;
}
TorqueStopParams stop() {
  TorqueStopParams p;p.compensate_coriolis=false;p.ramp_duration=.01;p.max_duration=.3;return p;
}
DroidJointPublication pub(uint64_t seq=1,uint64_t row=1,int64_t now=1000000000LL) {
  DroidJointPublication p;p.publication_sequence=seq;p.row_sequence=row;p.published_ns=now;p.expires_ns=now+1000000000;return p;
}
struct FakeMotion : DroidJointImpedanceMotion {
  int64_t now{1000000000LL};double raw{0};bool stop_during_compute{false};
  FakeMotion(size_t cap=1000) : DroidJointImpedanceMotion(gains(),limits(),stop(),cap,.1) {}
  int64_t clockNs() const override {return now;}
  franka::Torques trackingCommand(const RobotState &,const JointReference &,double) override {
    observeRawJointTorque(Vector7d::Constant(raw));if(stop_during_compute) requestStop();return franka::Torques(std::array<double,7>{});
  }
  franka::Torques step(RobotState &s) {now+=1000000;return nextCommand(s,franka::Duration(1),{}, {},std::nullopt);}
};
}
TEST_CASE("explicit limits and controller configuration required") {
  REQUIRE_THROWS(DroidJointRuntime(DroidJointLimits{}));
  REQUIRE_THROWS(DroidJointImpedanceMotion(JointImpedanceParams{},limits(),stop(),100,.1));
  REQUIRE_THROWS(FakeMotion(0));
}
TEST_CASE("receipt follows computation, not publication; heartbeat cannot refresh itself") {
  DroidJointRuntime r(limits());auto p=pub();Vector7d z=Vector7d::Zero();
  r.advance(1000000000,0,z,z,z,p);REQUIRE(r.receipt.row_sequence==0);
  r.checkCommand(z);r.consumed(1000000000);REQUIRE(r.receipt.row_sequence==1);REQUIRE(r.receipt.tick==1);
  auto h=pub(2,1,1001000000);h.expires_ns=p.expires_ns;r.advance(h.published_ns,.001,z,z,z,h);r.consumed(h.published_ns);
  REQUIRE(r.receipt.tick==1); // first consumption, not a fabricated heartbeat receipt
  for(int i=2;i<=51;++i) r.advance(1000000000LL+i*1000000,.001,z,z,z,std::nullopt);
  REQUIRE(r.fault==DroidFault::PublisherTimeout);
  r.advance(1052000000,.001,z,z,z,pub(3,2,1052000000));REQUIRE(r.fault==DroidFault::PublisherTimeout);
}
TEST_CASE("native guards latch independently") {
  Vector7d z=Vector7d::Zero();
  SECTION("callback wall gap") {DroidJointRuntime r(limits());r.advance(1000000000,0,z,z,z,pub());r.advance(1006000000,.001,z,z,z,std::nullopt);REQUIRE(r.fault==DroidFault::CallbackGap);}
  SECTION("callback reported gap") {DroidJointRuntime r(limits());r.advance(1000000000,.006,z,z,z,pub());REQUIRE(r.fault==DroidFault::CallbackGap);}
  SECTION("measured velocity") {DroidJointRuntime r(limits());r.advance(1000000000,0,z,Vector7d::Constant(2),z,pub());REQUIRE(r.fault==DroidFault::JointVelocity);}
  SECTION("raw torque before slew") {DroidJointRuntime r(limits());r.advance(1000000000,0,z,z,z,pub());r.checkCommand(Vector7d::Constant(11));r.consumed(1000000000);REQUIRE(r.fault==DroidFault::CommandTorque);REQUIRE(r.receipt.tick==0);}
  SECTION("nonfinite") {DroidJointRuntime r(limits());z[0]=std::numeric_limits<double>::quiet_NaN();r.advance(1000000000,0,z,z,z,pub());REQUIRE(r.fault==DroidFault::Nonfinite);}
  SECTION("deadline") {DroidJointRuntime r(limits());auto p=pub();p.expires_ns=1000000001;r.advance(1000000001,0,z,z,z,p);REQUIRE(r.fault==DroidFault::PublisherTimeout);}
  SECTION("reordered publication") {DroidJointRuntime r(limits());r.advance(1000000000,0,z,z,z,pub());r.advance(1001000000,.001,z,z,z,pub());REQUIRE(r.fault==DroidFault::Sequence);}
  SECTION("changed same row") {DroidJointRuntime r(limits());r.advance(1000000000,0,z,z,z,pub());auto p=pub(2,1,1001000000);p.q[0]=.1;r.advance(1001000000,.001,z,z,z,p);REQUIRE(r.fault==DroidFault::Sequence);}
}
TEST_CASE("actual motion callback receipts and stop evidence with fake torque and clock") {
  FakeMotion m;RobotState s{};s.joint_contact.setZero();s.cartesian_contact.setZero();s.joint_collision.setZero();s.cartesian_collision.setZero();s.q.setZero();s.dq.setZero();s.tau_J_d.setZero();
  Vector7d q=Vector7d::Zero();q[6]=.123;m.publish(q,1,m.now+1000000000);
  REQUIRE(m.status().consumed.tick==0);m.init(nullptr,s,std::nullopt);m.step(s);
  REQUIRE(m.status().consumed.row_sequence==1);REQUIRE(m.status().reference[6]==Approx(.123));
  m.requestStop();for(int i=0;i<150;++i) {auto cmd=m.step(s);if(cmd.motion_finished) break;}
  REQUIRE(m.status().stop_finished);REQUIRE(m.status().stop_at_rest);REQUIRE(m.status().stop_fault==0);REQUIRE(m.status().fault==0);
  REQUIRE_THROWS(m.init(nullptr,s,std::nullopt));
}
TEST_CASE("overflow faults without inventing consumption; terminal status survives full ring") {
  FakeMotion m(2);RobotState s{};s.joint_contact.setZero();s.cartesian_contact.setZero();s.joint_collision.setZero();s.cartesian_collision.setZero();s.q.setZero();s.dq.setZero();s.tau_J_d.setZero();
  m.publish(Vector7d::Zero(),1,m.now+1000000000);m.init(nullptr,s,std::nullopt);m.step(s);m.step(s);
  REQUIRE(m.status().fault==static_cast<int>(DroidFault::LogOverflow));REQUIRE(m.status().dropped==1);
  REQUIRE(m.status().stopping);REQUIRE(m.drain().size()==1);
}
TEST_CASE("faulting raw command cannot produce a receipt") {
  FakeMotion m;m.raw=20;RobotState s{};s.joint_contact.setZero();s.cartesian_contact.setZero();s.joint_collision.setZero();s.cartesian_collision.setZero();s.q.setZero();s.dq.setZero();s.tau_J_d.setZero();
  m.publish(Vector7d::Zero(),1,m.now+1000000000);m.init(nullptr,s,std::nullopt);m.step(s);
  REQUIRE(m.status().fault==static_cast<int>(DroidFault::CommandTorque));REQUIRE(m.status().consumed.tick==0);REQUIRE(m.status().tracking_raw[0]==Approx(20));
}
TEST_CASE("stop timeout is completion without false rest") {
  FakeMotion m;RobotState s{};s.joint_contact.setZero();s.cartesian_contact.setZero();s.joint_collision.setZero();s.cartesian_collision.setZero();s.q.setZero();s.dq.setConstant(.03);s.tau_J_d.setZero();
  m.publish(Vector7d::Zero(),1,m.now+1000000000);m.init(nullptr,s,std::nullopt);m.requestStop();
  for(int i=0;i<350;++i) {auto cmd=m.step(s);if(cmd.motion_finished)break;}
  REQUIRE(m.status().stop_finished);REQUIRE_FALSE(m.status().stop_at_rest);
  REQUIRE(m.status().stop_fault==static_cast<int>(DroidFault::StopTimeout));
}

TEST_CASE("publisher cannot extend same-row deadline or mutate its target") {
  FakeMotion m;auto q=Vector7d::Zero().eval();auto expiry=m.now+1000000000;
  m.publish(q,1,expiry);
  REQUIRE_THROWS(m.publish(q,1,expiry+1));q[0]=.01;REQUIRE_THROWS(m.publish(q,1,expiry));
}
TEST_CASE("callback gap during stopping records abort evidence") {
  FakeMotion m;RobotState s{};s.joint_contact.setZero();s.cartesian_contact.setZero();s.joint_collision.setZero();s.cartesian_collision.setZero();s.q.setZero();s.dq.setZero();s.tau_J_d.setZero();
  m.publish(Vector7d::Zero(),1,m.now+1000000000);m.init(nullptr,s,std::nullopt);m.step(s);
  m.requestStop();m.step(s);m.now+=6000000;REQUIRE_THROWS(m.step(s));
  REQUIRE(m.status().stop_fault==static_cast<int>(DroidFault::StopState));
  REQUIRE_FALSE(m.status().stop_finished);
}

TEST_CASE("actual motion stops on producer death without refreshing receipt") {
  FakeMotion m;RobotState s{};s.joint_contact.setZero();s.cartesian_contact.setZero();s.joint_collision.setZero();s.cartesian_collision.setZero();s.q.setZero();s.dq.setZero();s.tau_J_d.setZero();
  m.publish(Vector7d::Zero(),1,m.now+1000000000);m.init(nullptr,s,std::nullopt);
  for(int i=0;i<250;++i) {if(m.step(s).motion_finished) break;}
  REQUIRE(m.status().fault==static_cast<int>(DroidFault::PublisherTimeout));
  REQUIRE(m.status().consumed.tick==1);REQUIRE(m.status().stop_finished);REQUIRE(m.status().stop_at_rest);
}

TEST_CASE("a tracking command replaced by stop in the same callback is not acknowledged") {
  FakeMotion m;m.stop_during_compute=true;
  RobotState s{};s.joint_contact.setZero();s.cartesian_contact.setZero();s.joint_collision.setZero();s.cartesian_collision.setZero();s.q.setZero();s.dq.setZero();s.tau_J_d.setZero();
  m.publish(Vector7d::Zero(),1,m.now+1000000000);m.init(nullptr,s,std::nullopt);m.step(s);
  REQUIRE(m.status().stopping);REQUIRE(m.status().consumed.tick==0);
}
