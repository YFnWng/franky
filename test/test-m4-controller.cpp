#define CATCH_CONFIG_MAIN
#include <Eigen/QR>
#include <catch2/catch.hpp>

#include "franky/motion/cartesian_curve_impedance_motion.hpp"
#include "franky/motion/nullspace_projector.hpp"

namespace {

franky::Jacobian deterministicJacobian() {
  franky::Jacobian jacobian;
  jacobian << 0.4, -0.2, 0.1, 0.3, -0.5, 0.7, 0.2, -0.1, 0.6, 0.2, -0.4, 0.3, 0.1, -0.2, 0.5, 0.1, -0.3, 0.2, 0.4, -0.6,
      0.8, 0.2, 0.3, 0.7, -0.1, 0.5, 0.4, -0.3, -0.4, 0.2, 0.1, 0.8, -0.2, 0.3, 0.6, 0.3, -0.5, 0.4, 0.1, 0.7, -0.2,
      0.5;
  return jacobian;
}

Eigen::Matrix<double, 7, 7> deterministicMass() {
  Eigen::Matrix<double, 7, 7> factor;
  factor << 2.0, 0.1, 0.0, 0.2, 0.0, 0.0, 0.1, 0.0, 1.8, 0.2, 0.0, 0.1, 0.0, 0.0, 0.1, 0.0, 1.6, 0.1, 0.0, 0.2, 0.0,
      0.0, 0.1, 0.0, 1.5, 0.2, 0.0, 0.1, 0.1, 0.0, 0.2, 0.0, 1.4, 0.1, 0.0, 0.0, 0.1, 0.0, 0.2, 0.0, 1.3, 0.1, 0.1, 0.0,
      0.1, 0.0, 0.2, 0.0, 1.2;
  return factor.transpose() * factor + 0.5 * Eigen::Matrix<double, 7, 7>::Identity();
}

}  // namespace

TEST_CASE("kinematic projector preserves the historical COD result") {
  const auto jacobian = deterministicJacobian();
  const auto mass = deterministicMass();
  franky::Vector7d torque;
  torque << 1.0, -2.0, 0.5, 3.0, -0.4, 0.7, 1.5;

  Eigen::CompleteOrthogonalDecomposition<franky::Jacobian> decomposition;
  decomposition.setThreshold(1e-6);
  decomposition.compute(jacobian);
  const franky::Vector7d expected = torque - decomposition.pseudoInverse() * (jacobian * torque);
  const auto actual =
      franky::projectNullspaceTorque(jacobian, mass, torque, franky::NullspaceProjectorType::kKinematic);
  REQUIRE((actual - expected).norm() < 1e-12);
}

TEST_CASE("dynamic projector is mass-consistent") {
  const auto jacobian = deterministicJacobian();
  const auto mass = deterministicMass();
  franky::Vector7d torque;
  torque << 1.0, -2.0, 0.5, 3.0, -0.4, 0.7, 1.5;
  const auto projected =
      franky::projectNullspaceTorque(jacobian, mass, torque, franky::NullspaceProjectorType::kDynamicallyConsistent);
  REQUIRE((jacobian * mass.ldlt().solve(projected)).norm() < 1e-10);
  REQUIRE(projected.allFinite());
}

TEST_CASE("dynamic projector remains finite for a rank-deficient Jacobian") {
  auto jacobian = deterministicJacobian();
  jacobian.row(5) = jacobian.row(4);
  const auto projected = franky::projectNullspaceTorque(
      jacobian,
      deterministicMass(),
      franky::Vector7d::Ones(),
      franky::NullspaceProjectorType::kDynamicallyConsistent,
      1e-6);
  REQUIRE(projected.allFinite());
  REQUIRE((jacobian * deterministicMass().ldlt().solve(projected)).norm() < 1e-9);
}

TEST_CASE("dynamic projector rejects invalid inputs") {
  auto bad_mass = deterministicMass();
  bad_mass(0, 0) = -10.0;
  REQUIRE_THROWS(franky::projectNullspaceTorque(
      deterministicJacobian(),
      bad_mass,
      franky::Vector7d::Ones(),
      franky::NullspaceProjectorType::kDynamicallyConsistent));

  auto bad_jacobian = deterministicJacobian();
  bad_jacobian(0, 0) = std::numeric_limits<double>::quiet_NaN();
  REQUIRE_THROWS(franky::projectNullspaceTorque(
      bad_jacobian,
      deterministicMass(),
      franky::Vector7d::Ones(),
      franky::NullspaceProjectorType::kDynamicallyConsistent));
}

TEST_CASE("Z-line fixture projects only x and y") {
  franky::CurveFixtureParams params;
  params.type = franky::CurveFixtureType::kZLine;
  params.origin = Eigen::Vector3d(0.4, -0.1, 2.0);
  const auto geometry = franky::projectCurveFixture(params, Eigen::Vector3d(0.43, -0.16, 0.7));
  REQUIRE(geometry.projected_point.isApprox(Eigen::Vector3d(0.4, -0.1, 0.7), 1e-14));
  REQUIRE(geometry.tangent.isApprox(Eigen::Vector3d::UnitZ(), 1e-14));
  REQUIRE(geometry.normal_error.isApprox(Eigen::Vector3d(-0.03, 0.06, 0.0), 1e-14));
}

TEST_CASE("YZ-circle fixture produces radial projection and orthogonal tangent") {
  franky::CurveFixtureParams params;
  params.type = franky::CurveFixtureType::kYZCircle;
  params.origin = Eigen::Vector3d(0.5, 0.1, 0.2);
  params.radius = 0.2;
  const auto geometry = franky::projectCurveFixture(params, Eigen::Vector3d(0.53, 0.22, 0.36));
  const Eigen::Vector3d radial = geometry.projected_point - params.origin;
  REQUIRE(radial.norm() == Approx(params.radius).margin(1e-14));
  REQUIRE(radial.dot(geometry.tangent) == Approx(0.0).margin(1e-14));
  REQUIRE(geometry.tangent.norm() == Approx(1.0).margin(1e-14));

  REQUIRE_THROWS(franky::projectCurveFixture(params, params.origin));
}

TEST_CASE("fixture wrench separates tangent and normal damping") {
  franky::CurveFixtureGeometry geometry;
  geometry.tangent = Eigen::Vector3d::UnitZ();
  geometry.normal_error = Eigen::Vector3d(0.02, -0.01, 0.0);
  const Eigen::Vector3d velocity(0.4, -0.3, 0.2);
  const auto force = franky::computeCurveFixtureForce(geometry, velocity, 100.0, 10.0, 3.0);
  const Eigen::Vector3d expected =
      100.0 * geometry.normal_error - 10.0 * Eigen::Vector3d(0.4, -0.3, 0.0) - 3.0 * Eigen::Vector3d(0.0, 0.0, 0.2);
  REQUIRE(force.isApprox(expected, 1e-14));
}

TEST_CASE("equal fixture damping matches projected-target Cartesian translation") {
  franky::CurveFixtureGeometry geometry;
  geometry.tangent = Eigen::Vector3d::UnitZ();
  geometry.normal_error = Eigen::Vector3d(0.02, -0.01, 0.0);
  const Eigen::Vector3d velocity(0.4, -0.3, 0.2);
  const auto force = franky::computeCurveFixtureForce(geometry, velocity, 100.0, 10.0, 10.0);
  REQUIRE(force.isApprox(100.0 * geometry.normal_error - 10.0 * velocity, 1e-14));
}

TEST_CASE("kinematic projector remains the default") {
  const franky::CartesianImpedanceBase::Params params;
  REQUIRE(params.nullspace_projector == franky::NullspaceProjectorType::kKinematic);
}

#include "franky/motion/m4_runtime_motion.hpp"
TEST_CASE("M4 stop restores former fixture tangent and orientation") {
  franky::Matrix6d stiffness=franky::Matrix6d::Identity()*40.;
  stiffness.topLeftCorner<3,3>()=Eigen::Matrix3d::Identity()*800.;
  franky::Matrix6d damping=franky::Matrix6d::Identity()*13.;
  damping.topLeftCorner<3,3>()=Eigen::Matrix3d::Identity()*56.56854249;
  franky::Vector6d error=franky::Vector6d::Zero(), twist=franky::Vector6d::Zero();
  error[2]=-.01; error[4]=.02;
  const auto spring=franky::computeM4PoseHoldWrench(stiffness,damping,error,twist);
  REQUIRE(spring[2]==Approx(8.));
  REQUIRE(spring[4]==Approx(-.8));
  twist[2]=-.1;
  const auto braking=franky::computeM4PoseHoldWrench(stiffness,damping,error,twist);
  REQUIRE(braking[2]>spring[2]);
  REQUIRE(braking.dot(twist)<0.);
}
TEST_CASE("M4 stop never declares moving deadline successful") {
  franky::M4StopProgress progress;
  franky::TorqueStopParams params;
  auto moving=franky::Vector7d::Zero().eval(); moving[1]=.20058;
  REQUIRE(progress.update(0.,moving,params)==0);
  REQUIRE(progress.update(1.999,moving,params)==0);
  REQUIRE(progress.update(2.,moving,params)==18);
}
TEST_CASE("M4 stop requires sustained rest after ramp") {
  franky::M4StopProgress progress;
  franky::TorqueStopParams params;
  const auto rest=franky::Vector7d::Zero().eval();
  auto moving=rest; moving[0]=.03;
  REQUIRE(progress.update(.1,rest,params)==0);
  REQUIRE(progress.update(.2,rest,params)==0);
  REQUIRE(progress.update(.25,moving,params)==0);
  REQUIRE(progress.update(.3,rest,params)==0);
  REQUIRE(progress.update(.399,rest,params)==0);
  REQUIRE(progress.update(.401,rest,params)==1);
  REQUIRE(progress.update(.5,moving,params)==0);
}
TEST_CASE("M4 stop ramp obeys command bounds and previous filtered torque slew") {
  franky::TorqueStopParams params;
  const auto initial=franky::Vector7d::Constant(2.).eval();
  const auto hold=franky::Vector7d::Constant(-8.).eval();
  const auto bounds=franky::Vector7d::Constant(4.).eval();
  const auto previous=franky::Vector7d::Constant(1.5).eval();
  REQUIRE(franky::blendM4StopTorque(initial,hold,0.,params,previous,bounds).isApprox(initial));
  REQUIRE(franky::blendM4StopTorque(initial,hold,.2,params,previous,bounds).isApprox(franky::Vector7d::Constant(.5)));
  REQUIRE(franky::blendM4StopTorque(initial,hold,.2,params,franky::Vector7d::Constant(-4.),bounds).isApprox(franky::Vector7d::Constant(-4.)));
}
namespace {
franky::M4RuntimeLimits runtimeLimits() {
  franky::M4RuntimeLimits l;
  l.joint_lower=franky::Vector7d::Constant(-1); l.joint_upper=franky::Vector7d::Constant(1);
  l.joint_velocity=franky::Vector7d::Constant(1); l.external_torque=franky::Vector7d::Constant(10);
  l.command_torque=franky::Vector7d::Constant(20);
  l.workspace_lower=Eigen::Vector3d::Constant(-1); l.workspace_upper=Eigen::Vector3d::Constant(1);
  return l;
}
franky::M4RuntimeSample runtimeSample() {
  franky::M4RuntimeSample s; s.min_singular_value=.1; s.dt_s=.001; s.command_success_rate=1; s.tick=101;
  return s;
}
}
TEST_CASE("M4 runtime thresholds reject invalid configuration") {
  auto l=runtimeLimits(); REQUIRE_NOTHROW(l.validate());
  l.joint_velocity[0]=0; REQUIRE_THROWS(l.validate());
  l=runtimeLimits(); l.reference_timeout_s=std::numeric_limits<double>::quiet_NaN(); REQUIRE_THROWS(l.validate());
  REQUIRE_THROWS(franky::M4RuntimeLimits{}.validate());
}
TEST_CASE("M4 native guard faults identify boundary violations") {
  const auto l=runtimeLimits(); auto s=runtimeSample();
  REQUIRE(franky::evaluateM4Limits(l,s,false,false)==0);
  s.reference_age_s=.101; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==1);
  s=runtimeSample(); s.dt_s=.011; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==3);
  s=runtimeSample(); s.q[0]=std::numeric_limits<double>::quiet_NaN(); REQUIRE(franky::evaluateM4Limits(l,s,false,false)==4);
  s=runtimeSample(); s.q[0]=1.01; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==5);
  s=runtimeSample(); s.dq[0]=1.01; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==6);
  s=runtimeSample(); s.pose.translation()[0]=1.01; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==7);
  s=runtimeSample(); s.reference_pose.translation()[0]=.031; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==8);
  s=runtimeSample(); s.twist[0]=.11; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==9);
  s=runtimeSample(); s.external_torque[0]=10.01; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==10);
  s=runtimeSample(); REQUIRE(franky::evaluateM4Limits(l,s,true,false)==11);
  s.min_singular_value=.019; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==12);
  s=runtimeSample(); REQUIRE(franky::evaluateM4Limits(l,s,false,true)==13);
  s.command_success_rate=.94; REQUIRE(franky::evaluateM4Limits(l,s,false,false)==14);
}
TEST_CASE("M4 stop retains displacement and workspace containment after original fault") {
  const auto limits=runtimeLimits(); auto sample=runtimeSample();
  const franky::Affine anchor=franky::Affine::Identity();
  REQUIRE(franky::evaluateM4StopLimits(limits,sample,anchor)==0);
  sample.pose.translation()[2]=-.031;
  REQUIRE(franky::evaluateM4StopLimits(limits,sample,anchor)==8);
  sample.pose.translation()[2]=-1.1;
  REQUIRE(franky::evaluateM4StopLimits(limits,sample,anchor)==7);
  sample=runtimeSample();sample.q[0]=1.01;
  REQUIRE(franky::evaluateM4StopLimits(limits,sample,anchor)==5);
}
TEST_CASE("M4 hand guidance may allow contact but retains external torque guard") {
  auto l=runtimeLimits(); l.abort_on_contact=false; auto s=runtimeSample();
  REQUIRE(franky::evaluateM4Limits(l,s,true,false)==0);
  s.external_torque[0]=11; REQUIRE(franky::evaluateM4Limits(l,s,true,false)==10);
}
TEST_CASE("M4 publication validates posture and rigid target before starting") {
  franky::M4RuntimeMotion motion(franky::CartesianImpedanceBase::Params{},runtimeLimits(),franky::TorqueStopParams{});
  franky::M4Reference ref; ref.sequence=1;
  REQUIRE_NOTHROW(motion.publish(ref)); REQUIRE(motion.drain().empty());
  ref.has_posture=true; ref.posture[0]=2; REQUIRE_THROWS(motion.publish(ref));
  ref=franky::M4Reference{}; ref.cartesian.target.linear()(0,0)=2; REQUIRE_THROWS(motion.publish(ref));
}

TEST_CASE("DLS accepts the initial zero period without allowing later zero periods") {
  REQUIRE(franky::validM4DlsPeriod(0.,1,.01));
  REQUIRE(franky::validM4DlsPeriod(.001,2,.01));
  REQUIRE_FALSE(franky::validM4DlsPeriod(0.,2,.01));
  REQUIRE_FALSE(franky::validM4DlsPeriod(-.001,1,.01));
  REQUIRE_FALSE(franky::validM4DlsPeriod(.011,2,.01));
  REQUIRE_FALSE(franky::validM4DlsPeriod(std::numeric_limits<double>::quiet_NaN(),1,.01));
  const franky::Vector7d q=franky::Vector7d::Constant(.2);
  const auto step=franky::computeM4DlsStep(franky::Jacobian::Identity(),franky::Vector6d::Ones(),
      franky::Vector6d::Zero(),franky::Vector7d::Zero(),franky::Vector7d::Ones(),franky::M4DlsParams{});
  REQUIRE((q+0.*step.velocity-q).norm()==0.);
}
TEST_CASE("DLS maps twist and uses an orthogonal secondary projector") {
  const auto j=deterministicJacobian();
  franky::M4DlsParams p; p.lambda=1e-8; p.posture_gain=1.; p.validate();
  const franky::Vector6d v=franky::Vector6d::Constant(.01);
  const auto step=franky::computeM4DlsStep(j,franky::Vector6d::Zero(),v,
      franky::Vector7d::Ones(),franky::Vector7d::Constant(100),p);
  REQUIRE(step.rank==6);
  REQUIRE((j*step.velocity-v).norm()<1e-10);
  REQUIRE((j*step.secondary_velocity).norm()<1e-12);
}
TEST_CASE("DLS scaled velocity limits and rank deficiency remain finite") {
  auto j=deterministicJacobian();j.row(5)=j.row(4);
  franky::M4DlsParams p;p.task_scale.tail<3>().setConstant(.2);p.validate();
  const auto step=franky::computeM4DlsStep(j,franky::Vector6d::Ones(),franky::Vector6d::Zero(),
      franky::Vector7d::Ones(),franky::Vector7d::Constant(.01),p);
  REQUIRE(step.rank==5); REQUIRE(step.velocity.allFinite());
  REQUIRE(step.velocity.cwiseAbs().maxCoeff()<=.0100000001);
  REQUIRE(step.velocity_scale<1);
  REQUIRE((j*step.secondary_velocity).norm()<1e-12);
}
TEST_CASE("Shared joint PD returns known torque without model or slew duplication") {
  const auto torque=franky::computeJointPdTorque(franky::Vector7d::Constant(100),
      franky::Vector7d::Constant(20),franky::Vector7d::Constant(.01),franky::Vector7d::Constant(-.02));
  REQUIRE((torque-franky::Vector7d::Constant(.6)).norm()<1e-12);
  franky::M4DlsParams p;p.lambda=0;REQUIRE_THROWS(p.validate());
}

TEST_CASE("DLS comparison constructor preserves legacy API and rejects fixture hybrids") {
  franky::CartesianImpedanceBase::Params params;
  franky::M4RuntimeMotion legacy(params,runtimeLimits(),franky::TorqueStopParams{});
  franky::M4DlsParams dls;
  REQUIRE_NOTHROW(franky::M4RuntimeMotion(params,runtimeLimits(),franky::TorqueStopParams{},std::nullopt,4096,dls));
  REQUIRE_THROWS(franky::M4RuntimeMotion(params,runtimeLimits(),franky::TorqueStopParams{},franky::CurveFixtureParams{},4096,dls));
  REQUIRE_THROWS(franky::M4RuntimeMotion(params,runtimeLimits(),franky::TorqueStopParams{},std::nullopt,4096,dls,
                                      franky::Matrix6d::Identity(),std::nullopt));
}
