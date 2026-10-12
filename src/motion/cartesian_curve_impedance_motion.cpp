#include "franky/motion/cartesian_curve_impedance_motion.hpp"

#include <cmath>
#include <stdexcept>

namespace franky {

namespace {
Affine fixtureTarget(const CurveFixtureParams &params, const Eigen::Vector3d &position) {
  Affine target = Affine::Identity();
  target.linear() = params.orientation.normalized().toRotationMatrix();
  target.translation() = position;
  return target;
}
}  // namespace

void CurveFixtureParams::validate() const {
  if (!origin.allFinite() || !orientation.coeffs().allFinite()) {
    throw std::invalid_argument("Curve fixture origin and orientation must be finite.");
  }
  const double quaternion_norm = orientation.norm();
  if (!std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1e-6) {
    throw std::invalid_argument("Curve fixture orientation must be a unit quaternion.");
  }
  if (type == CurveFixtureType::kYZCircle && (!std::isfinite(radius) || radius <= 0.0)) {
    throw std::invalid_argument("A YZ-circle fixture requires a finite positive radius.");
  }
  for (const auto &[value, name] :
       {std::pair{normal_stiffness, "normal_stiffness"},
        std::pair{normal_damping, "normal_damping"},
        std::pair{tangent_damping, "tangent_damping"},
        std::pair{center_epsilon, "center_epsilon"},
        std::pair{initialization_tolerance, "initialization_tolerance"}}) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(std::string(name) + " must be finite and non-negative.");
    }
  }
  if (center_epsilon == 0.0) throw std::invalid_argument("center_epsilon must be positive.");
}

CurveFixtureGeometry projectCurveFixture(const CurveFixtureParams &params, const Eigen::Vector3d &measured_point) {
  params.validate();
  if (!measured_point.allFinite()) throw std::runtime_error("Curve projection received a non-finite point.");

  CurveFixtureGeometry geometry;
  if (params.type == CurveFixtureType::kZLine) {
    geometry.projected_point = Eigen::Vector3d(params.origin.x(), params.origin.y(), measured_point.z());
    geometry.tangent = Eigen::Vector3d::UnitZ();
  } else {
    const Eigen::Vector3d radial(0.0, measured_point.y() - params.origin.y(), measured_point.z() - params.origin.z());
    const double radial_norm = radial.norm();
    if (!std::isfinite(radial_norm) || radial_norm <= params.center_epsilon) {
      throw std::runtime_error("YZ-circle projection is undefined at the circle center.");
    }
    const Eigen::Vector3d unit_radial = radial / radial_norm;
    geometry.projected_point = params.origin + params.radius * unit_radial;
    geometry.tangent = Eigen::Vector3d(0.0, -unit_radial.z(), unit_radial.y());
  }
  geometry.normal_error = geometry.projected_point - measured_point;
  geometry.distance = geometry.normal_error.norm();
  return geometry;
}

Eigen::Vector3d computeCurveFixtureForce(
    const CurveFixtureGeometry &geometry, const Eigen::Vector3d &measured_velocity, double normal_stiffness,
    double normal_damping, double tangent_damping) {
  if (!geometry.projected_point.allFinite() || !geometry.tangent.allFinite() || !geometry.normal_error.allFinite() ||
      !measured_velocity.allFinite() || !std::isfinite(normal_stiffness) || !std::isfinite(normal_damping) ||
      !std::isfinite(tangent_damping) || normal_stiffness < 0.0 || normal_damping < 0.0 || tangent_damping < 0.0) {
    throw std::runtime_error("Curve fixture wrench received an invalid input.");
  }
  const Eigen::Vector3d tangent_velocity = geometry.tangent * geometry.tangent.dot(measured_velocity);
  const Eigen::Vector3d normal_velocity = measured_velocity - tangent_velocity;
  return normal_stiffness * geometry.normal_error - normal_damping * normal_velocity -
         tangent_damping * tangent_velocity;
}

CartesianCurveImpedanceMotion::CartesianCurveImpedanceMotion(
    const CurveFixtureParams &fixture_params, const CartesianImpedanceBase::Params &controller_params,
    double gains_time_constant)
    : CartesianImpedanceBase(
          fixtureTarget(fixture_params, fixture_params.origin), controller_params, gains_time_constant),
      fixture_params_(fixture_params) {
  fixture_params_.validate();
}

void CartesianCurveImpedanceMotion::initImpl(
    const RobotState &robot_state, const std::optional<franka::Torques> &previous_command) {
  (void)previous_command;
  current_geometry_ = projectCurveFixture(fixture_params_, robot_state.O_T_EE.translation());
  if (current_geometry_.distance > fixture_params_.initialization_tolerance) {
    throw std::runtime_error("Measured end-effector position is outside the curve fixture initialization tolerance.");
  }
  target_ = fixtureTarget(fixture_params_, current_geometry_.projected_point);
}

franka::Torques CartesianCurveImpedanceMotion::nextCommandImpl(
    const RobotState &robot_state, franka::Duration time_step, franka::Duration rel_time, franka::Duration abs_time,
    const std::optional<franka::Torques> &previous_command) {
  (void)rel_time;
  (void)abs_time;
  (void)previous_command;
  current_geometry_ = projectCurveFixture(fixture_params_, robot_state.O_T_EE.translation());
  target_ = fixtureTarget(fixture_params_, current_geometry_.projected_point);
  CartesianReference reference;
  reference.target = target_;
  reference.target_twist = Twist();
  return computeCommand(robot_state, reference, time_step.toSec());
}

Vector6d CartesianCurveImpedanceMotion::computeCartesianWrench(
    const RobotState &robot_state, const CartesianReference &reference, const Vector6d &error,
    const Vector6d &measured_twist, const Vector6d &desired_twist) {
  Vector6d wrench =
      CartesianImpedanceBase::computeCartesianWrench(robot_state, reference, error, measured_twist, desired_twist);
  wrench.head<3>() = computeCurveFixtureForce(
      current_geometry_,
      measured_twist.head<3>(),
      fixture_params_.normal_stiffness,
      fixture_params_.normal_damping,
      fixture_params_.tangent_damping);

  CurveFixtureDiagnostics diagnostics;
  diagnostics.type = fixture_params_.type;
  diagnostics.measured_point = robot_state.O_T_EE.translation();
  diagnostics.projected_point = current_geometry_.projected_point;
  diagnostics.tangent = current_geometry_.tangent;
  diagnostics.normal_error = current_geometry_.normal_error;
  diagnostics.measured_tangent_velocity =
      current_geometry_.tangent * current_geometry_.tangent.dot(measured_twist.head<3>());
  diagnostics.measured_normal_velocity = measured_twist.head<3>() - diagnostics.measured_tangent_velocity;
  diagnostics.wrench = wrench;
  diagnostics.projection_valid = true;
  diagnostics_.setUnsafe(diagnostics);
  return wrench;
}

}  // namespace franky
