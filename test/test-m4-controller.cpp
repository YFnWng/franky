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
