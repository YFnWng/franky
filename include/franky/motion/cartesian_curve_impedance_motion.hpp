#pragma once

#include <Eigen/Geometry>
#include <optional>

#include "franky/motion/cartesian_impedance_base.hpp"
#include "franky/wait_free_triple_buffer.hpp"

namespace franky {

enum class CurveFixtureType { kZLine, kYZCircle };

struct CurveFixtureParams {
  CurveFixtureType type{CurveFixtureType::kZLine};
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  double radius{0.0};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  double normal_stiffness{500.0};
  double normal_damping{20.0};
  double tangent_damping{20.0};
  double center_epsilon{1e-6};
  double initialization_tolerance{1e-3};

  void validate() const;
};

struct CurveFixtureGeometry {
  Eigen::Vector3d projected_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tangent{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d normal_error{Eigen::Vector3d::Zero()};
  double distance{0.0};
};

/** Pure curve projection used by the native controller and offline tests. */
[[nodiscard]] CurveFixtureGeometry projectCurveFixture(
    const CurveFixtureParams &params, const Eigen::Vector3d &measured_point);

/** Pure fixture translational impedance law. */
[[nodiscard]] Eigen::Vector3d computeCurveFixtureForce(
    const CurveFixtureGeometry &geometry, const Eigen::Vector3d &measured_velocity, double normal_stiffness,
    double normal_damping, double tangent_damping);

struct CurveFixtureDiagnostics {
  CurveFixtureType type{CurveFixtureType::kZLine};
  Eigen::Vector3d measured_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d projected_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tangent{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d normal_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d measured_normal_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d measured_tangent_velocity{Eigen::Vector3d::Zero()};
  Vector6d wrench{Vector6d::Zero()};
  bool projection_valid{false};
};

/**
 * Native non-terminating Cartesian impedance motion constrained to a Z line
 * or a circle in the base-frame YZ plane.
 */
class CartesianCurveImpedanceMotion : public CartesianImpedanceBase {
 public:
  CartesianCurveImpedanceMotion(
      const CurveFixtureParams &fixture_params, const CartesianImpedanceBase::Params &controller_params = {},
      double gains_time_constant = 0.1);

  [[nodiscard]] const CurveFixtureParams &fixtureParams() const { return fixture_params_; }
  [[nodiscard]] const CartesianImpedanceBase::Params &params() const { return base_params(); }

  /** Return the latest complete real-time diagnostic sample. */
  [[nodiscard]] CurveFixtureDiagnostics getDiagnostics() { return diagnostics_.get(); }

 protected:
  void initImpl(const RobotState &robot_state, const std::optional<franka::Torques> &previous_command) override;
  franka::Torques nextCommandImpl(
      const RobotState &robot_state, franka::Duration time_step, franka::Duration rel_time, franka::Duration abs_time,
      const std::optional<franka::Torques> &previous_command) override;
  Vector6d computeCartesianWrench(
      const RobotState &robot_state, const CartesianReference &reference, const Vector6d &error,
      const Vector6d &measured_twist, const Vector6d &desired_twist) override;

 private:
  CurveFixtureParams fixture_params_;
  CurveFixtureGeometry current_geometry_;
  WaitFreeTripleBuffer<CurveFixtureDiagnostics> diagnostics_;
};

}  // namespace franky
