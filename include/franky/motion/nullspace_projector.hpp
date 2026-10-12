#pragma once

#include "franky/types.hpp"

namespace franky {

/** Selects how a secondary joint torque is projected away from a Cartesian task. */
enum class NullspaceProjectorType {
  /** Euclidean/kinematic projector: (I - J^+ J) tau. */
  kKinematic,
  /** Mass-weighted torque projector: tau - J^T (J M^-1 J^T)^+ J M^-1 tau. */
  kDynamicallyConsistent,
};

/**
 * Project a secondary joint torque into the nullspace of a Cartesian task.
 *
 * This function uses fixed-size Eigen storage. It throws std::runtime_error for
 * non-finite inputs, an invalid mass matrix, or a failed eigendecomposition.
 */
[[nodiscard]] Vector7d projectNullspaceTorque(
    const Jacobian &jacobian, const Eigen::Matrix<double, 7, 7> &mass, const Vector7d &secondary_torque,
    NullspaceProjectorType type, double dynamic_rcond = 1e-6);

}  // namespace franky
