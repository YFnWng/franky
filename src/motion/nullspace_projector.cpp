#include "franky/motion/nullspace_projector.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <cmath>
#include <stdexcept>

namespace franky {

namespace {
constexpr double kKinematicRankTolerance = 1e-6;

void validateProjectionInputs(
    const Jacobian &jacobian, const Eigen::Matrix<double, 7, 7> &mass, const Vector7d &secondary_torque,
    double dynamic_rcond) {
  if (!jacobian.allFinite() || !mass.allFinite() || !secondary_torque.allFinite()) {
    throw std::runtime_error("Nullspace projection received a non-finite input.");
  }
  if (!std::isfinite(dynamic_rcond) || dynamic_rcond <= 0.0 || dynamic_rcond >= 1.0) {
    throw std::invalid_argument("dynamic_rcond must be finite and in (0, 1).");
  }
}
}  // namespace

Vector7d projectNullspaceTorque(
    const Jacobian &jacobian, const Eigen::Matrix<double, 7, 7> &mass, const Vector7d &secondary_torque,
    NullspaceProjectorType type, double dynamic_rcond) {
  validateProjectionInputs(jacobian, mass, secondary_torque, dynamic_rcond);

  if (type == NullspaceProjectorType::kKinematic) {
    Eigen::CompleteOrthogonalDecomposition<Jacobian> decomposition;
    decomposition.setThreshold(kKinematicRankTolerance);
    decomposition.compute(jacobian);
    const Eigen::Matrix<double, 7, 6> pseudoinverse = decomposition.pseudoInverse();
    const Vector7d result = secondary_torque - pseudoinverse * (jacobian * secondary_torque);
    if (!result.allFinite()) throw std::runtime_error("Kinematic nullspace projection failed.");
    return result;
  }

  Eigen::LDLT<Eigen::Matrix<double, 7, 7>> mass_ldlt(mass);
  if (mass_ldlt.info() != Eigen::Success || !mass_ldlt.isPositive()) {
    throw std::runtime_error("Dynamic nullspace projection requires a positive-definite mass matrix.");
  }

  const Eigen::Matrix<double, 7, 6> mass_inv_jacobian_transpose = mass_ldlt.solve(jacobian.transpose());
  const Vector7d mass_inv_secondary = mass_ldlt.solve(secondary_torque);
  if (mass_ldlt.info() != Eigen::Success || !mass_inv_jacobian_transpose.allFinite() ||
      !mass_inv_secondary.allFinite()) {
    throw std::runtime_error("Dynamic nullspace projection mass solve failed.");
  }

  Eigen::Matrix<double, 6, 6> task_inverse_mass = jacobian * mass_inv_jacobian_transpose;
  task_inverse_mass = 0.5 * (task_inverse_mass + task_inverse_mass.transpose()).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(task_inverse_mass);
  if (eigensolver.info() != Eigen::Success || !eigensolver.eigenvalues().allFinite()) {
    throw std::runtime_error("Dynamic nullspace projection eigendecomposition failed.");
  }

  const double maximum_eigenvalue = eigensolver.eigenvalues().maxCoeff();
  if (maximum_eigenvalue < 0.0) {
    throw std::runtime_error("Dynamic nullspace projection produced an invalid task-space spectrum.");
  }
  if (maximum_eigenvalue == 0.0) return secondary_torque;

  const double cutoff = dynamic_rcond * maximum_eigenvalue;
  Eigen::Matrix<double, 6, 1> inverse_eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < 6; ++i) {
    const double eigenvalue = eigensolver.eigenvalues()[i];
    if (eigenvalue < -cutoff) {
      throw std::runtime_error("Dynamic nullspace projection produced an indefinite task-space matrix.");
    }
    if (eigenvalue > cutoff) inverse_eigenvalues[i] = 1.0 / eigenvalue;
  }
  const Eigen::Matrix<double, 6, 6> task_inverse_mass_pinv =
      eigensolver.eigenvectors() * inverse_eigenvalues.asDiagonal() * eigensolver.eigenvectors().transpose();
  const Vector7d result =
      secondary_torque - jacobian.transpose() * task_inverse_mass_pinv * (jacobian * mass_inv_secondary);
  if (!result.allFinite()) throw std::runtime_error("Dynamic nullspace projection produced a non-finite torque.");
  return result;
}

}  // namespace franky
