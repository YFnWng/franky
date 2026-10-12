# M4 dynamic projector and curve-fixture implementation plan

Date: 2026-10-11

## Branch and compatibility contract

- Fork: `git@github.com:YFnWng/franky.git`
- Base branch: `m4-hardware-controller`
- Implementation branch: `m4-dynamic-projector-fixtures`
- Base commit: `2c12acab` (`Add native joint velocity impedance tracking`)
- Upstream Franky commit: `f88f0e9b8f598d7c537b4e727e4411e67c95e017`
- Hardware libfranka: 0.21.2

The existing kinematic nullspace projector remains the default. Existing public
constructors and the tested joint/velocity impedance routes must retain their
current behavior. M4 additions are opt-in, are built into a new wheel version,
and do not require a libfranka modification.

The existing Franky/libfranka torque-processing path is also preserved:

```text
tau_raw = task + nullspace + joint_limit + coriolis + friction
tau_previous = robot_state.tau_J_d
tau_franky = tau_previous + clip(tau_raw - tau_previous, +/-max_delta_tau)
tau_command = low_pass(tau_franky, tau_previous, cutoff_frequency=100 Hz)
```

Franky performs the clipping and libfranka performs the filter. Hardware calls
keep `limit_rate=false` and `cutoff_frequency=100`. Simulation will be changed
to reproduce this recurrence.

## Change 1: selectable nullspace projector

Add a public enum with two modes:

```cpp
enum class NullspaceProjectorType {
  kKinematic,
  kDynamicallyConsistent,
};
```

Add these fields to `CartesianImpedanceBase::Params`:

```cpp
NullspaceProjectorType nullspace_projector{
    NullspaceProjectorType::kKinematic};
double dynamic_projector_rcond{1e-6};
```

Validation requires finite `0 < dynamic_projector_rcond < 1`. Bind both fields
to Python and expose the enum. Constructors continue selecting `kKinematic`
unless the caller requests the new mode.

Keep the existing kinematic implementation:

```text
tau_null = (I - J^+ J) tau_secondary
```

For the dynamic mode implement the M4 torque projector:

```text
A = J M^-1 J^T
tau_null = tau_secondary - J^T A^+ J M^-1 tau_secondary
```

Use fixed-size Eigen matrices and factored operations; do not form a dense 7x7
projector in the real-time path. Compute `M^-1 J^T` and `M^-1 tau_secondary`
with an LDLT solve. Since `A` is symmetric positive semidefinite, compute its
pseudoinverse with `SelfAdjointEigenSolver` and the relative cutoff
`dynamic_projector_rcond * max_eigenvalue`. Reject non-finite inputs, failed
decompositions and invalid spectra through the controller's fault path rather
than silently returning an unprojected torque.

Apply the selected projector to the sum of configured posture/manipulability
secondary torques. The M4 initial implementation uses only `PostureTask`, but
the projector semantics must be consistent for every configured secondary task.

Extract the pure fixed-size projection calculation into an independently
testable helper. It must not allocate after motion initialization.

## Change 2: native curve-fixture motion

Add `CartesianCurveImpedanceMotion`, derived from
`CartesianImpedanceBase`. It is a non-terminating torque motion and computes the
fixture reference from the current measured pose at every FCI callback. Python
only configures and starts it; Python does not perform the projection loop.

Public geometry:

```cpp
enum class CurveFixtureType { kZLine, kYZCircle };

struct CurveFixtureParams {
  CurveFixtureType type;
  Eigen::Vector3d origin;       // line point or circle center, base frame
  double radius;                // required and positive for circle
  Eigen::Quaterniond orientation;
  double normal_stiffness;
  double normal_damping;
  double tangent_damping;
  double center_epsilon{1e-6};
  double initialization_tolerance{1e-3};
};
```

The M4 fixtures are:

```text
Z line:
  target = [origin.x, origin.y, measured.z]
  tangent = [0, 0, 1]

YZ circle:
  radial = [0, measured.y-center.y, measured.z-center.z]
  target = center + radius * radial / norm(radial)
  tangent = [0, -unit_radial.z, unit_radial.y]
```

For the circle, reject `norm(radial) <= center_epsilon`. At initialization,
require the measured position to be within `initialization_tolerance` of the
configured curve. Always hold the configured full orientation. The hardware
adapter separately enforces the measured Cartesian workspace because both
fixtures are endpoint-free.

The desired translational wrench is

```text
e_normal = (I - tangent*tangent^T) * (target - measured_position)
v_tangent = tangent*tangent^T * measured_velocity
v_normal = measured_velocity - v_tangent
force = normal_stiffness * e_normal
        - normal_damping * v_normal
        - tangent_damping * v_tangent
```

The current M4 profile has equal normal/tangent damping (20 N-s/m), so this can
reuse the base controller with the projected target and zero desired linear
twist. Implement the general decomposition as a pure helper anyway so future
profiles with unequal damping do not change semantics. Reuse the base class for
orientation error, `J^T` mapping, Coriolis, selected posture projection,
joint-limit/friction options and Franky torque-rate limiting.

Publish a lock-free diagnostic snapshot for the logger containing fixture type,
measured point, projected point, tangent, normal error, measured normal/tangent
velocity, wrench and projection validity. Do not call Python or allocate memory
from the FCI callback.

## Source changes

Expected files:

```text
include/franky/motion/cartesian_impedance_base.hpp
src/motion/cartesian_impedance_base.cpp
include/franky/motion/cartesian_curve_impedance_motion.hpp       (new)
src/motion/cartesian_curve_impedance_motion.cpp                  (new)
include/franky.hpp
python/bind_motion_torque.cpp
franky/__init__.py
python/docstrings.hpp                                             (regenerated)
test/test-m4-controller.cpp                                      (new)
examples/cartesian_curve_fixture.py                              (new)
```

If exposing the decomposed fixture wrench cleanly requires a base-class hook,
add one narrow protected virtual/reference-wrench hook. Do not copy the entire
Cartesian torque controller into the fixture class.

## Offline tests

The new Catch2 test must cover:

1. Kinematic mode reproduces the existing projector to tight tolerance.
2. Dynamic mode reproduces the portable NumPy/Torch M4 fixture for deterministic
   full-rank `J,M,tau` inputs.
3. `J M^-1 tau_null` is near zero in dynamic mode.
4. Rank-deficient and ill-conditioned Jacobians use the declared relative
   cutoff and remain finite.
5. Invalid mass/decomposition and non-finite inputs fail closed.
6. Z-line projection and tangent are exact.
7. YZ-circle projection/tangent are exact and tangent is orthogonal to radial.
8. The circle center is rejected.
9. Equal-damping fixture output matches the simpler projected-target Cartesian
   impedance law.
10. Unequal normal/tangent damping matches the portable M4 wrench equation.
11. Kinematic default constructors remain behaviorally unchanged.

Add Python binding smoke tests for enum/config round trips and construction of
both fixture types. Then run the existing Franky unit/integration simulator
tests and the five M4 golden fixtures. Hardware is not involved in these tests.

## Integration order

1. Commit this plan on `m4-dynamic-projector-fixtures`.
2. Implement and test the pure projector helper.
3. Add the opt-in base parameter and Python binding; confirm the default remains
   kinematic.
4. Implement/test pure curve geometry and wrench helpers.
5. Add the native motion, diagnostics and Python binding.
6. Build a versioned Server 10/libfranka 0.21.2 wheel in a new environment.
7. Record wheel SHA-256, `ldd`, exact Git commit and offline fixture results.
8. Integrate the motion into the M4 hardware adapter with watchdog, workspace,
   joint/contact/timing guards and bounded native telemetry.
9. Perform read-only FCI validation, then the existing incremental hold,
   reduced-gain line and small-circle commissioning sequence.

## Definition of implementation complete

- Existing Franky behavior is unchanged unless dynamic projection or curve
  fixtures are explicitly requested.
- Pure C++ and Python binding tests pass.
- Dynamic projector and curve results match the M4 portable fixtures within the
  declared tolerance.
- No allocation, Python callback or blocking I/O occurs in the 1 kHz path.
- A new wheel is reproducibly built and identified by source commit and hash.
- Hardware adapter faults and telemetry distinguish projection failure,
  watchdog failure and robot/control failure.
- No physical motion is authorized solely by completion of this implementation.
