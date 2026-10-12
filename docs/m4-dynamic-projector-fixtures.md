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

## Implementation status — 2026-10-11

The Franky library portion of this plan is implemented on this branch:

- `NullspaceProjectorType` keeps the kinematic projector as the default and adds
  the M4 mass-weighted projector with a relative eigenvalue cutoff.
- `CartesianCurveImpedanceMotion` evaluates Z-line and YZ-circle geometry,
  normal/tangent damping, fixed orientation and lock-free diagnostics inside
  the native control callback.
- Python exposes the configuration, diagnostics and pure numerical helpers.
- The transferred Z-line and YZ-circle golden fixtures match the compiled C++
  Python module to below `1.3e-15` maximum absolute error.
- The C++ test suite passes against libfranka 0.21.2 and Ruckig 0.17.3; a
  Python 3.12 binding smoke test also passes.

The example is dry-run by default. Hardware commissioning still requires the
separate runtime guards, reviewed configuration and staged procedure in
`franka-rl/deployment/MOTION_M4_HARDWARE_DEPLOYMENT_PLAN.md`.


## Hardware runtime and dependency preservation — 2026-10-12

The local working tree adds native `M4RuntimeMotion` watchdog/guard/stop dispatch and a
bounded 1 kHz telemetry ring, plus a hardware adapter and single-task runner
`deployment/franky_runtime/run_m4_hardware.py`. It supports pose/twist tracking,
measured-pose hold, native quintic posture references, and line/circle fixtures.
The first local candidate is `config/m4.hold.pending.yaml`: 10-second hold with
zero posture gains. It is pending local review, not automatically authorized.

The new wheel `2.0.1.dev63+gm4baseline.libfranka.0.21.2` bundles byte-identical
copies of all 21 native dependencies from the tested RL velocity runtime,
including libfranka and Ruckig. The earlier dev62 dependency rebuild is
superseded for commissioning. Installed environment:
`~/yifan/venvs/franky-m4-baseline-server10`. Artifact and dependency/source hash
receipt: `~/yifan/deployment_bundles/2026-10-12/franky-m4-baseline/`.

Native tests (13 cases/41 assertions), installed binding/adapter tests (9),
portable five-fixture replay and native curve helper parity passed offline.
There has been no FCI connection or hardware motion. Full compiled tracking/
posture torque replay and physical timing/stop commissioning remain unvalidated;
the simulation torque-processing correction requested above also remains open.
Detailed commands, telemetry semantics and guard provenance are in
`deployment/franky_runtime/M4_HARDWARE_PREP.md`.


## M4 stop v2 implementation (2026-10-12)

The user requested correction of the Z-line stopping failure. M4-only native
stop now captures measured encoder/FK pose and current joints when stopping
begins. It ignores subsequent publisher references and replaces the fixture's
free tangent with full-six-axis Cartesian hold using existing controller gains
(K800 translation/K40 rotation for this Z-line config), zero desired twist,
inherited Coriolis/joint-limit processing and additional configured joint damping.
A smoothstep ramp over the existing 0.2 seconds blends from actual prior
filtered tau_J_d into hold torque, then applies command bounds and the existing
1 Nm/tick slew; libfranka still filters at 100 Hz. No gravity added by client.
Regular guidance gains and immutable simulation bundle remain unchanged.

Normal MotionFinished requires all joints below 0.02 rad/s continuously for
100 ms after the ramp. Two-second deadline without sustained rest is stop
fault18 and a native control exception, not MotionFinished while moving.
Stop position/orientation containment uses 3 cm / 0.2 rad about captured stop
pose; joint and workspace bounds continue to apply. Model/nonfinite failure,
collision/robot errors or containment failure abort control explicitly.
A native abort may result in a stiff robot; it is not a compliant guidance
mode, and firmware behavior after FCI exits is not controlled by this code.
Pose hold under continued external force is not an assured collision-free stop.

Separate stop_fault preserves evidence even when original fault9 remains
latched. Python catches join exceptions, drains final telemetry and reports
stop_evidence/stop_fault without labeling an abort a smooth stop. Stop logs
include extra joint damping and identify raw torque as pre-transition hold
output. Standalone Franky TorqueStopMotion and existing velocity controller
are unchanged; only M4RuntimeMotion has this stop policy.

New isolated runtime: `~/yifan/venvs/franky-m4-stop-server10`, Franky
`2.0.1.dev64+gm4stop.libfranka.0.21.2`. All 21 native dependency files must
match baseline byte-for-byte; tested baseline environment is retained.
Z-line approved config now requires stop_behavior_version2 and the new
Franky version. `config/m4.stop-check.approved.yaml` is a five-second
measured-pose hold with the same Cartesian gains, no trajectory/homing or
hand guidance, for first stationary commissioning of the new stop.
Old circle/posture configs remain tied to their original runtime.

Native deterministic tests passed 18 cases/62 assertions, including former
fixture tangent restoring force, orientation restoring force, opposing braking
force, bounds/slew, low-speed dwell with reset, moving deadline and containment.
These tests do not prove physical braking or human-contact performance.
No robot access or motion performed during this fix. Verify stationary stop
first, then limited gentle Z guidance with release of hand force when stopping.


## Comparison implementation delivered — 2026-10-12

See `deployment/franky_runtime/M4_COMPARISONS.md` for commands and current
contracts. Native optional DLS route added to M4RuntimeMotion, sharing the tested
watchdog/guards/ring log and captured-pose stop-v2. Shared joint PD algebra is
reused with JointImpedanceBase. Separate configured Cartesian stop gains ensure
active gain sweeps do not silently vary braking gains. No libfranka/Ruckig/
firmware/dependency changes: isolated franky-m4-comparison-server10, local
Franky2.0.1.dev65+gm4compare.libfranka.0.21.2, all21 bundled dependency hashes
identical to retained stop-server10. Wheel/receipts under deployment_bundles/
2026-10-12/franky-m4-comparison.

User requested each session automatically repeat3 times: runner now defaults3
(or experiment.repeats), one start,5s interval, fresh live checks, individual
trial folders. Fault, failed stop, logging loss, operator stop or repeat-start
drift aborts continuation; no auto recovery/homing. --repeats1 for commissioning.
Posture targets frozen within the session; paired sessions still require target
and start matching from recorded plans. Existing executable configs are not
silently replaced by new comparison candidates.

22 candidate profiles generated, plus local single-profile executable-copy
selector. Phase events tied to applied reference sequence, joint/DLS telemetry,
phase/contour/posture/torque/settling metrics and repeat-aware report aggregator
implemented. Existing historical runs exercised into comparison-baseline-report
with indexed JSON/CSV, Markdown and PNG/PDF figures; exploratory evidence remains
labelled as such. Tests42 Python and22 native cases/75 assertions passed.
No FCI access or hardware motion performed. DLS composed route needs first
hardware hold, then one circle trial before three-trial comparisons. Portable
DLS law at source/franka_motion/franka_motion/dls_tracking.py is available to the
simulation workstation; native/portable numerical parity tested.

## Comparison startup fix and final validation — 2026-10-12

The deployed comparison build is
`2.0.1.dev66+gm4comparefix.libfranka.0.21.2`. DLS accepts libfranka's
zero-duration first callback and performs zero integration on that tick;
subsequent callback periods must be positive and within the configured bound.
This fixes the initial controller_projection fault without changing the
controller law or bundled dependencies.

Before committing the runtime changes, `cmake --build build-m4-021 --target
test-m4-controller -j2` succeeded and `./build-m4-021/test-m4-controller`
passed all 23 test cases / 82 assertions. These checks are offline and do not
connect to the robot.
