# DROID native joint-impedance candidate

Offline development only. This adds `DroidJointImpedanceMotion` alongside the
existing tracking motions; no existing M4/PPO configuration or wheel is replaced.
It derives from `JointImpedanceBase` and preserves its gain shaping and torque
processing. The only shared-base addition is a default no-op observation hook
immediately before Franky's slew limiter, after all compensation/limit terms.
The new motion uses that hook to guard the raw total non-gravity command.

## Parameters and references

Construction requires explicit `JointImpedanceParams`, `DroidJointLimits`,
`TorqueStopParams`, log capacity and gain-update time constant. Joint and Cartesian
stiffness/damping must all be explicit. Native clipping is configured by the
parameter struct before construction, not by mutating returned params. There are
no runnable commissioned thresholds in this document.

`publish(q, row_sequence, expires_ns)` accepts seven absolute positions and uses
zero desired velocity/feedforward. It validates finite positions, native joint
bounds, monotonic nonzero row identity and an absolute future deadline. Reusing a
row identity with a different target is rejected. The native publisher stamps
`CLOCK_MONOTONIC` time and increments a separate publication counter; its mailbox
is serialized on the non-RT writer side. The servo reads without that writer lock.

Refresh publication from the live executor more frequently than its commissioned
publisher timeout, even while the 15 Hz row stays unchanged. Do not refresh from
an independent timer that masks a dead executor. Retain the row's original expiry;
a heartbeat must never extend the admitted chunk/hold deadline. Publishing is not
an acknowledgement. Latest-value mailboxes can coalesce updates, so the runtime
must supervise first-row consumption and detect skipped required rows.

## Consumption and telemetry

`status().consumed` contains first successful native computation for a row:
`row_sequence`, `publication_sequence`, native `tick`, `monotonic_ns`. It updates
only after computing a finite, torque-guard-compliant tracking command. It remains
unchanged for same-row heartbeat refreshes and on a faulting callback. This is
callback consumption evidence, not an actuator/network acknowledgement. Match the
row identity before emitting any service-level `applied` receipt. Multiple rows
may execute between Python polls: drain telemetry rather than assuming the latest
status proves every preceding row executed.

`drain()` is for exactly one non-RT consumer. Its bounded preallocated SPSC ring
records q/dq/reference, raw pre-slew and callback outgoing torque, receipt, timing,
fault and stop evidence. Outgoing torque precedes libfranka filtering; it is not a
measured actuator command. Overflow increments a dropped counter and latches a
fault; the separate status triple buffer preserves terminal evidence even if the
ring is full. No serialization or file I/O occurs in the callback. Do not attach
Python callbacks/reactions to this candidate; inherited generic callback facilities
are not a substitute for native guards or a qualified logging path.

## Guards, latching and stop behavior

Fault codes: 1 publisher timeout/deadline, 2 publication/row sequence, 3 callback
gap/period, 4 nonfinite state/command, 5 target/measured joint bounds, 6 measured
joint velocity, 7 raw/previous command torque, 8 robot error, 9 contact/collision,
10 telemetry overflow, 11 stop timeout, 12 failed control/stop computation.
Bounds are guards, not a motion planner or a total-torque clamp. First fault wins;
there is no reset method. Motion instances are single-session objects.

A request or software guard transitions into the historical joint damping stop:
linearly blend inherited `tau_J_d` into `-D_stop*dq + coriolis` with explicit ramp
and slew parameters. No position locking spring is introduced. This is a candidate
reuse of the old torque-stop method, **not a universal safe-stop guarantee**.
Sustained rest requires 100 ms below the explicit velocity threshold after the
ramp. At the deadline, the loop finishes with `stop_fault=11` and
`stop_at_rest=false`; it must not be accepted as successful stopping. Nonfinite
feedback, robot errors, contact/collision, failed model computation or a stop
command exceeding the configured torque guard abort the software loop with
`stop_fault=12`. Terminal evidence is published before throwing. There is no
automatic robot recovery, homing or new session admission.

The service still needs to join the native motion and combine `joined` with the
native stop-finished/rest/fault/dropped evidence before `finish_stop`. A failed
libfranka loop may stop before another callback: service-side exception/join
handling remains necessary. Workspace, tool collision, external-torque thresholds,
command success monitoring and per-unit stop qualification remain runtime integration
and commissioning work; these joint guards alone do not qualify a robot session.

## Offline tests and build

`test/test-droid-runtime.cpp` exercises the same time-injected state machine used
by the motion plus the actual motion callback flow with injected time and a fake
torque computation (no model/network/robot). The tests cover producer death,
identity/deadline rejection, raw-torque and measured-state guards, post-computation
receipts, J7, latched faults, telemetry overflow, sustained rest and stop timeout.
They do not qualify model torque output, FCI communication or physical stopping.

Build only into a project directory with explicit read-only dependency prefixes.
Do not install into or rebuild libfranka/Ruckig/original baselines. On this host the
candidate build/validation is under `~/yifan/droid-deployment/`; the existing
modified dev66 and original wheels remain untouched. Backend/service injection,
versioned physical config, hand worker and packaging/qualification are separate
steps; the DROID service remains mock-only until those are implemented and tested.

## Current raw-build validation boundary

This host's raw build uses read-only libfranka 0.21.2 and Ruckig 0.17.3 prefixes
from the frozen server10 family. It is not an installed/repaired wheel and does
not claim binary dependency identity with dev66's bundled libraries. Tests select
those prefixes explicitly before ROS dependency directories for their process;
otherwise an inherited ROS path can resolve a different `libruckig.so`. The
candidate receipt records resolved library hashes. No global environment, native
install or frozen file is modified. Packaging must preserve/review and pin the
intended dependency binaries before any physical backend uses the candidate.

Offline validation: 11 DROID C++ cases (47 assertions), 23 existing M4 C++ cases
(82 assertions), and the runtime/contracts suite (148 passed, 2 M4/PPO-contract
skips). Python native-constructor checks and build/source hashes are retained in
`~/yifan/droid-deployment/validation/`. The existing dev66 wheel's 23 native hashes
were checked again and remain unchanged.
