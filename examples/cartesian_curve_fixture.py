"""Configure Franky's native 1 kHz M4 curve fixture.

This example is dry-run by default.  It prints the resolved controller setup
without opening FCI.  Passing --execute starts a fixture through the measured
current flange pose; use only as part of a separately reviewed hardware run.
"""

from argparse import ArgumentParser

import numpy as np

from franky import (
    CartesianCurveImpedanceMotion,
    CartesianImpedanceParams,
    CurveFixtureParams,
    CurveFixtureType,
    NullspaceProjectorType,
    PostureTask,
    Robot,
    TorqueSafetyParams,
    TorqueStopMotion,
)


FR3_LOWER = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
FR3_UPPER = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])


def main():
    parser = ArgumentParser()
    parser.add_argument("--host", default="172.16.0.2")
    parser.add_argument("--kind", choices=("z-line", "yz-circle"), default="z-line")
    parser.add_argument("--radius", type=float, default=0.05)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    print("Native fixture: feedback projection and torque control run in C++ at the FCI rate.")
    print("Projector: dynamically consistent; K_n=100 N/m, D_n=D_t=20 N s/m.")
    if not args.execute:
        print("Dry run only. Pass --execute after completing the hardware preflight.")
        return

    robot = Robot(args.host)
    pose = robot.current_pose.end_effector_pose
    position = np.asarray(pose.translation, dtype=float)
    q = np.asarray(robot.current_joint_positions, dtype=float)

    fixture = CurveFixtureParams()
    fixture.type = CurveFixtureType.ZLine if args.kind == "z-line" else CurveFixtureType.YZCircle
    fixture.origin = position.copy()
    if args.kind == "yz-circle":
        fixture.origin[2] -= args.radius
        fixture.radius = args.radius
    fixture.orientation = np.asarray(pose.quaternion, dtype=float)
    fixture.normal_stiffness = 100.0
    fixture.normal_damping = 20.0
    fixture.tangent_damping = 20.0
    fixture.initialization_tolerance = 0.001

    controller = CartesianImpedanceParams()
    controller.nullspace_projector = NullspaceProjectorType.DynamicallyConsistent
    controller.dynamic_projector_rcond = 1e-6
    controller.nullspace_tasks = [PostureTask(q, 2.0, 1.0)]
    safety = TorqueSafetyParams()
    safety.lower_joint_limits = FR3_LOWER
    safety.upper_joint_limits = FR3_UPPER
    safety.max_delta_tau = 1.0
    controller.safety = safety

    motion = CartesianCurveImpedanceMotion(fixture, controller)
    try:
        robot.move(motion, asynchronous=False, limit_rate=False, cutoff_frequency=100.0)
    except KeyboardInterrupt:
        robot.move(
            TorqueStopMotion(), asynchronous=False, limit_rate=False, cutoff_frequency=100.0
        )


if __name__ == "__main__":
    main()
