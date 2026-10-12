import numpy as np

import franky


def test_dynamic_projector_parameter_round_trip():
    params = franky.CartesianImpedanceParams()
    assert params.nullspace_projector == franky.NullspaceProjectorType.Kinematic
    params.nullspace_projector = franky.NullspaceProjectorType.DynamicallyConsistent
    params.dynamic_projector_rcond = 1e-5
    assert params.nullspace_projector == franky.NullspaceProjectorType.DynamicallyConsistent
    assert params.dynamic_projector_rcond == 1e-5


def test_curve_fixture_binding_and_construction():
    fixture = franky.CurveFixtureParams()
    fixture.type = franky.CurveFixtureType.YZCircle
    fixture.origin = np.array([0.3, 0.0, 0.5])
    fixture.radius = 0.05
    fixture.orientation = np.array([1.0, 0.0, 0.0, 0.0])

    controller = franky.CartesianImpedanceParams()
    controller.nullspace_projector = (
        franky.NullspaceProjectorType.DynamicallyConsistent
    )
    motion = franky.CartesianCurveImpedanceMotion(fixture, controller)

    geometry = franky.project_curve_fixture(
        fixture, np.array([0.3, 0.0, 0.55])
    )
    np.testing.assert_allclose(geometry.projected_point, [0.3, 0.0, 0.55])
    np.testing.assert_allclose(motion.fixture_params.orientation, [1.0, 0.0, 0.0, 0.0])
    assert (
        motion.params.nullspace_projector
        == franky.NullspaceProjectorType.DynamicallyConsistent
    )
