import math

import numpy as np

from ros2_grandtour_publishers import alignment, geometry


def test_nearest_timestamp_matches_filters_distant_queries():
    query_indices, reference_indices, deltas = (
        alignment.nearest_timestamp_matches(
            [0.01, 1.02, 3.0], [0.0, 1.0, 2.0], 0.05))

    np.testing.assert_array_equal(query_indices, [0, 1])
    np.testing.assert_array_equal(reference_indices, [0, 1])
    np.testing.assert_allclose(deltas, [0.01, 0.02])


def test_fit_rigid_transform_recovers_rotation_and_translation():
    angle = math.radians(35.0)
    expected_rotation = (
        (math.cos(angle), -math.sin(angle), 0.0),
        (math.sin(angle), math.cos(angle), 0.0),
        (0.0, 0.0, 1.0),
    )
    expected_translation = np.array([2.0, -3.0, 0.75])
    source = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 2.0, 0.0],
        [0.0, 0.0, 3.0],
        [1.0, 2.0, 3.0],
    ])
    target = source @ np.asarray(expected_rotation).T + expected_translation

    translation, quaternion, rmse = alignment.fit_rigid_transform(
        source, target)

    np.testing.assert_allclose(translation, expected_translation, atol=1e-12)
    transformed = np.array([
        np.asarray(geometry.rotate_vector(quaternion, point)) + translation
        for point in source
    ])
    np.testing.assert_allclose(transformed, target, atol=1e-12)
    assert rmse < 1.0e-12
