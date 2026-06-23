import math
import unittest

from ros2_openloris_publishers.geometry import relative_pose


class TestGeometry(unittest.TestCase):

    def test_relative_pose_moves_origin_to_identity(self):
        origin_position = (1.0, 2.0, 3.0)
        origin_orientation = (
            0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))

        position, orientation = relative_pose(
            origin_position,
            origin_orientation,
            origin_position,
            origin_orientation,
        )

        self.assertEqual(position, (0.0, 0.0, 0.0))
        self.assertEqual(orientation, (0.0, 0.0, 0.0, 1.0))

    def test_relative_pose_rotates_translation_into_origin_frame(self):
        position, _ = relative_pose(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
        )

        self.assertTrue(math.isclose(position[0], 1.0, abs_tol=1.0e-9))
        self.assertTrue(math.isclose(position[1], 0.0, abs_tol=1.0e-9))
        self.assertTrue(math.isclose(position[2], 0.0, abs_tol=1.0e-9))
