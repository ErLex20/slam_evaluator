import math
import unittest

from ros2_openloris_publishers.imu_processing import StartupImuCalibrator


def make_calibrator(duration=1.0):
    return StartupImuCalibrator(
        duration=duration,
        minimum_acceleration_samples=5,
        minimum_gyro_samples=5,
        maximum_acceleration_stddev=0.15,
        maximum_gyro_stddev=0.02,
        maximum_gyro_mean=0.05,
        minimum_gravity=8.5,
        maximum_gravity=11.0,
    )


class TestStartupImuCalibrator(unittest.TestCase):

    def test_estimates_and_removes_stationary_offsets(self):
        calibrator = make_calibrator()
        for index in range(11):
            timestamp = index * 100_000_000
            variation = 0.01 if index % 2 else -0.01
            calibrator.add_acceleration(
                timestamp, (0.4 + variation, -0.3, 9.78))
            calibrator.add_angular_velocity(
                timestamp, (0.001, 0.003, -0.002 + variation * 0.01))

        calibration = calibrator.calibration
        self.assertIsNotNone(calibration)
        corrected_acceleration = calibration.correct_acceleration(
            calibration.gravity_vector)
        corrected_gyro = calibration.correct_angular_velocity(
            calibration.gyro_bias)

        for value in corrected_acceleration + corrected_gyro:
            self.assertTrue(math.isclose(value, 0.0, abs_tol=1.0e-12))
        self.assertTrue(math.isclose(
            calibration.gravity_vector[0], 0.4, abs_tol=0.002))
        self.assertTrue(math.isclose(
            calibration.gyro_bias[1], 0.003, abs_tol=1.0e-12))

    def test_does_not_accept_too_short_a_window(self):
        calibrator = make_calibrator(duration=2.0)
        for index in range(20):
            timestamp = index * 50_000_000
            calibrator.add_acceleration(timestamp, (0.0, 0.0, 9.8))
            calibrator.add_angular_velocity(timestamp, (0.0, 0.0, 0.0))

        self.assertIsNone(calibrator.calibration)

    def test_rejects_motion(self):
        calibrator = make_calibrator()
        for index in range(11):
            timestamp = index * 100_000_000
            acceleration = 1.0 if index % 2 else -1.0
            calibrator.add_acceleration(
                timestamp, (acceleration, 0.0, 9.8))
            calibrator.add_angular_velocity(
                timestamp, (0.0, 0.0, 0.0))

        self.assertIsNone(calibrator.calibration)
        self.assertIn('acceleration is changing',
                      calibrator.failure_reason)


if __name__ == '__main__':
    unittest.main()
