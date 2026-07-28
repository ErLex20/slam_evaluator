"""Dependency-free processing helpers for the OpenLORIS IMU adapter."""

from dataclasses import dataclass
import math


def vector_norm(vector):
    """Return the Euclidean norm of a three-dimensional vector."""
    return math.sqrt(sum(value * value for value in vector))


def vector_mean(vectors):
    """Return the component-wise mean of a non-empty vector sequence."""
    count = len(vectors)
    return tuple(
        sum(vector[index] for vector in vectors) / count
        for index in range(3)
    )


def vector_stddev(vectors, mean):
    """Return the population standard deviation of each component."""
    count = len(vectors)
    return tuple(
        math.sqrt(
            sum(
                (vector[index] - mean[index]) ** 2
                for vector in vectors
            ) / count
        )
        for index in range(3)
    )


@dataclass(frozen=True)
class ImuCalibration:
    """Bias estimates obtained while the platform is stationary."""

    gravity_vector: tuple
    gyro_bias: tuple
    acceleration_stddev: tuple
    gyro_stddev: tuple
    acceleration_samples: int
    gyro_samples: int

    def correct_acceleration(self, vector):
        """Remove the measured stationary gravity/acceleration offset."""
        return tuple(
            vector[index] - self.gravity_vector[index]
            for index in range(3)
        )

    def correct_angular_velocity(self, vector):
        """Remove the stationary gyroscope bias."""
        return tuple(
            vector[index] - self.gyro_bias[index]
            for index in range(3)
        )


class StartupImuCalibrator:
    """Find a stationary window and estimate gravity and gyro bias.

    Acceleration and gyroscope samples may arrive at unrelated rates. A
    calibration is accepted only when both streams cover the requested time
    span and their variation is consistent with a stationary platform.
    """

    def __init__(
        self,
        duration,
        minimum_acceleration_samples,
        minimum_gyro_samples,
        maximum_acceleration_stddev,
        maximum_gyro_stddev,
        maximum_gyro_mean,
        minimum_gravity,
        maximum_gravity,
    ):
        self.duration_ns = int(duration * 1.0e9)
        self.minimum_acceleration_samples = minimum_acceleration_samples
        self.minimum_gyro_samples = minimum_gyro_samples
        self.maximum_acceleration_stddev = maximum_acceleration_stddev
        self.maximum_gyro_stddev = maximum_gyro_stddev
        self.maximum_gyro_mean = maximum_gyro_mean
        self.minimum_gravity = minimum_gravity
        self.maximum_gravity = maximum_gravity

        self.acceleration_samples = []
        self.gyro_samples = []
        self.calibration = None
        self.failure_reason = None

    def add_acceleration(self, timestamp_ns, vector):
        """Add an acceleration sample expressed in the output frame."""
        self.acceleration_samples.append((timestamp_ns, tuple(vector)))
        return self._try_calibrate()

    def add_angular_velocity(self, timestamp_ns, vector):
        """Add a gyroscope sample expressed in the output frame."""
        self.gyro_samples.append((timestamp_ns, tuple(vector)))
        return self._try_calibrate()

    def _try_calibrate(self):
        if self.calibration is not None:
            return self.calibration
        if not self.acceleration_samples or not self.gyro_samples:
            return None

        window_end = min(
            self.acceleration_samples[-1][0],
            self.gyro_samples[-1][0],
        )
        window_start = window_end - self.duration_ns
        acceleration = [
            vector for timestamp, vector in self.acceleration_samples
            if window_start <= timestamp <= window_end
        ]
        gyro = [
            vector for timestamp, vector in self.gyro_samples
            if window_start <= timestamp <= window_end
        ]

        if (
            len(acceleration) < self.minimum_acceleration_samples
            or len(gyro) < self.minimum_gyro_samples
        ):
            return None
        minimum_span = int(self.duration_ns * 0.9)
        acceleration_timestamps = [
            timestamp for timestamp, _ in self.acceleration_samples
            if window_start <= timestamp <= window_end
        ]
        gyro_timestamps = [
            timestamp for timestamp, _ in self.gyro_samples
            if window_start <= timestamp <= window_end
        ]
        if (
            acceleration_timestamps[-1] - acceleration_timestamps[0]
            < minimum_span
            or gyro_timestamps[-1] - gyro_timestamps[0] < minimum_span
        ):
            return None

        acceleration_mean = vector_mean(acceleration)
        gyro_mean = vector_mean(gyro)
        acceleration_stddev = vector_stddev(
            acceleration, acceleration_mean)
        gyro_stddev = vector_stddev(gyro, gyro_mean)

        gravity = vector_norm(acceleration_mean)
        if not self.minimum_gravity <= gravity <= self.maximum_gravity:
            self.failure_reason = (
                f'acceleration norm {gravity:.3f} m/s^2 is not gravity')
            self._discard_before(window_start)
            return None
        if vector_norm(acceleration_stddev) > \
                self.maximum_acceleration_stddev:
            self.failure_reason = (
                'acceleration is changing '
                f'(stddev norm {vector_norm(acceleration_stddev):.3f} '
                'm/s^2)')
            self._discard_before(window_start)
            return None
        if vector_norm(gyro_stddev) > self.maximum_gyro_stddev:
            self.failure_reason = (
                'angular velocity is changing '
                f'(stddev norm {vector_norm(gyro_stddev):.4f} rad/s)')
            self._discard_before(window_start)
            return None
        if vector_norm(gyro_mean) > self.maximum_gyro_mean:
            self.failure_reason = (
                'mean angular velocity is too large '
                f'({vector_norm(gyro_mean):.4f} rad/s)')
            self._discard_before(window_start)
            return None

        self.calibration = ImuCalibration(
            gravity_vector=acceleration_mean,
            gyro_bias=gyro_mean,
            acceleration_stddev=acceleration_stddev,
            gyro_stddev=gyro_stddev,
            acceleration_samples=len(acceleration),
            gyro_samples=len(gyro),
        )
        self.failure_reason = None
        self.acceleration_samples.clear()
        self.gyro_samples.clear()
        return self.calibration

    def _discard_before(self, timestamp_ns):
        """Bound memory while allowing a later stationary window to succeed."""
        self.acceleration_samples = [
            sample for sample in self.acceleration_samples
            if sample[0] >= timestamp_ns
        ]
        self.gyro_samples = [
            sample for sample in self.gyro_samples
            if sample[0] >= timestamp_ns
        ]
