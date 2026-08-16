"""Pure pose-filtering helpers used by the ROS perception node."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class FilterSnapshot:
    position: np.ndarray
    quaternion: np.ndarray
    covariance: np.ndarray
    stamp_sec: float
    confidence: float
    age_sec: float
    stale: bool
    outlier: bool
    accepted_samples: int
    rejected_outliers: int


class PoseFilter:
    def __init__(self, alpha, outlier_translation_m, stale_after_sec):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self.outlier_translation_m = outlier_translation_m
        self.stale_after_sec = stale_after_sec
        self.clear()

    def clear(self):
        self.position = None
        self.quaternion = None
        self.covariance = None
        self.stamp_sec = None
        self.confidence = 0.0
        self.accepted_samples = 0
        self.rejected_outliers = 0
        self.last_was_outlier = False

    def observe(self, position, quaternion, covariance, stamp_sec, confidence):
        position = np.asarray(position, dtype=float)
        quaternion = np.asarray(quaternion, dtype=float)
        quaternion /= np.linalg.norm(quaternion)
        covariance = np.asarray(covariance, dtype=float)

        if self.position is not None:
            jump = float(np.linalg.norm(position - self.position))
            recent = stamp_sec - self.stamp_sec <= self.stale_after_sec
            if recent and jump > self.outlier_translation_m:
                self.rejected_outliers += 1
                self.last_was_outlier = True
                return False

            if float(np.dot(quaternion, self.quaternion)) < 0.0:
                quaternion = -quaternion
            self.position = (1.0 - self.alpha) * self.position + self.alpha * position
            blended = (1.0 - self.alpha) * self.quaternion + self.alpha * quaternion
            self.quaternion = blended / np.linalg.norm(blended)
            self.covariance = (
                (1.0 - self.alpha) * self.covariance + self.alpha * covariance
            )
            self.confidence = (
                (1.0 - self.alpha) * self.confidence + self.alpha * confidence
            )
        else:
            self.position = position
            self.quaternion = quaternion
            self.covariance = covariance
            self.confidence = confidence

        self.stamp_sec = stamp_sec
        self.accepted_samples += 1
        self.last_was_outlier = False
        return True

    def snapshot(self, now_sec):
        if self.position is None:
            return None
        age = max(0.0, now_sec - self.stamp_sec)
        return FilterSnapshot(
            position=self.position.copy(),
            quaternion=self.quaternion.copy(),
            covariance=self.covariance.copy(),
            stamp_sec=self.stamp_sec,
            confidence=float(self.confidence),
            age_sec=age,
            stale=age > self.stale_after_sec,
            outlier=self.last_was_outlier,
            accepted_samples=self.accepted_samples,
            rejected_outliers=self.rejected_outliers,
        )


def rotation_matrix_to_quaternion(matrix):
    """Return an (x, y, z, w) unit quaternion from a 3x3 rotation matrix."""
    matrix = np.asarray(matrix, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        next_index = (index + 1) % 3
        last_index = (index + 2) % 3
        scale = math.sqrt(
            1.0
            + matrix[index, index]
            - matrix[next_index, next_index]
            - matrix[last_index, last_index]
        ) * 2.0
        quaternion = np.zeros(4)
        quaternion[index] = 0.25 * scale
        quaternion[3] = (
            matrix[last_index, next_index] - matrix[next_index, last_index]
        ) / scale
        quaternion[next_index] = (
            matrix[next_index, index] + matrix[index, next_index]
        ) / scale
        quaternion[last_index] = (
            matrix[last_index, index] + matrix[index, last_index]
        ) / scale
    return quaternion / np.linalg.norm(quaternion)


def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
