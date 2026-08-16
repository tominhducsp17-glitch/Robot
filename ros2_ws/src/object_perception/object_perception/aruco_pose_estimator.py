#!/usr/bin/env python3
"""Estimate peg and fixture poses from simulated ArUco observations."""

import json
import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion
from manipulation_msgs.msg import ObjectPose, ObjectPoseArray
from moveit_msgs.msg import CollisionObject
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from object_perception.filtering import (
    PoseFilter,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
)


MARKERS = {
    0: {
        "object_id": "peg",
        "size_m": 0.034,
        "world_offset_m": np.array([0.0, 0.0, -0.06135]),
    },
    1: {
        "object_id": "fixture",
        "size_m": 0.080,
        "world_offset_m": np.array([0.0, 0.15, 0.04765]),
    },
}
SCENE_IDS = (
    "perception_peg",
    "perception_fixture_x_pos",
    "perception_fixture_x_neg",
    "perception_fixture_y_pos",
    "perception_fixture_y_neg",
)


class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__("aruco_pose_estimator")
        defaults = {
            "world_frame": "world",
            "camera_frame": "phase1_camera_optical",
            "image_topic": "/phase1/camera/image",
            "depth_topic": "/phase1/camera/depth_image",
            "camera_info_topic": "/phase1/camera/camera_info",
            "stale_after_sec": 0.5,
            "max_depth_age_sec": 1.0,
            "outlier_translation_m": 0.05,
            "filter_alpha": 0.35,
            "minimum_confidence": 0.50,
            "minimum_samples": 5,
            "scene_publish_period_sec": 0.5,
            "adaptive_threshold_constant": 25.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.world_frame = self.get_parameter("world_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.minimum_confidence = float(
            self.get_parameter("minimum_confidence").value
        )
        self.minimum_samples = int(self.get_parameter("minimum_samples").value)
        alpha = float(self.get_parameter("filter_alpha").value)
        outlier_threshold = float(
            self.get_parameter("outlier_translation_m").value
        )
        stale_after = float(self.get_parameter("stale_after_sec").value)
        self.max_depth_age_sec = float(
            self.get_parameter("max_depth_age_sec").value
        )

        self.filters = {
            marker_id: PoseFilter(alpha, outlier_threshold, stale_after)
            for marker_id in MARKERS
        }
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion = None
        self.frozen = False
        self.latest_depth = None
        self.latest_depth_stamp_sec = None
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.detector_parameters.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX
        )
        self.detector_parameters.adaptiveThreshConstant = float(
            self.get_parameter("adaptive_threshold_constant").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.pose_publisher = self.create_publisher(
            ObjectPoseArray, "/perception/object_poses", 10
        )
        self.scene_publisher = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self.create_service(Trigger, "/perception/reset", self._reset)
        self.create_service(Trigger, "/perception/snapshot", self._snapshot)
        self.create_service(Trigger, "/perception/freeze", self._freeze)
        self.create_timer(0.1, self._publish_poses)
        self.create_timer(
            float(self.get_parameter("scene_publish_period_sec").value),
            self._publish_scene,
        )
        self.get_logger().info(
            f"Detecting ArUco IDs {tuple(MARKERS)} on {image_topic}"
        )

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_camera_info(self, message):
        self.camera_matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
        self.distortion = np.asarray(message.d, dtype=float)

    def _on_depth(self, message):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            message, desired_encoding="32FC1"
        )
        self.latest_depth_stamp_sec = (
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        )

    def _world_from_camera(self):
        transform = self.tf_buffer.lookup_transform(
            self.world_frame,
            self.camera_frame,
            Time(),
        ).transform
        quaternion = np.array(
            [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ]
        )
        translation = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z]
        )
        return quaternion_to_rotation_matrix(quaternion), translation

    def _on_image(self, message):
        if self.frozen or self.camera_matrix is None:
            return
        try:
            world_rotation, world_translation = self._world_from_camera()
        except TransformException as error:
            self.get_logger().warn(
                f"Waiting for camera transform: {error}",
                throttle_duration_sec=2.0,
            )
            return

        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            image,
            self.dictionary,
            parameters=self.detector_parameters,
        )
        if ids is None:
            return

        stamp_sec = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp_sec <= 0.0:
            stamp_sec = self._now_sec()
        for marker_corners, marker_id_value in zip(corners, ids.flatten()):
            marker_id = int(marker_id_value)
            if marker_id not in MARKERS:
                continue
            self._observe_marker(
                marker_id,
                marker_corners,
                stamp_sec,
                world_rotation,
                world_translation,
            )

    def _observe_marker(
        self,
        marker_id,
        marker_corners,
        stamp_sec,
        world_rotation,
        world_translation,
    ):
        marker_size = MARKERS[marker_id]["size_m"]
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [marker_corners],
            marker_size,
            self.camera_matrix,
            self.distortion,
        )
        rvec = rvecs[0, 0]
        camera_position = tvecs[0, 0]
        camera_position = self._depth_corrected_position(
            camera_position, marker_corners, stamp_sec
        )
        if camera_position is None:
            return
        camera_rotation, _ = cv2.Rodrigues(rvec)
        world_tag_position = world_rotation @ camera_position + world_translation
        world_object_position = (
            world_tag_position + MARKERS[marker_id]["world_offset_m"]
        )
        world_object_rotation = world_rotation @ camera_rotation
        quaternion = rotation_matrix_to_quaternion(world_object_rotation)

        projected, _ = cv2.projectPoints(
            self._marker_object_points(marker_size),
            rvec,
            camera_position,
            self.camera_matrix,
            self.distortion,
        )
        pixels = marker_corners.reshape(4, 2)
        reprojection_error = float(
            np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pixels) ** 2, axis=1)))
        )
        pixel_sides = np.linalg.norm(pixels - np.roll(pixels, 1, axis=0), axis=1)
        pixel_side = float(np.mean(pixel_sides))
        confidence = min(1.0, pixel_side / 24.0) * math.exp(
            -reprojection_error / 2.0
        )
        distance = float(np.linalg.norm(camera_position))
        translation_sigma = max(0.001, distance / max(pixel_side, 1.0) * 0.02)
        rotation_sigma = max(0.01, reprojection_error * 0.01)
        covariance = np.zeros(36)
        covariance[[0, 7, 14]] = translation_sigma**2
        covariance[[21, 28, 35]] = rotation_sigma**2
        self.filters[marker_id].observe(
            world_object_position,
            quaternion,
            covariance,
            stamp_sec,
            confidence,
        )

    def _depth_corrected_position(self, pnp_position, marker_corners, stamp_sec):
        if (
            self.latest_depth is None
            or self.latest_depth_stamp_sec is None
            or abs(stamp_sec - self.latest_depth_stamp_sec) > self.max_depth_age_sec
        ):
            return None
        center = np.mean(marker_corners.reshape(4, 2), axis=0)
        u = int(round(float(center[0])))
        v = int(round(float(center[1])))
        y0, y1 = max(0, v - 2), min(self.latest_depth.shape[0], v + 3)
        x0, x1 = max(0, u - 2), min(self.latest_depth.shape[1], u + 3)
        samples = self.latest_depth[y0:y1, x0:x1]
        valid = samples[np.isfinite(samples) & (samples > 0.0)]
        if valid.size == 0 or abs(float(pnp_position[2])) < 1e-6:
            return None
        # Gazebo publishes optical-axis depth (Z), matching CameraInfo's
        # pinhole convention.  Scale the complete PnP ray by that Z value.
        optical_z = float(np.median(valid))
        return pnp_position * (optical_z / float(pnp_position[2]))

    @staticmethod
    def _marker_object_points(size):
        half = size / 2.0
        return np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

    def _snapshot_dict(self):
        now_sec = self._now_sec()
        objects = {}
        ready = True
        for marker_id, spec in MARKERS.items():
            snapshot = self.filters[marker_id].snapshot(now_sec)
            if snapshot is None:
                ready = False
                objects[spec["object_id"]] = None
                continue
            valid = (
                not snapshot.stale
                and not snapshot.outlier
                and snapshot.confidence >= self.minimum_confidence
                and snapshot.accepted_samples >= self.minimum_samples
            )
            ready = ready and valid
            objects[spec["object_id"]] = {
                "marker_id": marker_id,
                "position": snapshot.position.tolist(),
                "quaternion_xyzw": snapshot.quaternion.tolist(),
                "covariance": snapshot.covariance.tolist(),
                "confidence": snapshot.confidence,
                "measurement_age_sec": snapshot.age_sec,
                "stale": snapshot.stale,
                "outlier": snapshot.outlier,
                "accepted_samples": snapshot.accepted_samples,
                "rejected_outliers": snapshot.rejected_outliers,
                "valid": valid,
            }
        return {
            "ready": ready,
            "frozen": self.frozen,
            "world_frame": self.world_frame,
            "camera_frame": self.camera_frame,
            "minimum_confidence": self.minimum_confidence,
            "minimum_samples": self.minimum_samples,
            "objects": objects,
        }

    def _publish_poses(self):
        snapshot_dict = self._snapshot_dict()
        message = ObjectPoseArray()
        message.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame
        )
        for marker_id, spec in MARKERS.items():
            snapshot = self.filters[marker_id].snapshot(self._now_sec())
            if snapshot is None:
                continue
            pose_message = ObjectPose()
            stamp_sec = int(snapshot.stamp_sec)
            pose_message.header.stamp.sec = stamp_sec
            pose_message.header.stamp.nanosec = int(
                (snapshot.stamp_sec - stamp_sec) * 1e9
            )
            pose_message.header.frame_id = self.world_frame
            pose_message.object_id = spec["object_id"]
            pose_message.marker_id = marker_id
            pose_message.pose.pose = Pose(
                position=Point(
                    x=float(snapshot.position[0]),
                    y=float(snapshot.position[1]),
                    z=float(snapshot.position[2]),
                ),
                orientation=Quaternion(
                    x=float(snapshot.quaternion[0]),
                    y=float(snapshot.quaternion[1]),
                    z=float(snapshot.quaternion[2]),
                    w=float(snapshot.quaternion[3]),
                ),
            )
            pose_message.pose.covariance = snapshot.covariance.tolist()
            pose_message.confidence = snapshot.confidence
            pose_message.measurement_age_sec = snapshot.age_sec
            pose_message.stale = snapshot.stale
            pose_message.outlier = snapshot.outlier
            pose_message.accepted_samples = snapshot.accepted_samples
            pose_message.rejected_outliers = snapshot.rejected_outliers
            message.poses.append(pose_message)
        self.pose_publisher.publish(message)

    def _publish_scene(self):
        if self.frozen:
            return
        snapshot = self._snapshot_dict()
        if not snapshot["ready"]:
            return
        peg = snapshot["objects"]["peg"]["position"]
        fixture = snapshot["objects"]["fixture"]["position"]
        self.scene_publisher.publish(
            self._collision_object(
                "perception_peg",
                SolidPrimitive.CYLINDER,
                [0.12, 0.018],
                peg,
            )
        )
        walls = (
            ("perception_fixture_x_pos", [0.02, 0.09, 0.10], [0.035, 0.0, 0.05]),
            ("perception_fixture_x_neg", [0.02, 0.09, 0.10], [-0.035, 0.0, 0.05]),
            ("perception_fixture_y_pos", [0.05, 0.02, 0.10], [0.0, 0.035, 0.05]),
            ("perception_fixture_y_neg", [0.05, 0.02, 0.10], [0.0, -0.035, 0.05]),
        )
        for object_id, dimensions, offset in walls:
            position = [fixture[index] + offset[index] for index in range(3)]
            self.scene_publisher.publish(
                self._collision_object(
                    object_id,
                    SolidPrimitive.BOX,
                    dimensions,
                    position,
                )
            )

    def _collision_object(self, object_id, primitive_type, dimensions, position):
        primitive = SolidPrimitive(type=primitive_type, dimensions=dimensions)
        return CollisionObject(
            header=Header(frame_id=self.world_frame),
            id=object_id,
            primitives=[primitive],
            primitive_poses=[
                Pose(
                    position=Point(
                        x=float(position[0]),
                        y=float(position[1]),
                        z=float(position[2]),
                    ),
                    orientation=Quaternion(w=1.0),
                )
            ],
            operation=CollisionObject.ADD,
        )

    def _remove_scene(self):
        for object_id in SCENE_IDS:
            self.scene_publisher.publish(
                CollisionObject(
                    header=Header(frame_id=self.world_frame),
                    id=object_id,
                    operation=CollisionObject.REMOVE,
                )
            )

    def _reset(self, _request, response):
        for pose_filter in self.filters.values():
            pose_filter.clear()
        self.frozen = False
        self._remove_scene()
        response.success = True
        response.message = json.dumps(self._snapshot_dict(), separators=(",", ":"))
        return response

    def _snapshot(self, _request, response):
        response.success = True
        response.message = json.dumps(self._snapshot_dict(), separators=(",", ":"))
        return response

    def _freeze(self, _request, response):
        snapshot = self._snapshot_dict()
        response.success = snapshot["ready"]
        if response.success:
            self.frozen = True
            snapshot["frozen"] = True
        response.message = json.dumps(snapshot, separators=(",", ":"))
        return response


def main():
    rclpy.init()
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
