from pathlib import Path
import stat
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0, str(ROOT / "ros2_ws/src/object_perception")
)

from object_perception.filtering import PoseFilter  # noqa: E402


class Phase2ContractTest(unittest.TestCase):
    def test_project_interfaces_are_declared(self) -> None:
        package = ET.parse(
            ROOT / "ros2_ws/src/manipulation_msgs/package.xml"
        ).getroot()
        self.assertEqual(package.findtext("name"), "manipulation_msgs")
        for name in ("ObjectPose.msg", "ObjectPoseArray.msg"):
            self.assertTrue(
                (ROOT / "ros2_ws/src/manipulation_msgs/msg" / name).is_file()
            )

    def test_perception_configuration_has_runtime_guards(self) -> None:
        config = yaml.safe_load(
            (
                ROOT
                / "ros2_ws/src/object_perception/config/phase2_perception.yaml"
            ).read_text(encoding="utf-8")
        )["aruco_pose_estimator"]["ros__parameters"]
        self.assertLessEqual(config["stale_after_sec"], 0.5)
        self.assertGreaterEqual(config["minimum_confidence"], 0.5)
        self.assertGreaterEqual(config["minimum_samples"], 10)
        self.assertLessEqual(config["outlier_translation_m"], 0.05)

    def test_pose_filter_rejects_jumps_and_marks_stale(self) -> None:
        pose_filter = PoseFilter(
            alpha=0.5,
            outlier_translation_m=0.05,
            stale_after_sec=0.5,
        )
        covariance = np.eye(6).reshape(-1)
        self.assertTrue(
            pose_filter.observe(
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                covariance,
                1.0,
                0.9,
            )
        )
        self.assertFalse(
            pose_filter.observe(
                [0.10, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                covariance,
                1.1,
                0.9,
            )
        )
        rejected = pose_filter.snapshot(1.1)
        self.assertTrue(rejected.outlier)
        self.assertEqual(rejected.rejected_outliers, 1)
        self.assertTrue(pose_filter.snapshot(1.6).stale)

    def test_camera_bridge_is_gazebo_to_ros_only(self) -> None:
        launch = (
            ROOT / "ros2_ws/src/manipulation_bringup/launch/phase1.launch.py"
        ).read_text(encoding="utf-8")
        for topic in (
            "image@sensor_msgs/msg/Image[gz.msgs.Image",
            "camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ):
            with self.subTest(topic=topic):
                self.assertIn(topic, launch)

    def test_phase2_runner_uses_frozen_perception(self) -> None:
        runner = (
            ROOT
            / "ros2_ws/src/experiment_runner/experiment_runner/phase1_task.py"
        ).read_text(encoding="utf-8")
        for required in (
            'choices=("configured", "perception")',
            'perception_client("reset"',
            'perception_client("freeze"',
            '"source": "perception_snapshot"',
            '"perception_evaluation_only": perception',
        ):
            with self.subTest(required=required):
                self.assertIn(required, runner)

    def test_phase2_benchmark_is_executable(self) -> None:
        script = ROOT / "scripts/phase2/run_vision_benchmark.sh"
        self.assertTrue(script.is_file())
        self.assertTrue(script.stat().st_mode & stat.S_IXUSR)
        contents = script.read_text(encoding="utf-8")
        self.assertIn('"phase2_exit_gate": exit_gate', contents)
        self.assertIn('"perception_snapshot"', contents)


if __name__ == "__main__":
    unittest.main()
