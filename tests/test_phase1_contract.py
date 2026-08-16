from pathlib import Path
import stat
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


class Phase1ContractTest(unittest.TestCase):
    def test_world_contains_required_entities_and_camera(self) -> None:
        world = ET.parse(
            ROOT
            / "ros2_ws/src/manipulation_description/worlds/phase1_baseline.sdf"
        ).getroot()
        names = {model.attrib["name"] for model in world.findall(".//world/model")}
        self.assertTrue(
            {"ground_plane", "work_table", "peg", "fixture", "overhead_camera"}
            <= names
        )
        self.assertIsNotNone(world.find(".//sensor[@type='rgbd_camera']"))

    def test_nominal_matrix_has_five_unique_poses(self) -> None:
        config_path = (
            ROOT / "ros2_ws/src/experiment_runner/config/nominal_poses.yaml"
        )
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        poses = config["poses"]
        self.assertEqual(len(poses), 5)
        self.assertEqual(len({pose["id"] for pose in poses}), 5)
        self.assertEqual(len({(pose["x"], pose["y"]) for pose in poses}), 5)

    def test_runner_keeps_ground_truth_and_force_limits_explicit(self) -> None:
        runner = (
            ROOT
            / "ros2_ws/src/experiment_runner/experiment_runner/phase1_task.py"
        ).read_text(encoding="utf-8")
        for required in (
            '"mode": "ground_truth_geometric_baseline"',
            '"simulation_only": True',
            '"deployable_controller": False',
            '"force_monitor_available": False',
            '"force_violation": None',
            '"physical_outcome": physical_outcome',
        ):
            with self.subTest(required=required):
                self.assertIn(required, runner)

    def test_phase1_scripts_are_executable(self) -> None:
        for name in (
            "install_dependencies.sh",
            "reset_peg.sh",
            "attach_peg.sh",
            "detach_peg.sh",
            "run_nominal_benchmark.sh",
        ):
            path = ROOT / "scripts/phase1" / name
            with self.subTest(script=name):
                self.assertTrue(path.is_file())
                self.assertTrue(path.stat().st_mode & stat.S_IXUSR)


if __name__ == "__main__":
    unittest.main()
