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
        contact_sensor = world.find(".//sensor[@type='contact']")
        self.assertIsNotNone(contact_sensor)
        self.assertEqual(
            contact_sensor.findtext("./contact/topic"),
            "/phase1/peg/contacts",
        )
        plugins = {plugin.attrib["filename"] for plugin in world.findall(".//plugin")}
        self.assertIn("gz-sim-contact-system", plugins)

    def test_nominal_matrix_has_five_unique_poses(self) -> None:
        config_path = (
            ROOT / "ros2_ws/src/experiment_runner/config/nominal_poses.yaml"
        )
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        poses = config["poses"]
        self.assertEqual(len(poses), 5)
        self.assertEqual(len({pose["id"] for pose in poses}), 5)
        self.assertEqual(len({(pose["x"], pose["y"]) for pose in poses}), 5)

    def test_runner_keeps_ground_truth_and_force_evaluation_explicit(self) -> None:
        runner = (
            ROOT
            / "ros2_ws/src/experiment_runner/experiment_runner/phase1_task.py"
        ).read_text(encoding="utf-8")
        for required in (
            '"ground_truth_geometric_baseline"',
            '"simulation_only": True',
            '"deployable_controller": False',
            '"force_monitor_available": force_metrics["available"]',
            '"force_violation": force_metrics["force_violation"]',
            '"contact_force": force_metrics',
            '"physical_outcome": physical_outcome',
        ):
            with self.subTest(required=required):
                self.assertIn(required, runner)

    def test_force_monitor_is_wired_into_bringup_and_gate(self) -> None:
        launch = (
            ROOT
            / "ros2_ws/src/manipulation_bringup/launch/phase1.launch.py"
        ).read_text(encoding="utf-8")
        benchmark = (
            ROOT / "scripts/phase1/run_nominal_benchmark.sh"
        ).read_text(encoding="utf-8")
        monitor = (
            ROOT
            / "ros2_ws/src/experiment_runner/experiment_runner/contact_force_monitor.py"
        ).read_text(encoding="utf-8")
        self.assertIn("/phase1/peg/contacts@ros_gz_interfaces/msg/Contacts", launch)
        self.assertIn('DeclareLaunchArgument("force_limit_n"', launch)
        self.assertIn('"phase1_exit_gate": phase1_exit_gate', benchmark)
        self.assertIn("body_1_wrench.force", monitor)
        self.assertIn('"rms_force_n": rms', monitor)

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
