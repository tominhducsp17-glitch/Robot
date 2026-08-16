from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RepositoryLayoutTest(unittest.TestCase):
    def test_required_root_files_exist(self) -> None:
        for relative_path in (
            "README.md",
            "LICENSE",
            "THIRD_PARTY.md",
            ".gitignore",
            "docs/environment_audit.md",
            "docs/phase1_status.md",
            "docs/version_matrix.md",
        ):
            with self.subTest(path=relative_path):
                self.assertTrue((ROOT / relative_path).is_file())

    def test_generated_artifacts_are_ignored(self) -> None:
        for relative_path in (
            "build/probe",
            "install/probe",
            "log/probe",
            "rosbag2_probe/metadata.yaml",
            "experiments/raw/probe.csv",
            "experiments/summaries/probe.json",
            "videos/probe.mp4",
            "datasets/probe.bin",
        ):
            with self.subTest(path=relative_path):
                result = subprocess.run(
                    ["git", "check-ignore", "--quiet", relative_path],
                    cwd=ROOT,
                    check=False,
                )
                self.assertEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
