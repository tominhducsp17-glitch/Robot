from glob import glob
from setuptools import find_packages, setup


package_name = "experiment_runner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="duc",
    maintainer_email="tominhducsp17@gmail.com",
    description="Phase 1 MTC runner",
    license="LicenseRef-Proprietary",
    entry_points={
        "console_scripts": [
            "phase1_task = experiment_runner.phase1_task:main",
        ],
    },
)
