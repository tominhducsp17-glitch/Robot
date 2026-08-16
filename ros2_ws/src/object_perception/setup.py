from glob import glob
from setuptools import find_packages, setup


package_name = "object_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="duc",
    maintainer_email="tominhducsp17@gmail.com",
    description="ArUco pose estimation and planning-scene updates",
    license="LicenseRef-Proprietary",
    entry_points={
        "console_scripts": [
            "aruco_pose_estimator = object_perception.aruco_pose_estimator:main",
            "perception_snapshot_client = object_perception.snapshot_client:main",
        ],
    },
)
