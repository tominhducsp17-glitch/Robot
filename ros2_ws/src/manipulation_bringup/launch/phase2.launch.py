from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")
    force_limit_n = LaunchConfiguration("force_limit_n")

    bringup_share = Path(get_package_share_directory("manipulation_bringup"))
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / "launch" / "phase1.launch.py")),
        launch_arguments={
            "rviz": rviz,
            "headless": headless,
            "force_limit_n": force_limit_n,
        }.items(),
    )
    camera_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0.50",
            "--y", "0.0",
            "--z", "1.85",
            "--qx", "0.7071067811865476",
            "--qy", "-0.7071067811865476",
            "--qz", "0.0",
            "--qw", "0.0",
            "--frame-id", "world",
            "--child-frame-id", "phase1_camera_optical",
        ],
    )
    perception_config = str(
        Path(get_package_share_directory("object_perception"))
        / "config"
        / "phase2_perception.yaml"
    )
    perception = Node(
        package="object_perception",
        executable="aruco_pose_estimator",
        output="screen",
        parameters=[perception_config, {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("force_limit_n", default_value="50.0"),
            base,
            camera_transform,
            perception,
        ]
    )
