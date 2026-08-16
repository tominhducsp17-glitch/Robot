from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    peg_x = LaunchConfiguration("peg_x")
    peg_y = LaunchConfiguration("peg_y")
    perception_timeout = LaunchConfiguration("perception_timeout")

    moveit_config = (
        MoveItConfigsBuilder("phase1_panda", package_name="manipulation_moveit_config")
        .robot_description(file_path="config/phase1_panda.urdf.xacro")
        .robot_description_semantic(file_path="config/phase1_panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    runner = Node(
        package="experiment_runner",
        executable="phase1_task",
        name="phase2_task",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        arguments=[
            "--task",
            "task_b",
            "--peg-x",
            peg_x,
            "--peg-y",
            peg_y,
            "--pose-source",
            "perception",
            "--perception-timeout",
            perception_timeout,
            "--log-dir",
            "experiments/raw/phase2",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("peg_x", default_value="0.38"),
            DeclareLaunchArgument("peg_y", default_value="0.20"),
            DeclareLaunchArgument("perception_timeout", default_value="15.0"),
            runner,
        ]
    )
