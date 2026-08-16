from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_enabled = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")

    description_file = PathJoinSubstitution(
        [FindPackageShare("manipulation_description"), "urdf", "panda_phase1.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("manipulation_description"), "config", "ros2_controllers.yaml"]
    )
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                description_file,
                " controllers_file:=",
                controllers_file,
            ]
        )
    }

    world_path = str(
        Path(get_package_share_directory("manipulation_description"))
        / "worlds"
        / "phase1_baseline.sdf"
    )
    gz_launch = Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py"

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch)),
        launch_arguments={"gz_args": f"-r -v 2 {world_path}"}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch)),
        launch_arguments={"gz_args": f"-s -r -v 2 {world_path}"}.items(),
        condition=IfCondition(headless),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    spawn_panda = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "panda"],
    )
    reset_peg = Node(
        package="manipulation_bringup",
        executable="reset_peg.sh",
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["panda_arm_controller", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )
    hand_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["panda_hand_controller", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_panda, on_exit=[reset_peg, joint_state_spawner])
    )
    spawn_motion_controllers = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_spawner, on_exit=[arm_spawner, hand_spawner])
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/phase1/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/phase1/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/phase1/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/phase1/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
    )

    moveit_config = (
        MoveItConfigsBuilder("phase1_panda", package_name="manipulation_moveit_config")
        .robot_description(file_path="config/phase1_panda.urdf.xacro")
        .robot_description_semantic(file_path="config/phase1_panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("manipulation_bringup"), "rviz", "phase1.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            gazebo_gui,
            gazebo_headless,
            bridge,
            state_publisher,
            spawn_panda,
            spawn_controllers,
            spawn_motion_controllers,
            move_group,
            rviz,
        ]
    )
