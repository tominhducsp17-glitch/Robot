# Third-party software and contribution boundary

Phase 0 installs and executes upstream binary packages only. Those packages,
their demos, robot models, assets, and licenses remain the property of their
respective authors.

| Component | Source / package family | Use in this repository |
|---|---|---|
| ROS 2 Jazzy | Open Robotics ROS apt repository, `ros-jazzy-*` | Middleware and development tooling |
| MoveIt 2 | MoveIt binary release for Jazzy | Planning framework and upstream smoke tests |
| Panda resources | `moveit_resources_panda_*` | Upstream robot model/config used only for Phase 0 validation |
| Gazebo Harmonic | Gazebo vendor packages supplied through ROS Jazzy | Simulator and upstream worlds |
| ros_gz | `ros-jazzy-ros-gz` | ROS/Gazebo integration |
| ros2_control / ros2_controllers | ROS Controls Jazzy binaries | Controller framework |
| gz_ros2_control demos | `ros-jazzy-gz-ros2-control-demos` | Upstream Phase 0 integration smoke test |

Running, documenting, or wrapping an upstream demo is not claimed as an
original project contribution. Future project-owned nodes must be identified in
this file and must preserve all relevant upstream notices and licenses.

Exact installed Debian versions and source links are recorded in
`docs/version_matrix.md`.
