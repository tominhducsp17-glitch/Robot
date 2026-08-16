# Third-party software and contribution boundary

The repository depends on upstream binary packages and robot assets. Those
packages, demos, models, assets, and licenses remain the property of their
respective authors.

| Component | Source / package family | Use in this repository |
|---|---|---|
| ROS 2 Jazzy | Open Robotics ROS apt repository, `ros-jazzy-*` | Middleware and development tooling |
| MoveIt 2 | MoveIt binary release for Jazzy | Planning framework and Phase 0 smoke tests |
| MoveIt Task Constructor | MoveIt binary release for Jazzy | Phase 1 staged planning and execution framework |
| Panda resources | `moveit_resources_panda_*` | Upstream meshes and base robot description reused by the project-owned Phase 1 wrapper |
| Gazebo Harmonic | Gazebo vendor packages supplied through ROS Jazzy | Simulator and upstream worlds |
| ros_gz | `ros-jazzy-ros-gz` | ROS/Gazebo integration |
| ros2_control / ros2_controllers | ROS Controls Jazzy binaries | Controller framework |
| gz_ros2_control demos | `ros-jazzy-gz-ros2-control-demos` | Upstream Phase 0 integration smoke test |

Running, documenting, or wrapping an upstream demo is not claimed as an
original project contribution. Project-owned Phase 1 work consists of the
simulation world, wrapper description/configuration, staged task orchestration,
physical-result validation, and logging. It does not claim ownership of Panda,
MoveIt, Gazebo, or ROS assets and software.

Exact installed Debian versions and source links are recorded in
`docs/version_matrix.md`.
