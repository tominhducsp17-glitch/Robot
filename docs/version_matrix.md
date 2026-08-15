# Phase 0 version matrix

Matrix decision date: 2026-08-15. The distribution pair is chosen by support
compatibility, not because it is the newest available release.

## Supported matrix

| Layer | Selected release / package | Rationale | Installed Debian version |
|---|---|---|---|
| OS | Ubuntu 24.04 Noble, amd64 | ROS 2 Jazzy Tier-1 binary platform | `24.04.4 LTS` (OS release) |
| ROS | ROS 2 Jazzy Jalisco LTS, `ros-jazzy-desktop` | Jazzy is the LTS paired with Noble | `0.11.0-1noble.20260616.084553` |
| Build tools | `ros-dev-tools`, Ubuntu `build-essential`, CMake, Ninja | Official ROS development tooling plus native compiler/test tools | `ros-dev-tools 1.0.1`; `build-essential 12.10ubuntu1`; CMake `3.28.3-1build7`; Ninja `1.11.1-2` |
| MoveIt | Jazzy binary `ros-jazzy-moveit` | Stay on the ROS distribution binary release | `2.12.4-1noble.20260617.161956` |
| Panda model/config | `ros-jazzy-moveit-resources-panda-description`, `ros-jazzy-moveit-resources-panda-moveit-config` | Official MoveIt resource packages for the smoke demo | Description `3.1.0-1noble.20260225.235702`; config `3.1.0-1noble.20260615.174424` |
| Panda demo runtime fixes | `ros-jazzy-joint-state-broadcaster`, `ros-jazzy-joint-trajectory-controller`, `ros-jazzy-rviz-visual-tools` | Required by the upstream demo at runtime but not pulled into the initial binary install | Controllers `4.40.1-1noble.20260615.171040` / `4.40.1-1noble.20260615.171409`; visual tools `4.1.4-4noble.20260615.175111` |
| Gazebo | Gazebo Harmonic through ROS Jazzy vendor packages | Officially recommended ROS 2 Jazzy pairing | `gz sim 8.11.0`; `ros-jazzy-gz-sim-vendor 0.0.10-1noble.20260604.111001` |
| ROS/Gazebo bridge | `ros-jazzy-ros-gz` | Pulls the Gazebo release paired by the ROS repository | `1.0.22-1noble.20260616.074726` |
| ros2_control | `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers` | Official Jazzy binary packages | `4.45.2-1noble.20260615.175135`; controllers `4.40.1-1noble.20260616.074625` |
| Gazebo control adapter | `ros-jazzy-gz-ros2-control`, `ros-jazzy-gz-ros2-control-demos` | Official Jazzy adapter; its vendor dependencies select Harmonic | Adapter `1.2.19-1noble.20260615.171757`; demos `1.2.19-1noble.20260616.073637` |
| Python | Ubuntu system Python only | Matches Noble/ROS binary ABI; no Conda | `3.12.3-0ubuntu2.1`; pytest `7.4.4-1`; pip `24.0+dfsg-1ubuntu1.3` |

## Official compatibility sources

- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html): Noble 24.04 binary installation, `ros2-apt-source`, `ros-jazzy-desktop`, and `ros-dev-tools`.
- [MoveIt Getting Started](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html): recommends Jazzy on Ubuntu 24.04 and sourcing `/opt/ros/jazzy/setup.bash`.
- [Gazebo/ROS installation matrix](https://gazebosim.org/docs/jetty/ros_installation/): explicitly identifies ROS 2 Jazzy + Gazebo Harmonic as the recommended combination and recommends `ros-${ROS_DISTRO}-ros-gz` from the ROS repository.
- [Gazebo Harmonic release](https://gazebosim.org/docs/harmonic/install/): Harmonic is an LTS release supporting Ubuntu Noble amd64; its core simulator is `gz-sim` 8.x.
- [ros2_control Jazzy installation](https://control.ros.org/jazzy/doc/getting_started/getting_started.html): names the Jazzy `ros2-control` and `ros2-controllers` binary packages.
- [gz_ros2_control Jazzy documentation](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html): names the Jazzy adapter/demo packages and states that Jazzy vendor packages provide Gazebo Harmonic.

No OSRF Gazebo repository is added for this matrix. This avoids accidentally
mixing Jetty/Ionic or another non-default Gazebo line into the Jazzy stack.

## Exact version capture

Exact D1-D4 versions were captured from `dpkg-query`. To reproduce the capture:

```bash
./scripts/phase0/package_versions.sh
```

The script uses `dpkg-query`, so values are installed versions rather than
website release labels or guessed apt candidates.

## Prohibited substitutions

- No ROS Rolling.
- No Gazebo Jetty or Ionic.
- No Conda-provided ROS/Gazebo stack.
- No CUDA, PyTorch GPU, YOLO, VLA, or other deep-learning dependency in Phase 0.
- No source checkout merely to obtain a newer MoveIt/Gazebo feature.
