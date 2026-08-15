# Environment audit

Audit time: `2026-08-15T12:51:38+00:00`

Final Phase 0 verification: `2026-08-15T13:44:57+00:00`

Workspace: `/home/duc/projects/closed_loop_compliant_manipulation`

## Summary

| Check | Observed value | Status |
|---|---|---|
| Guest OS | Ubuntu 24.04.4 LTS (Noble), x86_64 | PASS for ROS 2 Jazzy |
| Kernel / virtualization | `6.18.33.2-microsoft-standard-WSL2`; Microsoft full virtualization | PASS (WSL2 detected) |
| CPU visible to WSL | Intel Core Ultra 5 125H, 18 logical CPUs | PASS |
| Memory visible to WSL | 15 GiB RAM, 4 GiB swap | PASS with constraint; less than the host's known 31.6 GB |
| Workspace filesystem | ext4 (`/dev/sdg`), not `/mnt/*` | PASS |
| Free disk | 953 GiB available of 1007 GiB | PASS |
| WSLg environment | `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`, `/dev/dxg` present | PASS functional; RViz and Gazebo rendered responsively |
| OpenGL renderer | Mesa 25.2.8 `llvmpipe` (LLVM 20.1.2), direct rendering, OpenGL 4.5 | PASS functional / WARNING software rendering (`Accelerated: no`) |
| Ubuntu apt sources | Noble main/universe/restricted/multiverse, updates/backports/security | PASS |
| ROS/Gazebo apt sources | none present before Phase 0 installation | EXPECTED |
| Locale | `C.UTF-8` for `LANG` and `LC_ALL` | PASS |
| Git | 2.43.0 (`1:2.43.0-1ubuntu7.3`) | PASS |
| Python | 3.12.3 (`3.12.3-0ubuntu2.1`) | PASS |
| Compiler / CMake / Ninja | GCC/G++ 13.3.0, CMake 3.28.3, Ninja 1.11.1 | PASS |
| ROS 2 / RViz / Gazebo | ROS 2 Jazzy Desktop, MoveIt/RViz, and Gazebo Harmonic 8.11.0 installed | D2-D4 runtime smoke tests PASS |
| Privilege | User successfully completed D1 interactively; unattended `sudo -n` still requires a password | Expected for later interactive apt gates |

WSLg libraries include Mesa GLX/EGL and Microsoft's D3D12/DXCore libraries.
`/dev/dri` is absent. `glxinfo -B` now proves that OpenGL works, but it selects
CPU-based `llvmpipe`, not accelerated D3D12/Intel rendering. RViz and Gazebo GUI
must therefore be evaluated for responsiveness, while their headless modes will
be used to distinguish a GUI limitation from a simulator failure.

## Commands and evidence

The pre-install audit used:

```bash
cat /etc/os-release
uname -a
lscpu
free -h
df -hT / /home/duc/projects/closed_loop_compliant_manipulation
find /etc/apt/sources.list /etc/apt/sources.list.d -maxdepth 1 -type f -print
git --version
python3 --version
locale
command -v gcc g++ cmake ninja colcon rosdep glxinfo rviz2 gz
ls -ld /mnt/wslg /mnt/wslg/runtime-dir /dev/dxg
ldconfig -p | rg 'lib(EGL|GLX|OpenGL|d3d12|dxcore)'
sudo -n true
```

Important excerpts:

```text
PRETTY_NAME="Ubuntu 24.04.4 LTS"
Linux ... 6.18.33.2-microsoft-standard-WSL2 ... x86_64 GNU/Linux
CPU(s): 18
Model name: Intel(R) Core(TM) Ultra 5 125H
Mem: 15Gi total, 13Gi available
/dev/sdg ext4 1007G total, 953G available
DISPLAY=:0
WAYLAND_DISPLAY=wayland-0
sudo: a password is required
```

The only configured apt source file with active entries was
`/etc/apt/sources.list.d/ubuntu.sources`, covering Noble, Noble updates,
backports, and security with all standard components. No ROS or OSRF source was
present.

## Gate log

| Gate | Commands actually run | Result | Evidence / blocker |
|---|---|---|---|
| Audit | commands above plus RViz/Gazebo GUI tests | **PASS** | OS, WSL2, resources, apt, locale, OpenGL, and both GUIs identified/tested |
| D1 base toolchain | `./scripts/phase0/install_d1.sh`; `./scripts/phase0/smoke_toolchain.sh` | **PASS** | Re-run exit 0; all required executables ran and `glxinfo -B` returned a valid OpenGL 4.5 context |
| D2 ROS graph | `./scripts/phase0/smoke_ros_graph.sh` | **PASS** | Exit 0; listener received `Hello World: 1` from C++ talker |
| D3 MoveIt/Panda/RViz | corrected install; `./scripts/phase0/smoke_moveit_panda.sh`; RViz Plan from `<current>` to `extended` | **PASS** | Arm and joint-state controllers active; Panda/scene rendered; trajectory planned in 0.011 s and displayed/moved in RViz; user confirmed |
| D4 Gazebo/ros2_control | install; `./scripts/phase0/smoke_gazebo.sh`; isolated `./scripts/phase0/smoke_gz_ros2_control.sh` re-test | **PASS** | Harmonic empty-world services and responsive GUI PASS; `/joint_states` published `slider_to_cart`; both cart controllers active; action accepted and reached `0 → -1 → 1 → 0` |
| Repository Python smoke | `python3 -m unittest discover -s tests -p 'test_*.py' -v` | **PASS** | 2 tests passed; no simulator required |
| Repository C++ smoke | CMake configure/build followed by `ctest --test-dir build/repository-tests --output-on-failure` | **PASS** | 1/1 C++17 test passed with GCC 13.3.0 |

The original D1 attempt failed before installation because sudo required the
user's password. After the user reset the WSL password and ran the installer,
the D1 smoke test was repeated independently and passed with exit 0.

The first D2 installer run successfully installed ROS Desktop but stopped while
sourcing ROS because strict Bash nounset mode exposed an unset optional setup
variable. The Phase 0 scripts now disable nounset only while sourcing the
ROS-generated setup file. A repeat source check and the graph smoke test both
passed. The corrected installer then completed `rosdep init` and the Jazzy-only
rosdep cache update.

The first D3 launch rendered Panda and the planning scene, but manual planning
failed. Diagnostics showed that only `panda_hand_controller` was active;
`panda_arm_controller` and `joint_state_broadcaster` were absent, and
`/joint_states` produced no messages. The installed Panda config declares those
controllers but its binary dependencies did not install the corresponding
`joint_trajectory_controller` and `joint_state_broadcaster` plugins. D3 remains
FAIL at that point, pending installation of those exact runtime packages and a
clean re-test.

After installing the two missing controller plugins (and the optional
`rviz_visual_tools` plugin), the hardened smoke script verified actual joint
state messages and active arm/joint-state controllers before allowing manual
confirmation. Planning from `<current>` to `extended` completed in 0.011 s;
RViz displayed the trajectory and the user confirmed motion. D3 therefore PASS.

The first D4 run installed all requested binary packages and passed both the
headless empty-world service check and the visible Gazebo GUI check. The cart
controller command then connected to a stale Panda `/controller_manager` left
by the prior background launch, so `joint_trajectory_controller` was inactive
and rejected the goal. The stale launch trees were terminated. Smoke scripts
now use process groups with TERM/wait cleanup; D3 and D4 control use separate
ROS domains, D4 uses a unique Gazebo partition, and the cart demo runs with
server-only `gz_args:=-s`. The clean re-test then passed: the cart joint-state
broadcaster and trajectory controller were active, the goal was accepted, and
all commanded positions completed successfully. No Phase 0 test process was
left running after cleanup.

The pristine Noble apt cache has no `ros-dev-tools` candidate. The reproducible
D1 installer therefore adds the official `ros2-apt-source` configuration after
installing the Ubuntu-native tools, then installs `ros-dev-tools`. D2 remains
the first gate that installs and runs ROS Desktop.

Git was initialized on branch `main`. Because the owner explicitly prohibited
committing, the new scaffold remains intentionally untracked; a literally clean
Git worktree is therefore not yet possible in this brand-new repository. No
commit or push was made.

## GUI result

`glxinfo -B`, RViz, and Gazebo GUI all ran through WSLg. RViz rendered Panda and
displayed a valid planned trajectory. Gazebo rendered the empty world and
responded to play/camera interaction. Rendering uses software `llvmpipe`, so
performance is a known constraint even though the Phase 0 GUI smoke tests pass.

## Current conclusion

All technical Phase 0 gates **D1-D4 PASS**, including simulator-independent
Python/C++ tests. Phase 0 is not declared unconditionally complete only because
the new repository has no initial commit and therefore cannot satisfy the
literal clean-worktree exit criterion without owner authorization. The
scaffold is clean of generated outputs; `build/`, `install/`, `log/`, bags,
metrics, datasets, and videos are ignored. No Phase 1 work or CV-readiness claim
has started.
