# Closed-Loop Vision-Guided Compliant Manipulation

Closed-loop pick-and-insert research project for a simulated Franka Panda. The
intended system combines camera-based relative-pose estimation, MoveIt 2 motion
planning, bounded visual alignment, compliant insertion, verification, and
bounded failure recovery.

## Current status

**Phase 0 is complete and Phase 1 is in progress.** The repository now contains
a simulation-only, ground-truth geometric baseline for Task A pick-and-place
and Task B large-clearance insertion. Perception, visual servoing, compliant
control, force-limit monitoring, and recovery are not implemented yet.

See:

- `CLOSED_LOOP_VISION_COMPLIANT_MANIPULATION_PLAN.md` for the complete plan.
- `docs/environment_audit.md` for machine and gate evidence.
- `docs/version_matrix.md` for the supported dependency matrix.
- `docs/phase1_status.md` for Phase 1 evidence and remaining gate.
- `THIRD_PARTY.md` for the boundary between upstream software and future
  project-owned work.

## Phase 0 quick start

Ubuntu 24.04 (Noble) is required. Run these commands from a normal Ubuntu/WSL
shell; do not use Conda.

### D1: base toolchain

```bash
sudo apt update
./scripts/phase0/install_d1.sh
./scripts/phase0/smoke_toolchain.sh
```

`install_d1.sh` also configures the official `ros2-apt-source` package before
installing `ros-dev-tools`, because that meta-package is supplied by the ROS apt
repository rather than the pristine Noble sources. It does not install ROS
Desktop, MoveIt, or Gazebo.

### D2: ROS 2 Jazzy

```bash
./scripts/phase0/install_d2.sh && ./scripts/phase0/smoke_ros_graph.sh
```

### D3: MoveIt 2 and Panda resources

```bash
./scripts/phase0/install_d3.sh && ./scripts/phase0/smoke_moveit_panda.sh
```

The D3 script verifies the model, `move_group`, planning-scene topic, and RViz
process. In RViz, set a collision-free goal with the interactive marker, click
**Plan**, confirm a trajectory is displayed, then press Enter in the terminal
to record the manual GUI check.

### D4: Gazebo Harmonic and ros2_control

Install only the Jazzy binary packages from the ROS repository; these select
Gazebo Harmonic through ROS vendor packages.

```bash
./scripts/phase0/install_d4.sh && \
  ./scripts/phase0/smoke_gazebo.sh && \
  ./scripts/phase0/smoke_gz_ros2_control.sh
```

### Repository tests (no simulator)

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
cmake -S . -B build/repository-tests -G Ninja
cmake --build build/repository-tests
ctest --test-dir build/repository-tests --output-on-failure
```

Generated `build/`, `install/`, `log/`, bags, metrics, datasets, and videos are
ignored by Git.

## Phase 1 quick start

Install the MoveIt Task Constructor binary dependencies once:

```bash
./scripts/phase1/install_dependencies.sh
```

Build the four Phase 1 packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_ws/src \
  --packages-select manipulation_description manipulation_moveit_config \
  manipulation_bringup experiment_runner
source install/setup.bash
```

Keep the simulator running in terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch manipulation_bringup phase1.launch.py
```

Run a task in terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch experiment_runner phase1_task.launch.py task:=task_a
ros2 launch experiment_runner phase1_task.launch.py task:=task_b
```

For the five-pose Task B matrix, leave terminal 1 running and execute:

```bash
./scripts/phase1/run_nominal_benchmark.sh
```

Episode JSON is written below `experiments/raw/phase1/`; benchmark summaries go
below `experiments/summaries/phase1/`. Both locations are intentionally ignored
by Git.

## Scope boundaries

- The Phase 1 runner intentionally uses configured simulator ground truth and
  marks every log `simulation_only: true` and `deployable_controller: false`.
- No learned-perception/CUDA dependency is required for the MVP.
- Phase 2 must not start until the remaining Phase 1 force-monitoring gate is
  resolved or the project plan is explicitly revised.
- Simulation-only results must always be described as simulation-only.

## License

No open-source license has been selected yet. See `LICENSE`.
