# Closed-Loop Vision-Guided Compliant Manipulation

Closed-loop pick-and-insert research project for a simulated Franka Panda. The
intended system combines camera-based relative-pose estimation, MoveIt 2 motion
planning, bounded visual alignment, compliant insertion, verification, and
bounded failure recovery.

## Current status

Only **Phase 0 (environment and repository readiness)** is in scope. No Task B
world, visual-servo, compliance, perception, or recovery implementation exists
yet. A successful upstream Panda demo is an environment smoke test, not a
project contribution and not evidence that the CV-readiness gate has passed.

See:

- `CLOSED_LOOP_VISION_COMPLIANT_MANIPULATION_PLAN.md` for the complete plan.
- `docs/environment_audit.md` for machine and gate evidence.
- `docs/version_matrix.md` for the supported dependency matrix.
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

## Scope boundaries

- Simulator ground truth may eventually be used only by evaluators, never by a
  deployable controller.
- No learned-perception/CUDA dependency is required for the MVP.
- Phase 1 must not start until every Phase 0 smoke test is evidenced as PASS.
- Simulation-only results must always be described as simulation-only.

## License

No open-source license has been selected yet. See `LICENSE`.
