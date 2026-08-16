# Architecture

The full planned architecture is defined in sections 3-5 of
`CLOSED_LOOP_VISION_COMPLIANT_MANIPULATION_PLAN.md`. Phase 1 implements only the
geometric simulation path:

```text
configured ground-truth poses
  -> MoveIt Task Constructor stages
  -> MoveGroup / OMPL
  -> ros2_control trajectory and gripper controllers
  -> Gazebo Harmonic Panda + detachable peg
  -> final Gazebo pose validation + JSON log
```

Package responsibilities:

- `manipulation_description`: Panda simulation URDF, controllers, table, peg,
  fixture, and overhead RGB-D camera.
- `manipulation_moveit_config`: SRDF, kinematics, OMPL, joint limits, and
  controller mapping.
- `manipulation_bringup`: one-command Gazebo, bridge, controllers, MoveGroup,
  and optional RViz launch.
- `experiment_runner`: Task A/Task B MTC orchestration and episode logging.

The runner splits execution around physical attach/detach events so the Gazebo
peg and MoveIt collision object remain synchronized. A result is successful
only when all MTC segments execute and the final Gazebo peg pose satisfies the
task tolerance.

There is deliberately no edge from the camera to the controller in Phase 1.
Camera topics exist for later phases, but configured ground truth drives the
baseline. Perception, visual servo, compliance, failure detection, and recovery
packages remain placeholders.
