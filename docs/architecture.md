# Architecture

The full planned architecture is defined in sections 3-5 of
`CLOSED_LOOP_VISION_COMPLIANT_MANIPULATION_PLAN.md`. Phases 1 and 2 implement
the geometric and perception-driven simulation paths:

```text
configured ground-truth poses
  -> MoveIt Task Constructor stages
  -> MoveGroup / OMPL
  -> ros2_control trajectory and gripper controllers
  -> Gazebo Harmonic Panda + detachable peg
  -> final Gazebo pose validation + JSON log

Gazebo peg contact sensor
  -> ros_gz_bridge Contacts topic
  -> independent force evaluator (peak/RMS/limit)
  -> JSON log + nominal exit gate

Gazebo RGB-D camera + CameraInfo + calibrated optical TF
  -> OpenCV ArUco IDs 0 (peg) and 1 (fixture)
  -> RGB-D pose estimate -> outlier rejection / EMA / confidence / age checks
  -> ObjectPoseArray + perception collision objects
  -> reset -> valid snapshot -> freeze
  -> MTC grasp and pre-insert targets (no ground-truth edge to planning)

Gazebo ground truth
  -> pose-error and final-outcome evaluator only
  -> Phase 2 JSON log + five-pose exit gate
```

Package responsibilities:

- `manipulation_description`: Panda simulation URDF, controllers, table, peg,
  fixture, and overhead RGB-D camera.
- `manipulation_moveit_config`: SRDF, kinematics, OMPL, joint limits, and
  controller mapping.
- `manipulation_bringup`: one-command Gazebo, bridge, controllers, MoveGroup,
  and optional RViz launch.
- `experiment_runner`: Task A/Task B MTC orchestration, contact-force evaluator,
  and episode logging.
- `manipulation_msgs`: project-owned timestamped pose/confidence interfaces.
- `object_perception`: calibrated ArUco/RGB-D pose estimation, filtering,
  snapshot services, and planning-scene publication.

The runner splits execution around physical attach/detach events so the Gazebo
peg and MoveIt collision object remain synchronized. A result is successful
only when all MTC segments execute, the final Gazebo peg pose satisfies the
task tolerance, the contact topic is available, and peak force remains within
the configured evaluator limit.

There is deliberately no edge from the camera to the controller in Phase 1.
Phase 2 adds that edge through an immutable snapshot: the runner resets the
filter after placing the peg, waits for both observations to satisfy the data
guards, freezes them, then uses those positions for the planning scene and MTC
targets. The same runner still knows the commanded simulator pose solely so it
can score perception and physical outcome after the episode.

The contact monitor remains evaluator-side simulation instrumentation and does
not command the robot. Visual servo, compliance, runtime failure detection,
and recovery packages remain placeholders.
