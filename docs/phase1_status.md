# Phase 1 status

Last runtime verification: 2026-08-16 on Ubuntu 24.04 / WSL2, ROS 2 Jazzy,
MoveIt 2, MoveIt Task Constructor, and Gazebo Harmonic.

## Implemented

- Panda, table, 36 mm cylindrical peg, four-wall 50 mm fixture, and overhead
  RGB-D camera in Gazebo.
- Active arm, gripper, and joint-state ros2_control controllers.
- MoveIt planning scene and MTC execution capability.
- Ground-truth Task A pick-and-place and Task B 50 mm large-clearance insertion.
- Explicit pre-grasp, approach, close, attach, lift, transfer/pre-insert,
  insert, release, detach, retreat, and return-ready stages.
- JSON logging of segment planning/execution and intermediate/final physical
  peg poses.
- Five valid nominal initial poses and a reproducible benchmark command.
- Gazebo peg contact sensor, ROS bridge, independent force evaluator, and
  per-episode peak/RMS force logging with a configurable 50 N nominal limit.

## Verified evidence

- All four Phase 1 packages build successfully.
- Gazebo models, camera topics, MoveGroup, and all three controllers run.
- Task A completed with all three execution segments successful and final
  planar placement error of approximately 0.053 mm in the recorded run.
- Task B completed with all three execution segments successful and final
  planar insertion error of approximately 0.187 mm in the center run.
- The center and four +/-20 mm initial-pose cases have each completed
  successfully after adding retry handling for transient Gazebo pose queries.
- The final five-pose verification achieved 5/5 overall successes, zero
  force-limit violations, and a maximum peak contact force of 12.333 N.

Generated run logs are intentionally ignored by Git; rerun
`scripts/phase1/run_nominal_benchmark.sh` to produce a local evidence summary.

## Gate status

The full Phase 1 exit criterion is **passed** in the verified five-pose nominal
run: trajectory-and-physical success was 5/5 (100%), the contact monitor was
available in every episode, and no episode exceeded the 50 N evaluator limit.
Phase 2 subsequently passed its separate vision-driven gate; see
`docs/phase2_status.md`.

## Known runtime issue

After a completed run, manually stopping the combined bringup with Ctrl-C can
produce a `move_group` destructor segmentation fault while unloading the MTC
capability. This has only been observed during shutdown after the controllers
and simulator receive the stop signal; it did not invalidate any running task
or recorded outcome. It remains an integration issue to investigate rather
than a clean-shutdown claim.

This is a simulation-only geometric baseline. It uses exact configured poses,
and its force check is evaluator-side instrumentation rather than active force
control. It is not a deployable controller or evidence of perception, visual
servo, compliance, recovery, or overall CV readiness.
