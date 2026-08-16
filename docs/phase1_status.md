# Phase 1 status

Last runtime verification: 2026-08-15 on Ubuntu 24.04 / WSL2, ROS 2 Jazzy,
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

## Verified evidence

- All four Phase 1 packages build successfully.
- Gazebo models, camera topics, MoveGroup, and all three controllers run.
- Task A completed with all three execution segments successful and final
  planar placement error of approximately 0.053 mm in the recorded run.
- Task B completed with all three execution segments successful and final
  planar insertion error of approximately 0.187 mm in the center run.
- The center and four +/-20 mm initial-pose cases have each completed
  successfully after adding retry handling for transient Gazebo pose queries.

Generated run logs are intentionally ignored by Git; rerun
`scripts/phase1/run_nominal_benchmark.sh` to produce a local evidence summary.

## Gate status

The trajectory-and-physical-result portion meets the nominal target in the
verified five-pose run. The full Phase 1 exit criterion is **not yet passed**:
the current world has no contact-force sensor or force-limit monitor, so the
"no contact-force violation" condition cannot be evaluated. Phase 2 has not
started.

## Known runtime issue

After a completed run, manually stopping the combined bringup with Ctrl-C can
produce a `move_group` destructor segmentation fault while unloading the MTC
capability. This has only been observed during shutdown after the controllers
and simulator receive the stop signal; it did not invalidate any running task
or recorded outcome. It remains an integration issue to investigate rather
than a clean-shutdown claim.

This is a simulation-only geometric baseline. It uses exact configured poses,
is not a deployable controller, and is not evidence of perception, visual
servo, compliance, recovery, or overall CV readiness.
