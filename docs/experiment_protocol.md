# Experiment protocol

The planned full benchmark and metrics are defined in sections 6-7 and 16.3 of
`CLOSED_LOOP_VISION_COMPLIANT_MANIPULATION_PLAN.md`. Phase 1 adds a narrow
simulation-only nominal protocol.

## Phase 1 nominal protocol

- Task A: pick the upright cylindrical peg and place it in a wide target area.
- Task B: pick the same peg, transfer it over a 50 mm square opening, insert it
  50 mm, release it, and retreat.
- Peg diameter: 36 mm; Task B diametral clearance: 14 mm.
- Inputs: exact configured poses; no perception output is consumed.
- Task B initial positions: center and +/-20 mm on each table axis.
- One episode is successful only when planning and execution succeed for every
  segment and the final physical peg is upright, on the table, and inside the
  position tolerance (6 mm for Task B; 50 mm wide-area tolerance for Task A).
- The peg contact sensor is bridged to ROS 2 at `/phase1/peg/contacts`. The
  evaluator resets before motion and records contact message count, wrench
  sample count, peak total/axial (world Z)/lateral force, and RMS total force.
- The nominal force limit is 50 N. An executed episode fails when the sensor
  topic is unavailable or its measured peak total force exceeds this limit.

Run the five-pose matrix with `scripts/phase1/run_nominal_benchmark.sh`. Each
episode records planning/execution times, intermediate/final Gazebo poses, and
the physical outcome. The summary evaluates the 95% success-rate gate from the
JSON values, not the wrapper process exit code.

## Verified nominal result

The verified five-pose Task B run on 2026-08-16 achieved 5/5 trajectory and
physical successes, zero force-limit violations, and a maximum peak contact
force of 12.333 N. The generated summary reported `phase1_exit_gate: true`.

This is evaluator-side simulation ground truth, not a deployable force-feedback
controller. Compliant control, active force limiting, and retreat behavior
remain later-phase work.
