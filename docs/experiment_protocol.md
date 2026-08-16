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

Run the five-pose matrix with `scripts/phase1/run_nominal_benchmark.sh`. Each
episode records planning/execution times, intermediate/final Gazebo poses, and
the physical outcome. The summary evaluates the 95% success-rate gate from the
JSON values, not the wrapper process exit code.

## Current limitation

Contact-force monitoring is not present. Logs therefore record
`force_monitor_available: false` and `force_violation: null`. The benchmark may
pass its trajectory-and-physical-outcome gate, but the overall Phase 1 exit gate
must remain false until contact-force violations can be evaluated.
