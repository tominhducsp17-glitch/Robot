# Failure taxonomy

The planned failure codes and bounded recovery mapping are defined in sections
5.8-5.9 of `CLOSED_LOOP_VISION_COMPLIANT_MANIPULATION_PLAN.md`. Phase 1 records
planning failure, execution failure, Gazebo query failure, final physical pose
failure, unavailable contact instrumentation, and force-limit violation through
its JSON result and nonzero runner exit status. Force violations are evaluated
after the episode; Phase 1 does not implement an online stop, runtime failure
detector, retry policy, or recovery behavior.
