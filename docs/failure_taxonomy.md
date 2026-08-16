# Failure taxonomy

The planned failure codes and bounded recovery mapping are defined in sections
5.8-5.9 of `CLOSED_LOOP_VISION_COMPLIANT_MANIPULATION_PLAN.md`. Phase 1 records
planning failure, execution failure, Gazebo query failure, and final physical
pose failure through its JSON result and nonzero runner exit status. It does not
implement a runtime failure detector, retry policy, or recovery behavior.
