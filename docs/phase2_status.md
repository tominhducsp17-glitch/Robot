# Phase 2 status

Last runtime verification: 2026-08-16 on Ubuntu 24.04 / WSL2, ROS 2 Jazzy,
MoveIt 2, MoveIt Task Constructor, Gazebo Harmonic, and OpenCV 4.6.

## Implemented

- 800x600 overhead RGB-D camera with `CameraInfo` intrinsics and a calibrated
  `world -> phase1_camera_optical` transform.
- Gazebo-rendered ArUco `DICT_4X4_50` tags: ID 0 on the peg and ID 1 beside the
  fixture, decoded by a project-owned ROS 2 Python node.
- Depth-corrected 3D position, pose covariance, confidence, translation-outlier
  rejection, exponential filtering, sample counts, and stale-data checks.
- `manipulation_msgs/ObjectPose` and `ObjectPoseArray` interfaces on
  `/perception/object_poses`.
- `/perception/reset`, `/perception/snapshot`, and `/perception/freeze` services
  so the planner consumes one internally consistent observation window.
- Perceived peg and fixture collision objects on `/collision_object`.
- Phase 2 MTC execution in which the frozen peg estimate drives the grasp and
  the frozen fixture estimate drives pre-insert. Simulator truth is logged only
  by the evaluator.
- One-way Gazebo-to-ROS camera bridges, preventing RGB/depth feedback loops.
- Five-pose JSON benchmark with pose, latency, task, physical-outcome, and
  contact-force gates.

## Data guards and gate thresholds

| Guard | Threshold |
|---|---:|
| Minimum confidence | 0.50 |
| Minimum accepted samples | 10 |
| Stale observation | >0.50 s |
| Translation jump outlier | >0.05 m |
| Phase 2 3D pose error | <=0.015 m |
| Phase 2 measurement age | <=0.50 s |
| Task B success | 5/5 (>=95%) |
| Contact force | no violation of 50 N evaluator limit |

RGB and depth can arrive several hundred milliseconds apart under WSL GPU
load. The estimator accepts depth at most 2 s from the RGB timestamp while the
scene is stationary during snapshot acquisition, and it drops RGB detections
rather than falling back to a biased monocular estimate when usable depth is
absent. The published measurement age is still required to remain below 0.5 s.

## Verified evidence

The verified five-pose Task B run completed 5/5 episodes successfully using
`perception_snapshot` planning inputs. Across the ten object estimates:

- maximum 3D translation error: 0.002266 m;
- maximum measurement age: 0.232 s;
- force-limit violations: 0;
- maximum peak contact force: 12.495 N.

Generated evidence is intentionally ignored by Git. Reproduce it with:

```bash
ros2 launch manipulation_bringup phase2.launch.py
# In a second sourced terminal:
./scripts/phase2/run_vision_benchmark.sh
```

## Gate status and limits

The Phase 2 exit criterion is **passed**: pose error and latency remained below
their declared limits, and the robot grasped and reached/inserted at the
pre-insert target using vision poses rather than configured poses.

This remains simulation-only and tag-based. It is not visual servoing: the
snapshot is frozen before motion, so object movement after planning is deferred
to Phase 3. It is also not compliant control or a hardware-safety claim.

The known MoveGroup destructor segmentation fault can still appear only while
stopping the combined bringup after a run; it has not affected runtime results.
