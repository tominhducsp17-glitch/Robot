#!/usr/bin/env python3
"""Ground-truth geometric baseline for Phase 1.

This module intentionally reads configured simulator poses. It is not a
deployable controller and must be replaced by perception beginning in Phase 2.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import subprocess
import time

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3, Vector3Stamped
from moveit.task_constructor import core, stages
from moveit_msgs.msg import CollisionObject
import rclcpp
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


ARM = "panda_arm"
HAND = "hand"
PEG = "phase1_peg"
WORLD = "world"
HAND_LINK = "panda_hand"
TCP_LINK = "panda_link8"
CONTACT_LINKS = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]

TABLE_X = 0.55
TABLE_Z = 0.375
TABLE_SIZE = (0.90, 1.00, 0.75)
PEG_RADIUS = 0.018
PEG_HEIGHT = 0.12
INSERTION_DEPTH = 0.05
GRASP_Z_OFFSET = 0.03
FIXTURE_X = 0.58
FIXTURE_Y = -0.20
FIXTURE_BASE_Z = 0.80
TCP_OFFSET = 0.1034


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("task_a", "task_b"), default="task_b")
    parser.add_argument("--peg-x", type=float, default=0.38)
    parser.add_argument("--peg-y", type=float, default=0.20)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--max-solutions", type=int, default=5)
    parser.add_argument("--log-dir", type=Path, default=Path("experiments/raw/phase1"))
    return parser.parse_known_args()[0]


def pose_stamped(x, y, z, *, top_down=False):
    orientation = Quaternion(x=1.0, w=0.0) if top_down else Quaternion(w=1.0)
    return PoseStamped(
        header=Header(frame_id=WORLD),
        pose=Pose(position=Point(x=x, y=y, z=z), orientation=orientation),
    )


def identity_in_frame(frame_id):
    return PoseStamped(
        header=Header(frame_id=frame_id),
        pose=Pose(orientation=Quaternion(w=1.0)),
    )


def box_object(object_id, size, xyz):
    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
    pose = Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]), orientation=Quaternion(w=1.0))
    return CollisionObject(
        header=Header(frame_id=WORLD),
        id=object_id,
        primitives=[primitive],
        primitive_poses=[pose],
        operation=CollisionObject.ADD,
    )


def cylinder_object(object_id, radius, height, xyz):
    primitive = SolidPrimitive(
        type=SolidPrimitive.CYLINDER,
        dimensions=[height, radius],
    )
    pose = Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]), orientation=Quaternion(w=1.0))
    return CollisionObject(
        header=Header(frame_id=WORLD),
        id=object_id,
        primitives=[primitive],
        primitive_poses=[pose],
        operation=CollisionObject.ADD,
    )


def add_nominal_scene(stage, peg_x, peg_y):
    stage.addObject(box_object("work_table", TABLE_SIZE, (TABLE_X, 0.0, TABLE_Z)))
    stage.addObject(cylinder_object(PEG, PEG_RADIUS, PEG_HEIGHT, (peg_x, peg_y, 0.812)))

    walls = (
        ("fixture_x_pos", (0.02, 0.09, 0.10), (FIXTURE_X + 0.035, FIXTURE_Y, 0.85)),
        ("fixture_x_neg", (0.02, 0.09, 0.10), (FIXTURE_X - 0.035, FIXTURE_Y, 0.85)),
        ("fixture_y_pos", (0.05, 0.02, 0.10), (FIXTURE_X, FIXTURE_Y + 0.035, 0.85)),
        ("fixture_y_neg", (0.05, 0.02, 0.10), (FIXTURE_X, FIXTURE_Y - 0.035, 0.85)),
    )
    for object_id, size, xyz in walls:
        stage.addObject(box_object(object_id, size, xyz))


def cartesian_stage(name, planner, dz):
    stage = stages.MoveRelative(name, planner)
    stage.group = ARM
    stage.ik_frame = identity_in_frame(TCP_LINK)
    stage.min_distance = abs(dz) * 0.95
    stage.max_distance = abs(dz)
    stage.setDirection(
        Vector3Stamped(
            header=Header(frame_id=WORLD),
            vector=Vector3(z=1.0 if dz > 0.0 else -1.0),
        )
    )
    return stage


def move_arm_to(name, planner, pose):
    stage = stages.MoveTo(name, planner)
    stage.group = ARM
    stage.ik_frame = identity_in_frame(TCP_LINK)
    stage.setGoal(pose)
    stage.timeout = 10.0
    return stage


def move_hand_to(name, planner, named_pose):
    stage = stages.MoveTo(name, planner)
    stage.group = HAND
    stage.setGoal(named_pose)
    stage.timeout = 5.0
    return stage


def move_arm_named(name, planner, named_pose):
    stage = stages.MoveTo(name, planner)
    stage.group = ARM
    stage.setGoal(named_pose)
    stage.timeout = 10.0
    return stage


def make_task(name, node):
    task = core.Task()
    task.name = name
    task.loadRobotModel(node)
    return task


def build_grasp_task(node, peg_x, peg_y):
    pipeline = core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault")
    cartesian = core.CartesianPath()
    cartesian.max_velocity_scaling_factor = 0.10
    cartesian.max_acceleration_scaling_factor = 0.10
    joints = core.JointInterpolationPlanner()

    task = make_task("phase1/01_grasp", node)
    task.add(stages.CurrentState("current state"))

    detach_old = stages.ModifyPlanningScene("detach object from previous episode")
    detach_old.detachObject(PEG, HAND_LINK)
    task.add(detach_old)

    cleanup = stages.ModifyPlanningScene("remove previous episode scene")
    for object_id in (
        PEG,
        "work_table",
        "fixture_x_pos",
        "fixture_x_neg",
        "fixture_y_pos",
        "fixture_y_neg",
    ):
        cleanup.removeObject(object_id)
    task.add(cleanup)

    scene = stages.ModifyPlanningScene("add nominal ground-truth scene")
    add_nominal_scene(scene, peg_x, peg_y)
    task.add(scene)

    allow = stages.ModifyPlanningScene("allow peg/finger contact")
    allow.allowCollisions(PEG, CONTACT_LINKS, True)
    task.add(allow)
    task.add(move_hand_to("open gripper", joints, "open"))

    # Grasp 30 mm above the peg center so the Panda hand housing clears the
    # peg top while the finger pads still overlap the part.
    pregrasp_link8_z = 0.812 + TCP_OFFSET + GRASP_Z_OFFSET + 0.14
    task.add(
        move_arm_to(
            "move to pre-grasp",
            pipeline,
            pose_stamped(peg_x, peg_y, pregrasp_link8_z, top_down=True),
        )
    )
    task.add(cartesian_stage("approach peg", cartesian, -0.14))
    return task, (pipeline, cartesian, joints)


def build_transfer_task(node, task_name):
    pipeline = core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault")
    cartesian = core.CartesianPath()
    cartesian.max_velocity_scaling_factor = 0.08
    cartesian.max_acceleration_scaling_factor = 0.08
    joints = core.JointInterpolationPlanner()

    task = make_task("phase1/02_transfer_and_place", node)
    task.add(stages.CurrentState("current state"))
    task.add(move_hand_to("close gripper", joints, "closed"))

    attach = stages.ModifyPlanningScene("attach peg to hand")
    attach.attachObject(PEG, HAND_LINK)
    task.add(attach)
    task.add(cartesian_stage("lift peg", cartesian, 0.15))

    if task_name == "task_a":
        target = pose_stamped(
            0.58,
            0.20,
            0.812 + TCP_OFFSET + GRASP_Z_OFFSET,
            top_down=True,
        )
        task.add(move_arm_to("transfer to wide place target", pipeline, target))
    else:
        preinsert = pose_stamped(
            FIXTURE_X,
            FIXTURE_Y,
            0.98 + TCP_OFFSET + GRASP_Z_OFFSET,
            top_down=True,
        )
        task.add(move_arm_to("transfer to pre-insert", pipeline, preinsert))
        # Keep the wider Panda fingers above the 50 mm fixture opening.  The
        # 120 mm peg still enters 50 mm before release and settles onto the
        # table inside the fixture under gravity.
        task.add(
            cartesian_stage(
                "insert peg 50 mm with large clearance",
                cartesian,
                -INSERTION_DEPTH,
            )
        )

    task.add(move_hand_to("release peg", joints, "open"))
    return task, (pipeline, cartesian, joints)


def build_retreat_task(node):
    pipeline = core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault")
    cartesian = core.CartesianPath()
    cartesian.max_velocity_scaling_factor = 0.10
    joints = core.JointInterpolationPlanner()

    task = make_task("phase1/03_detach_and_retreat", node)
    task.add(stages.CurrentState("current state"))

    detach = stages.ModifyPlanningScene("detach peg from hand")
    detach.detachObject(PEG, HAND_LINK)
    task.add(detach)
    task.add(cartesian_stage("retreat", cartesian, 0.15))

    restore_collisions = stages.ModifyPlanningScene("restore peg collision checking")
    restore_collisions.allowCollisions(PEG, CONTACT_LINKS, False)
    task.add(restore_collisions)
    task.add(move_arm_named("return ready", pipeline, "ready"))
    return task, (pipeline, cartesian, joints)


def gz_topic(topic):
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", "unused: true"],
        check=True,
    )


def reset_gazebo_peg(x, y):
    gz_topic("/phase1/peg/detach")
    # The transport publish is asynchronous.  Let Gazebo remove the fixed
    # joint before teleporting the free peg, otherwise the arm can drag the
    # pose command away on the next physics step.
    time.sleep(0.50)
    request = (
        f'name: "peg", position: {{x: {x}, y: {y}, z: 0.812}}, '
        "orientation: {w: 1.0}"
    )
    subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/world/phase1_baseline/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            request,
        ],
        check=True,
    )


def reset_gripper():
    goal = "{command: {position: 0.035, max_effort: 20.0}}"
    subprocess.run(
        [
            "ros2",
            "action",
            "send_goal",
            "/panda_hand_controller/gripper_cmd",
            "control_msgs/action/GripperCommand",
            goal,
        ],
        check=True,
        timeout=15,
    )


def reset_arm():
    joints = [f"panda_joint{index}" for index in range(1, 8)]
    positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    names_yaml = ", ".join(f'"{name}"' for name in joints)
    positions_yaml = ", ".join(str(value) for value in positions)
    goal = (
        "{trajectory: {joint_names: ["
        + names_yaml
        + "], points: [{positions: ["
        + positions_yaml
        + "], time_from_start: {sec: 4}}]}}"
    )
    subprocess.run(
        [
            "ros2",
            "action",
            "send_goal",
            "/panda_arm_controller/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            goal,
        ],
        check=True,
        timeout=20,
    )


def git_commit():
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except subprocess.CalledProcessError:
        return "unknown"


def force_monitor(action):
    """Reset or read the independent ROS contact monitor."""
    try:
        result = subprocess.run(
            ["ros2", "run", "experiment_runner", "force_monitor_client", action],
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        )
        marker = "PHASE1_FORCE="
        line = next(
            output_line
            for output_line in reversed(result.stdout.splitlines())
            if marker in output_line
        )
        return json.loads(line.split(marker, 1)[1])
    except (
        subprocess.CalledProcessError,
        subprocess.TimeoutExpired,
        StopIteration,
        json.JSONDecodeError,
    ) as error:
        return {
            "available": False,
            "topic": "/phase1/peg/contacts",
            "force_limit_n": None,
            "contact_message_count": 0,
            "wrench_sample_count": 0,
            "peak_force_n": None,
            "peak_axial_force_n": None,
            "peak_lateral_force_n": None,
            "rms_force_n": None,
            "force_violation": None,
            "error": str(error),
        }


def run_segment(task, max_solutions, execute):
    started = time.monotonic()
    planned = bool(task.plan(max_solutions))
    planning_seconds = time.monotonic() - started
    result = {
        "name": task.name,
        "planned": planned,
        "planning_seconds": round(planning_seconds, 6),
        "solutions": len(task.solutions),
        "executed": False,
        "execution_seconds": None,
    }
    if not planned:
        return False, result

    task.publish(task.solutions[0])
    if not execute:
        return True, result

    started = time.monotonic()
    executed = bool(task.execute(task.solutions[0]))
    result["execution_seconds"] = round(time.monotonic() - started, 6)
    result["executed"] = executed
    return executed, result


def gazebo_peg_pose():
    last_error = None
    for _ in range(3):
        try:
            result = subprocess.run(
                ["gz", "model", "-m", "peg", "-p"],
                check=True,
                capture_output=True,
                text=True,
                timeout=10,
            )
            pose = re.search(
                r"Pose \[ XYZ \(m\) \].*?:\s*\n\s*"
                r"\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]\s*\n\s*"
                r"\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]",
                result.stdout,
            )
            if pose is not None:
                break
            last_error = RuntimeError(
                f"Could not parse Gazebo peg pose:\n{result.stdout}"
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
            last_error = error
        time.sleep(0.25)
    else:
        raise RuntimeError("Gazebo peg pose query failed after 3 attempts") from last_error
    return {
        "x": float(pose.group(1)),
        "y": float(pose.group(2)),
        "z": float(pose.group(3)),
        "roll": float(pose.group(4)),
        "pitch": float(pose.group(5)),
        "yaw": float(pose.group(6)),
    }


def evaluate_physical_outcome(task_name, pose):
    target_x = 0.58
    target_y = 0.20 if task_name == "task_a" else FIXTURE_Y
    xy_error = math.hypot(pose["x"] - target_x, pose["y"] - target_y)
    upright = abs(pose["roll"]) < 0.15 and abs(pose["pitch"]) < 0.15
    on_table = 0.795 <= pose["z"] <= 0.835
    tolerance = 0.05 if task_name == "task_a" else 0.006
    return {
        "success": xy_error <= tolerance and upright and on_table,
        "target_x": target_x,
        "target_y": target_y,
        "xy_error_m": xy_error,
        "xy_tolerance_m": tolerance,
        "upright": upright,
        "on_table": on_table,
    }


def write_log(
    args,
    segments,
    success,
    task_success,
    started_at,
    physical_states,
    physical_outcome,
    force_metrics,
):
    args.log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    path = args.log_dir / f"{args.task}_{stamp}.json"
    payload = {
        "schema_version": 1,
        "phase": 1,
        "task": args.task,
        "mode": "ground_truth_geometric_baseline",
        "simulation_only": True,
        "deployable_controller": False,
        "git_commit": git_commit(),
        "started_at": started_at,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "success": success,
        "task_success": task_success,
        "plan_only": args.plan_only,
        "ground_truth": {
            "peg_pose": {"x": args.peg_x, "y": args.peg_y, "z": 0.812},
            "fixture_pose": {"x": FIXTURE_X, "y": FIXTURE_Y, "z": FIXTURE_BASE_Z},
            "peg_diameter_m": PEG_RADIUS * 2.0,
            "opening_width_m": 0.05,
            "diametral_clearance_m": 0.05 - PEG_RADIUS * 2.0,
            "commanded_insertion_depth_m": (
                INSERTION_DEPTH if args.task == "task_b" else None
            ),
        },
        "segments": segments,
        "gazebo_peg_poses": physical_states,
        "physical_outcome": physical_outcome,
        "force_monitor_available": force_metrics["available"],
        "force_violation": force_metrics["force_violation"],
        "contact_force": force_metrics,
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"PHASE1_LOG={path}")


def main():
    args = parse_args()
    started_at = datetime.now(timezone.utc).isoformat()
    segments = []
    success = False
    task_success = False
    physical_states = {}
    physical_outcome = None
    force_metrics = None
    force_reset_succeeded = False

    rclcpp.init()
    options = rclcpp.NodeOptions()
    options.automatically_declare_parameters_from_overrides = True
    node = rclcpp.Node("phase1_task", options)

    keep_alive = []
    try:
        reset_gripper()
        reset_arm()
        reset_gazebo_peg(args.peg_x, args.peg_y)
        time.sleep(0.25)
        physical_states["after_reset"] = gazebo_peg_pose()
        reset_metrics = force_monitor("reset")
        force_reset_succeeded = "error" not in reset_metrics

        grasp, planners = build_grasp_task(node, args.peg_x, args.peg_y)
        keep_alive.extend(planners)
        ok, result = run_segment(grasp, args.max_solutions, not args.plan_only)
        segments.append(result)
        if args.plan_only:
            task_success = ok
        elif ok:
            gz_topic("/phase1/peg/attach")
            time.sleep(0.25)
            physical_states["after_attach"] = gazebo_peg_pose()
            transfer, planners = build_transfer_task(node, args.task)
            keep_alive.extend(planners)
            ok, result = run_segment(transfer, args.max_solutions, True)
            segments.append(result)

            if ok:
                time.sleep(0.25)
                physical_states["before_detach"] = gazebo_peg_pose()
                gz_topic("/phase1/peg/detach")
                time.sleep(0.50)
                physical_states["after_detach"] = gazebo_peg_pose()
                retreat, planners = build_retreat_task(node)
                keep_alive.extend(planners)
                ok, result = run_segment(retreat, args.max_solutions, True)
                segments.append(result)
                time.sleep(0.50)
                physical_states["final"] = gazebo_peg_pose()
                physical_outcome = evaluate_physical_outcome(
                    args.task, physical_states["final"]
                )
                task_success = ok and physical_outcome["success"]
    finally:
        force_metrics = force_monitor("snapshot")
        force_metrics["reset_succeeded"] = force_reset_succeeded
        force_metrics["available"] = (
            force_metrics["available"] and force_reset_succeeded
        )
        if not force_metrics["available"]:
            force_metrics["force_violation"] = None
        if args.plan_only:
            force_metrics["force_violation"] = None
            success = task_success
        else:
            success = (
                task_success
                and force_metrics["available"]
                and force_metrics["force_violation"] is False
            )
        write_log(
            args,
            segments,
            success,
            task_success,
            started_at,
            physical_states,
            physical_outcome,
            force_metrics,
        )
        del keep_alive
        rclcpp.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
