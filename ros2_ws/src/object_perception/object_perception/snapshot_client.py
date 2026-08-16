#!/usr/bin/env python3
"""CLI client for Phase 2 perception reset, snapshot, and freeze services."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("reset", "snapshot", "freeze"))
    parser.add_argument("--require-ready", action="store_true")
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("perception_snapshot_client")
    service_name = f"/perception/{args.action}"
    client = node.create_client(Trigger, service_name)
    deadline = time.monotonic() + args.timeout
    try:
        if not client.wait_for_service(timeout_sec=args.timeout):
            raise RuntimeError(f"service unavailable: {service_name}")
        while time.monotonic() < deadline:
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            response = future.result()
            if response is not None:
                snapshot = json.loads(response.message)
                if response.success and (
                    not args.require_ready or snapshot.get("ready", False)
                ):
                    print(
                        "PHASE2_PERCEPTION="
                        + json.dumps(snapshot, separators=(",", ":"))
                    )
                    return
            time.sleep(0.1)
        raise RuntimeError(f"timed out waiting for valid perception: {service_name}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
