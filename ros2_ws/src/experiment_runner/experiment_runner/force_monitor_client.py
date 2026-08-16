#!/usr/bin/env python3
"""CLI client used by the MTC runner to reset or read force metrics."""

import argparse
import json

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("reset", "snapshot"))
    parser.add_argument("--timeout", type=float, default=5.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Node("force_monitor_client")
    service_name = f"/phase1/contact_monitor/{args.action}"
    client = node.create_client(Trigger, service_name)
    try:
        if not client.wait_for_service(timeout_sec=args.timeout):
            raise RuntimeError(f"service unavailable: {service_name}")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"service call failed: {service_name}")
        metrics = json.loads(response.message)
        print("PHASE1_FORCE=" + json.dumps(metrics, separators=(",", ":")))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
