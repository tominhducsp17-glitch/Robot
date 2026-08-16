#!/usr/bin/env python3
"""Aggregate Gazebo peg-contact wrenches for the Phase 1 evaluator."""

import json
import math

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_srvs.srv import Trigger


class ContactForceMonitor(Node):
    def __init__(self):
        super().__init__("contact_force_monitor")
        self.declare_parameter("contact_topic", "/phase1/peg/contacts")
        self.declare_parameter("force_limit_n", 50.0)
        self.contact_topic = self.get_parameter("contact_topic").value
        self.force_limit_n = float(self.get_parameter("force_limit_n").value)
        if self.force_limit_n <= 0.0:
            raise ValueError("force_limit_n must be positive")

        self.subscription = self.create_subscription(
            Contacts,
            self.contact_topic,
            self._on_contacts,
            100,
        )
        self.reset_service = self.create_service(
            Trigger,
            "/phase1/contact_monitor/reset",
            self._reset,
        )
        self.snapshot_service = self.create_service(
            Trigger,
            "/phase1/contact_monitor/snapshot",
            self._snapshot,
        )
        self._clear()
        self.get_logger().info(
            f"Monitoring {self.contact_topic} with {self.force_limit_n:.1f} N limit"
        )

    def _clear(self):
        self.contact_message_count = 0
        self.wrench_sample_count = 0
        self.force_squared_sum = 0.0
        self.peak_force_n = 0.0
        self.peak_axial_force_n = 0.0
        self.peak_lateral_force_n = 0.0

    def _on_contacts(self, message):
        self.contact_message_count += 1
        for contact in message.contacts:
            for joint_wrench in contact.wrenches:
                force = joint_wrench.body_1_wrench.force
                lateral = math.hypot(force.x, force.y)
                axial = abs(force.z)
                total = math.sqrt(force.x**2 + force.y**2 + force.z**2)
                self.wrench_sample_count += 1
                self.force_squared_sum += total**2
                self.peak_force_n = max(self.peak_force_n, total)
                self.peak_axial_force_n = max(self.peak_axial_force_n, axial)
                self.peak_lateral_force_n = max(self.peak_lateral_force_n, lateral)

    def _metrics(self):
        available = self.count_publishers(self.contact_topic) > 0
        rms = (
            math.sqrt(self.force_squared_sum / self.wrench_sample_count)
            if self.wrench_sample_count
            else 0.0
        )
        return {
            "available": available,
            "topic": self.contact_topic,
            "force_limit_n": self.force_limit_n,
            "contact_message_count": self.contact_message_count,
            "wrench_sample_count": self.wrench_sample_count,
            "peak_force_n": self.peak_force_n,
            "peak_axial_force_n": self.peak_axial_force_n,
            "peak_lateral_force_n": self.peak_lateral_force_n,
            "rms_force_n": rms,
            "force_violation": (
                self.peak_force_n > self.force_limit_n if available else None
            ),
        }

    def _reset(self, _request, response):
        self._clear()
        response.success = True
        response.message = json.dumps(self._metrics(), separators=(",", ":"))
        return response

    def _snapshot(self, _request, response):
        response.success = True
        response.message = json.dumps(self._metrics(), separators=(",", ":"))
        return response


def main():
    rclpy.init()
    node = ContactForceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
