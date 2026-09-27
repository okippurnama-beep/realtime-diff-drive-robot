#!/usr/bin/env python3
# Copyright 2026 xiayuru
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify the live M7.2 velocity graph has no safety bypass."""

import argparse
from dataclasses import dataclass
import sys
import time

import rclpy
from rclpy.node import Node


@dataclass(frozen=True)
class TopicContract:
    """Expected ownership for one point-to-point velocity topic."""

    topic: str
    message_type: str
    publisher: str
    subscriber: str


CONTRACTS = (
    TopicContract(
        '/cmd_vel_smoothed',
        'geometry_msgs/msg/Twist',
        'velocity_smoother',
        'collision_monitor',
    ),
    TopicContract(
        '/cmd_vel_collision_checked',
        'geometry_msgs/msg/Twist',
        'collision_monitor',
        'safety_supervisor',
    ),
    TopicContract(
        '/cmd_vel_safe',
        'geometry_msgs/msg/Twist',
        'safety_supervisor',
        'twist_to_twist_stamped_node',
    ),
    TopicContract(
        '/diff_drive_base_controller/cmd_vel',
        'geometry_msgs/msg/TwistStamped',
        'twist_to_twist_stamped_node',
        'diff_drive_base_controller',
    ),
)


def endpoint_names(endpoints):
    """Return fully qualified endpoint node names in stable order."""
    names = []
    for endpoint in endpoints:
        namespace = endpoint.node_namespace.rstrip('/')
        names.append(f'{namespace}/{endpoint.node_name}' or '/')
    return sorted(names)


def expected_node_name(node_name):
    """Return the root-namespace name used by the current launch files."""
    return f'/{node_name}'


def evaluate_contracts(node):
    """Return current graph errors and a human-readable snapshot."""
    errors = []
    snapshots = []

    for contract in CONTRACTS:
        publishers = node.get_publishers_info_by_topic(contract.topic)
        subscribers = node.get_subscriptions_info_by_topic(contract.topic)
        publisher_names = endpoint_names(publishers)
        subscriber_names = endpoint_names(subscribers)
        snapshots.append(
            (
                contract.topic,
                publisher_names,
                subscriber_names,
            )
        )

        if publisher_names != [expected_node_name(contract.publisher)]:
            errors.append(
                f'{contract.topic}: publishers={publisher_names}, expected '
                f"['/{contract.publisher}']"
            )
        if subscriber_names != [expected_node_name(contract.subscriber)]:
            errors.append(
                f'{contract.topic}: subscribers={subscriber_names}, expected '
                f"['/{contract.subscriber}']"
            )

        endpoint_types = {
            endpoint.topic_type
            for endpoint in publishers + subscribers
        }
        if endpoint_types and endpoint_types != {contract.message_type}:
            errors.append(
                f'{contract.topic}: types={sorted(endpoint_types)}, expected '
                f"['{contract.message_type}']"
            )

    return errors, snapshots


def parse_arguments():
    """Parse verifier-specific arguments and leave ROS arguments untouched."""
    parser = argparse.ArgumentParser(
        description='Verify the live M7.2 velocity command chain.',
    )
    parser.add_argument(
        '--timeout-sec',
        type=float,
        default=15.0,
        help='Maximum wall-clock time to wait for graph discovery.',
    )
    parser.add_argument(
        '--stable-sec',
        type=float,
        default=1.0,
        help='How long the complete graph must remain valid.',
    )
    arguments, _ = parser.parse_known_args()
    if arguments.timeout_sec <= 0.0:
        parser.error('--timeout-sec must be positive')
    if arguments.stable_sec < 0.0:
        parser.error('--stable-sec cannot be negative')
    return arguments


def main():
    """Wait for a stable graph, print evidence, and return a gate result."""
    arguments = parse_arguments()
    rclpy.init(args=sys.argv)
    node = Node('verify_runtime_command_chain')
    deadline = time.monotonic() + arguments.timeout_sec
    stable_since = None
    last_errors = []
    last_snapshot = []

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            last_errors, last_snapshot = evaluate_contracts(node)
            now = time.monotonic()
            if last_errors:
                stable_since = None
                continue
            if stable_since is None:
                stable_since = now
            if now - stable_since >= arguments.stable_sec:
                for topic, publishers, subscribers in last_snapshot:
                    print(
                        f'PASS {topic}: publisher={publishers[0]} '
                        f'subscriber={subscribers[0]}'
                    )
                print('Runtime command-chain validation: PASS')
                return 0

        print('Runtime command-chain validation: FAIL', file=sys.stderr)
        for error in last_errors:
            print(f'- {error}', file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
