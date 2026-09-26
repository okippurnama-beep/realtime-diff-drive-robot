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

"""Run repeatable Nav2 goals and write one durable CSV row per trial."""

import csv
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


CSV_FIELDS = (
    'schema_version',
    'run_id',
    'repetition',
    'trial_index',
    'goal_name',
    'target_x_m',
    'target_y_m',
    'target_yaw_rad',
    'start_x_m',
    'start_y_m',
    'start_yaw_rad',
    'finish_x_m',
    'finish_y_m',
    'finish_yaw_rad',
    'position_error_m',
    'yaw_error_rad',
    'action_status',
    'status_code',
    'nav2_error_code',
    'nav2_error_msg',
    'navigation_time_s',
    'wall_time_s',
    'max_recoveries',
    'final_distance_remaining_m',
    'timed_out',
    'return_status',
)

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}

REQUIRED_LIFECYCLE_NODES = (
    'map_server',
    'amcl',
    'controller_server',
    'smoother_server',
    'planner_server',
    'route_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
    'waypoint_follower',
    'docking_server',
)

REQUIRED_CONTROLLERS = (
    'joint_state_broadcaster',
    'diff_drive_base_controller',
)


def duration_seconds(duration):
    """Convert a ROS duration message to fractional seconds."""
    return float(duration.sec) + float(duration.nanosec) / 1.0e9


def normalize_angle(angle):
    """Normalize an angle to the closed-open interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_from_yaw(yaw):
    """Return the planar quaternion Z and W values for a yaw angle."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_from_quaternion(quaternion):
    """Extract planar yaw from a quaternion."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class NavBenchmarkRunner(Node):
    """Execute configured NavigateToPose trials and persist their metrics."""

    def __init__(self):
        super().__init__('nav_benchmark_runner')

        share_dir = Path(get_package_share_directory('diffbot_navigation'))
        default_layout = share_dir / 'config' / 'nav_benchmark_layout.yaml'

        self.declare_parameter('layout_file', str(default_layout))
        self.declare_parameter('output_csv', '')
        self.declare_parameter('goal_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('goal_names_csv', '')
        self.declare_parameter('repetitions', 1)
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('server_timeout_sec', 60.0)
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('transform_timeout_sec', 10.0)
        self.declare_parameter('settle_time_sec', 1.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('dry_run_wait_for_server', False)

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in REQUIRED_LIFECYCLE_NODES
        }
        self._controller_manager_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
        self._latest_feedback = None
        self._max_recoveries = 0

        self._run_id = datetime.now(timezone.utc).strftime(
            '%Y%m%dT%H%M%SZ'
        )
        self._layout = self._load_layout()
        self._goals = self._select_goals()
        self._output_path = self._resolve_output_path()
        self._validate_parameters()

    def _load_layout(self):
        layout_path = Path(
            self.get_parameter('layout_file').get_parameter_value().string_value
        ).expanduser()
        with layout_path.open(encoding='utf-8') as stream:
            layout = yaml.safe_load(stream)

        if layout.get('version') != 1:
            raise ValueError('layout version must be 1')
        if 'robot' not in layout or 'goals' not in layout:
            raise ValueError('layout must define robot and goals sections')
        return layout

    def _select_goals(self):
        configured_goals = {
            goal['name']: goal for goal in self._layout['goals']
        }
        requested_parameter = self.get_parameter_or(
            'goal_names',
            Parameter(
                'goal_names',
                Parameter.Type.STRING_ARRAY,
                [],
            ),
        )
        requested_names = list(requested_parameter.value)
        requested_csv = self.get_parameter('goal_names_csv').value.strip()
        if requested_names and requested_csv:
            raise ValueError(
                'set only one of goal_names or goal_names_csv'
            )
        if requested_csv:
            requested_names = [
                name.strip() for name in requested_csv.split(',')
                if name.strip()
            ]
        if not requested_names:
            return list(self._layout['goals'])

        if len(requested_names) != len(set(requested_names)):
            raise ValueError('benchmark goal names must not be duplicated')

        unknown = [
            name for name in requested_names if name not in configured_goals
        ]
        if unknown:
            raise ValueError(f'unknown benchmark goal names: {unknown}')
        return [configured_goals[name] for name in requested_names]

    def _resolve_output_path(self):
        configured_path = (
            self.get_parameter('output_csv')
            .get_parameter_value()
            .string_value
        )
        if configured_path:
            return Path(configured_path).expanduser().resolve()
        return (
            Path.cwd()
            / 'benchmark_results'
            / f'nav_benchmark_{self._run_id}.csv'
        )

    def _validate_parameters(self):
        repetitions = self.get_parameter('repetitions').value
        if repetitions < 1:
            raise ValueError('repetitions must be at least 1')

        positive_parameters = (
            'server_timeout_sec',
            'goal_timeout_sec',
            'transform_timeout_sec',
        )
        for name in positive_parameters:
            if self.get_parameter(name).value <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.get_parameter('settle_time_sec').value < 0.0:
            raise ValueError('settle_time_sec must not be negative')

    def _goal_message(self, pose):
        x, y, yaw = pose
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        quaternion_z, quaternion_w = quaternion_from_yaw(float(yaw))
        goal.pose.pose.orientation.z = quaternion_z
        goal.pose.pose.orientation.w = quaternion_w
        return goal

    def _feedback_callback(self, feedback_message):
        self._latest_feedback = feedback_message.feedback
        self._max_recoveries = max(
            self._max_recoveries,
            int(feedback_message.feedback.number_of_recoveries),
        )

    def _lookup_pose(self):
        timeout = float(self.get_parameter('transform_timeout_sec').value)
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    Time(),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                return (
                    float(translation.x),
                    float(translation.y),
                    yaw_from_quaternion(rotation),
                )
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError('timed out waiting for map -> base_footprint TF')

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return future.done()

    def _wait_for_active_nodes(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        pending = set(REQUIRED_LIFECYCLE_NODES)

        while rclpy.ok() and pending and time.monotonic() < deadline:
            for name in tuple(pending):
                client = self._lifecycle_clients[name]
                if not client.wait_for_service(timeout_sec=0.1):
                    continue

                future = client.call_async(GetState.Request())
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                if not self._wait_for_future(future, min(1.0, remaining)):
                    continue
                response = future.result()
                if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                    pending.remove(name)
                    self.get_logger().info(f'Lifecycle ready: {name}')

        if pending:
            names = ', '.join(sorted(pending))
            self.get_logger().error(
                f'Timed out waiting for active lifecycle nodes: {names}'
            )
            return False
        return True

    def _wait_for_active_controllers(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        pending = set(REQUIRED_CONTROLLERS)

        while rclpy.ok() and pending and time.monotonic() < deadline:
            if not self._controller_manager_client.wait_for_service(
                timeout_sec=0.2
            ):
                continue

            future = self._controller_manager_client.call_async(
                ListControllers.Request()
            )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            if not self._wait_for_future(future, min(1.0, remaining)):
                continue

            response = future.result()
            active = {
                controller.name
                for controller in response.controller
                if controller.state == 'active'
            }
            newly_ready = pending.intersection(active)
            for name in sorted(newly_ready):
                self.get_logger().info(f'Controller ready: {name}')
            pending.difference_update(active)

        if pending:
            names = ', '.join(sorted(pending))
            self.get_logger().error(
                f'Timed out waiting for active controllers: {names}'
            )
            return False
        return True

    def _execute_goal(self, name, pose):
        self._latest_feedback = None
        self._max_recoveries = 0
        timeout = float(self.get_parameter('goal_timeout_sec').value)
        started_at = time.monotonic()

        send_future = self._action_client.send_goal_async(
            self._goal_message(pose),
            feedback_callback=self._feedback_callback,
        )
        if not self._wait_for_future(send_future, 10.0):
            return self._failed_result(
                'SEND_TIMEOUT',
                'timed out while sending the goal',
                started_at,
            )

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return self._failed_result(
                'REJECTED',
                'Nav2 rejected the goal',
                started_at,
            )

        self.get_logger().info(
            f"Goal '{name}' accepted: x={pose[0]:.2f}, "
            f'y={pose[1]:.2f}, yaw={pose[2]:.4f}'
        )
        result_future = goal_handle.get_result_async()
        completed = self._wait_for_future(result_future, timeout)
        timed_out = not completed

        if timed_out:
            self.get_logger().error(
                f"Goal '{name}' exceeded {timeout:.1f} seconds; canceling."
            )
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, 5.0)
            self._wait_for_future(result_future, 5.0)

        if result_future.done():
            wrapped_result = result_future.result()
            status_code = int(wrapped_result.status)
            nav2_result = wrapped_result.result
            error_code = int(nav2_result.error_code)
            error_msg = nav2_result.error_msg
        else:
            status_code = GoalStatus.STATUS_UNKNOWN
            error_code = -1
            error_msg = 'result unavailable after cancellation'

        feedback = self._latest_feedback
        return {
            'status_code': status_code,
            'action_status': STATUS_NAMES.get(status_code, 'UNKNOWN'),
            'nav2_error_code': error_code,
            'nav2_error_msg': error_msg,
            'navigation_time_s': (
                duration_seconds(feedback.navigation_time)
                if feedback is not None else math.nan
            ),
            'wall_time_s': time.monotonic() - started_at,
            'max_recoveries': self._max_recoveries,
            'final_distance_remaining_m': (
                float(feedback.distance_remaining)
                if feedback is not None else math.nan
            ),
            'timed_out': timed_out,
        }

    def _failed_result(self, status, message, started_at):
        return {
            'status_code': -1,
            'action_status': status,
            'nav2_error_code': -1,
            'nav2_error_msg': message,
            'navigation_time_s': math.nan,
            'wall_time_s': time.monotonic() - started_at,
            'max_recoveries': 0,
            'final_distance_remaining_m': math.nan,
            'timed_out': status == 'SEND_TIMEOUT',
        }

    def _settle(self):
        duration = float(self.get_parameter('settle_time_sec').value)
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

    @staticmethod
    def _pose_metrics(target, pose):
        x, y, yaw = pose
        target_x, target_y, target_yaw = target
        return {
            'finish_x_m': x,
            'finish_y_m': y,
            'finish_yaw_rad': yaw,
            'position_error_m': math.hypot(
                target_x - x,
                target_y - y,
            ),
            'yaw_error_rad': abs(normalize_angle(target_yaw - yaw)),
        }

    def _run_trial(self, repetition, trial_index, goal):
        start_pose = self._lookup_pose()
        result = self._execute_goal(goal['name'], goal['pose'])
        self._settle()
        finish_pose = self._lookup_pose()

        row = {
            'schema_version': 1,
            'run_id': self._run_id,
            'repetition': repetition,
            'trial_index': trial_index,
            'goal_name': goal['name'],
            'target_x_m': goal['pose'][0],
            'target_y_m': goal['pose'][1],
            'target_yaw_rad': goal['pose'][2],
            'start_x_m': start_pose[0],
            'start_y_m': start_pose[1],
            'start_yaw_rad': start_pose[2],
            **self._pose_metrics(goal['pose'], finish_pose),
            **result,
            'return_status': 'DISABLED',
        }

        if self.get_parameter('return_to_start').value:
            return_result = self._execute_goal(
                'return_to_start',
                self._layout['robot']['start'],
            )
            row['return_status'] = return_result['action_status']
            self._settle()

        return row

    def _write_row(self, stream, writer, row):
        writer.writerow(row)
        stream.flush()
        os.fsync(stream.fileno())
        self.get_logger().info(
            f"Saved trial {row['trial_index']} ({row['goal_name']}): "
            f"{row['action_status']}, error={row['position_error_m']:.3f} m"
        )

    def run(self):
        """Run the configured benchmark and return a process exit code."""
        goal_names = ', '.join(goal['name'] for goal in self._goals)
        repetitions = int(self.get_parameter('repetitions').value)
        self.get_logger().info(
            f'Benchmark goals: {goal_names}; repetitions: {repetitions}'
        )

        dry_run = self.get_parameter('dry_run').value
        wait_in_dry_run = self.get_parameter(
            'dry_run_wait_for_server'
        ).value
        if dry_run and not wait_in_dry_run:
            self.get_logger().info('Dry run complete; no goals were sent.')
            return 0

        server_timeout = float(
            self.get_parameter('server_timeout_sec').value
        )
        if not self._action_client.wait_for_server(server_timeout):
            self.get_logger().error(
                'NavigateToPose action server did not become available.'
            )
            return 2

        if not self._wait_for_active_nodes(server_timeout):
            return 2
        if not self._wait_for_active_controllers(server_timeout):
            return 2

        if dry_run:
            self._lookup_pose()
            self.get_logger().info(
                'Dry run health check complete; action server and TF are ready.'
            )
            return 0

        self._lookup_pose()
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Writing CSV results to {self._output_path}')

        overall_success = True
        trial_index = 0
        with self._output_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            stream.flush()
            os.fsync(stream.fileno())

            for repetition in range(1, repetitions + 1):
                for goal in self._goals:
                    trial_index += 1
                    row = self._run_trial(repetition, trial_index, goal)
                    self._write_row(stream, writer, row)
                    trial_success = (
                        row['action_status'] == 'SUCCEEDED'
                        and row['return_status']
                        in ('SUCCEEDED', 'DISABLED')
                    )
                    overall_success = overall_success and trial_success
                    if (
                        self.get_parameter('return_to_start').value
                        and row['return_status'] != 'SUCCEEDED'
                    ):
                        self.get_logger().error(
                            'Return-to-start failed; stopping to preserve '
                            'benchmark comparability.'
                        )
                        return 1

        return 0 if overall_success else 1


def main(args=None):
    """Initialize ROS, run the benchmark, and exit with its status."""
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = NavBenchmarkRunner()
        exit_code = node.run()
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'benchmark configuration error: {error}')
        exit_code = 2
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = 130
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
