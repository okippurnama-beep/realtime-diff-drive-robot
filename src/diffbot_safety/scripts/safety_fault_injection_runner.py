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

"""Inject controlled safety faults and persist command-level response data."""

import csv
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import time

from controller_manager_msgs.srv import ListControllers
from diffbot_interfaces.msg import SafetyStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


CSV_FIELDS = (
    'schema_version',
    'run_id',
    'repetition',
    'trial_index',
    'fault_name',
    'expected_fault_bit',
    'configured_timeout_ms',
    'acceptance_limit_ms',
    'injection_monotonic_ns',
    'fault_status_monotonic_ns',
    'zero_output_monotonic_ns',
    'fault_detection_latency_ms',
    'zero_command_latency_ms',
    'pre_fault_linear_x',
    'pre_fault_angular_z',
    'observed_state',
    'observed_active_faults',
    'observed_latched_faults',
    'observed_input_age_sec',
    'zero_output_observed',
    'reset_success',
    'reset_message',
    'trial_pass',
    'failure_reason',
)

SCENARIOS = {
    'estop': {
        'fault_bit': SafetyStatus.FAULT_MANUAL_ESTOP,
        'state': SafetyStatus.STATE_ESTOP_LATCHED,
        'timeout_ms': 0.0,
        'limit_ms': 40.0,
        'age_field': 'command_age_sec',
    },
    'scan_timeout': {
        'fault_bit': SafetyStatus.FAULT_SCAN_STALE,
        'state': SafetyStatus.STATE_FAULT_LATCHED,
        'timeout_ms': 500.0,
        'limit_ms': 540.0,
        'age_field': 'scan_age_sec',
    },
    'odom_timeout': {
        'fault_bit': SafetyStatus.FAULT_ODOM_STALE,
        'state': SafetyStatus.STATE_FAULT_LATCHED,
        'timeout_ms': 500.0,
        'limit_ms': 540.0,
        'age_field': 'odom_age_sec',
    },
    'nav2_timeout': {
        'fault_bit': SafetyStatus.FAULT_NAV2_INACTIVE,
        'state': SafetyStatus.STATE_FAULT_LATCHED,
        'timeout_ms': 1000.0,
        'limit_ms': 1040.0,
        'age_field': None,
    },
}

DEFAULT_SCENARIO_ORDER = tuple(SCENARIOS)

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


def quaternion_from_yaw(yaw):
    """Return the planar quaternion Z and W values for a yaw angle."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def command_is_zero(message, epsilon=1.0e-4):
    """Return whether every Twist component is effectively zero."""
    components = (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    )
    return all(math.isfinite(value) and abs(value) <= epsilon
               for value in components)


class SafetyFaultInjectionRunner(Node):
    """Relay live evidence, inject one fault, and measure safe output."""

    def __init__(self):
        super().__init__('safety_fault_injection_runner')

        self.declare_parameter('output_csv', '')
        self.declare_parameter('repetitions', 3)
        self.declare_parameter('scenarios_csv', '')
        self.declare_parameter('server_timeout_sec', 120.0)
        self.declare_parameter('motion_timeout_sec', 30.0)
        self.declare_parameter('observation_margin_sec', 2.0)
        self.declare_parameter('reset_zero_duration_sec', 1.20)
        self.declare_parameter('reset_zero_rate_hz', 20.0)
        self.declare_parameter('goal_x', 2.80)
        self.declare_parameter('goal_y', 0.00)
        self.declare_parameter('goal_yaw', 0.00)
        self.declare_parameter('dry_run', False)

        self._validate_parameters()
        self._scenarios = self._select_scenarios()
        self._run_id = datetime.now(timezone.utc).strftime(
            '%Y%m%dT%H%M%SZ'
        )
        self._output_path = self._resolve_output_path()

        self._scan_enabled = True
        self._odom_enabled = True
        self._scan_source_seen = False
        self._odom_source_seen = False
        self._latest_status = None
        self._latest_safe_command = None
        self._latest_safe_command_ns = None
        self._injection_ns = None
        self._fault_status_ns = None
        self._fault_status = None
        self._zero_output_ns = None
        self._expected_fault_bit = 0

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._scan_pub = self.create_publisher(
            LaserScan,
            '/safety_benchmark/scan',
            qos_profile_sensor_data,
        )
        self._odom_pub = self.create_publisher(
            Odometry,
            '/safety_benchmark/odometry',
            reliable_qos,
        )
        self._estop_pub = self.create_publisher(
            Bool,
            '/safety/estop',
            retained_qos,
        )
        self._scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_callback,
            reliable_qos,
        )
        self._status_sub = self.create_subscription(
            SafetyStatus,
            '/safety/status',
            self._status_callback,
            retained_qos,
        )
        self._safe_command_sub = self.create_subscription(
            Twist,
            '/cmd_vel_safe',
            self._safe_command_callback,
            reliable_qos,
        )

        self._localization_health_service = self.create_service(
            Trigger,
            '/safety_benchmark/localization/is_active',
            self._health_callback,
        )
        self._navigation_health_service = None
        self._restore_navigation_health_service()

        self._reset_client = self.create_client(
            Trigger,
            '/safety/reset',
        )
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        self._lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in REQUIRED_LIFECYCLE_NODES
        }
        self._controller_manager_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )

    def _validate_parameters(self):
        if self.get_parameter('repetitions').value < 1:
            raise ValueError('repetitions must be at least 1')
        for name in (
            'server_timeout_sec',
            'motion_timeout_sec',
            'observation_margin_sec',
            'reset_zero_duration_sec',
            'reset_zero_rate_hz',
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('goal_x', 'goal_y', 'goal_yaw'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')

    def _select_scenarios(self):
        configured = self.get_parameter('scenarios_csv').value.strip()
        if not configured:
            return list(DEFAULT_SCENARIO_ORDER)
        names = [name.strip() for name in configured.split(',')
                 if name.strip()]
        if not names:
            raise ValueError('scenarios_csv must contain a scenario')
        if len(names) != len(set(names)):
            raise ValueError('scenarios_csv must not contain duplicates')
        unknown = [name for name in names if name not in SCENARIOS]
        if unknown:
            raise ValueError(f'unknown safety scenarios: {unknown}')
        return names

    def _resolve_output_path(self):
        configured = self.get_parameter('output_csv').value
        if configured:
            return Path(configured).expanduser().resolve()
        return (
            Path.cwd()
            / 'benchmark_results'
            / f'safety_benchmark_{self._run_id}.csv'
        )

    def _scan_callback(self, message):
        self._scan_source_seen = True
        if self._scan_enabled:
            self._scan_pub.publish(message)

    def _odom_callback(self, message):
        self._odom_source_seen = True
        if self._odom_enabled:
            self._odom_pub.publish(message)

    def _status_callback(self, message):
        self._latest_status = message
        if (
            self._injection_ns is not None
            and self._fault_status_ns is None
            and message.active_faults & self._expected_fault_bit
        ):
            self._fault_status_ns = time.monotonic_ns()
            self._fault_status = message

    def _safe_command_callback(self, message):
        now_ns = time.monotonic_ns()
        self._latest_safe_command = message
        self._latest_safe_command_ns = now_ns
        if (
            self._injection_ns is not None
            and now_ns >= self._injection_ns
            and self._zero_output_ns is None
            and command_is_zero(message)
        ):
            self._zero_output_ns = now_ns

    @staticmethod
    def _health_callback(request, response):
        del request
        response.success = True
        response.message = 'fault-injection proxy active'
        return response

    def _restore_navigation_health_service(self):
        if self._navigation_health_service is None:
            self._navigation_health_service = self.create_service(
                Trigger,
                '/safety_benchmark/navigation/is_active',
                self._health_callback,
            )

    def _remove_navigation_health_service(self):
        if self._navigation_health_service is not None:
            self.destroy_service(self._navigation_health_service)
            self._navigation_health_service = None

    def _wait_for(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.01)
        return bool(predicate())

    def _wait_for_future(self, future, timeout_sec):
        return self._wait_for(lambda: future.done(), timeout_sec)

    def _wait_for_active_nodes(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        pending = set(REQUIRED_LIFECYCLE_NODES)
        while rclpy.ok() and pending and time.monotonic() < deadline:
            for name in tuple(pending):
                client = self._lifecycle_clients[name]
                if not client.wait_for_service(timeout_sec=0.01):
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue
                future = client.call_async(GetState.Request())
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                if not self._wait_for_future(future, min(1.0, remaining)):
                    continue
                if future.result().current_state.id == (
                    State.PRIMARY_STATE_ACTIVE
                ):
                    pending.remove(name)
                    self.get_logger().info(f'Lifecycle ready: {name}')
        if pending:
            self.get_logger().error(
                'Timed out waiting for lifecycle nodes: '
                + ', '.join(sorted(pending))
            )
            return False
        return True

    def _wait_for_active_controllers(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        pending = set(REQUIRED_CONTROLLERS)
        while rclpy.ok() and pending and time.monotonic() < deadline:
            if not self._controller_manager_client.wait_for_service(
                timeout_sec=0.05
            ):
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            future = self._controller_manager_client.call_async(
                ListControllers.Request()
            )
            if not self._wait_for_future(future, 1.0):
                continue
            active = {
                controller.name
                for controller in future.result().controller
                if controller.state == 'active'
            }
            pending.difference_update(active)
        if pending:
            self.get_logger().error(
                'Timed out waiting for controllers: '
                + ', '.join(sorted(pending))
            )
            return False
        return True

    def _status_is_clean_ready(self):
        return (
            self._latest_status is not None
            and self._latest_status.state == SafetyStatus.STATE_READY
            and self._latest_status.active_faults == SafetyStatus.FAULT_NONE
            and self._latest_status.latched_faults == SafetyStatus.FAULT_NONE
        )

    def _wait_for_stack(self):
        timeout = float(self.get_parameter('server_timeout_sec').value)
        if not self._wait_for(
            lambda: self._action_client.server_is_ready(),
            timeout,
        ):
            self.get_logger().error('NavigateToPose server is unavailable.')
            return False
        if not self._wait_for_active_nodes(timeout):
            return False
        if not self._wait_for_active_controllers(timeout):
            return False
        if not self._reset_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('/safety/reset is unavailable.')
            return False
        if not self._wait_for(
            lambda: (
                self._scan_source_seen
                and self._odom_source_seen
                and self._status_is_clean_ready()
            ),
            timeout,
        ):
            self.get_logger().error(
                'Safety relays did not reach a clean READY state.'
            )
            return False
        return True

    def _goal_message(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(
            self.get_parameter('goal_x').value
        )
        goal.pose.pose.position.y = float(
            self.get_parameter('goal_y').value
        )
        quaternion_z, quaternion_w = quaternion_from_yaw(
            float(self.get_parameter('goal_yaw').value)
        )
        goal.pose.pose.orientation.z = quaternion_z
        goal.pose.pose.orientation.w = quaternion_w
        return goal

    def _start_motion(self):
        send_future = self._action_client.send_goal_async(
            self._goal_message()
        )
        if not self._wait_for_future(send_future, 10.0):
            raise RuntimeError('timed out sending navigation goal')
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('navigation goal was rejected')

        timeout = float(self.get_parameter('motion_timeout_sec').value)
        nonzero_seen = self._wait_for(
            lambda: (
                self._latest_safe_command is not None
                and self._latest_safe_command_ns is not None
                and not command_is_zero(self._latest_safe_command)
                and time.monotonic_ns() - self._latest_safe_command_ns
                <= 200_000_000
            ),
            timeout,
        )
        if not nonzero_seen:
            self._cancel_goal(goal_handle)
            raise RuntimeError('no recent nonzero safe command was observed')
        return goal_handle, (
            float(self._latest_safe_command.linear.x),
            float(self._latest_safe_command.angular.z),
        )

    def _cancel_goal(self, goal_handle):
        if goal_handle is None:
            return False
        cancel_future = goal_handle.cancel_goal_async()
        if not self._wait_for_future(cancel_future, 5.0):
            return False
        return bool(cancel_future.result().goals_canceling)

    def _begin_observation(self, scenario):
        self._expected_fault_bit = int(scenario['fault_bit'])
        self._fault_status_ns = None
        self._fault_status = None
        self._zero_output_ns = None
        self._injection_ns = time.monotonic_ns()

    def _inject(self, fault_name):
        scenario = SCENARIOS[fault_name]
        if fault_name == 'estop':
            self._begin_observation(scenario)
            self._estop_pub.publish(Bool(data=True))
        elif fault_name == 'scan_timeout':
            self._scan_enabled = False
            self._begin_observation(scenario)
        elif fault_name == 'odom_timeout':
            self._odom_enabled = False
            self._begin_observation(scenario)
        elif fault_name == 'nav2_timeout':
            self._remove_navigation_health_service()
            self._begin_observation(scenario)
        else:
            raise ValueError(f'unsupported fault scenario: {fault_name}')

    def _restore_fault_source(self, fault_name):
        if fault_name == 'estop':
            self._estop_pub.publish(Bool(data=False))
        elif fault_name == 'scan_timeout':
            self._scan_enabled = True
        elif fault_name == 'odom_timeout':
            self._odom_enabled = True
        elif fault_name == 'nav2_timeout':
            self._restore_navigation_health_service()

    def _publish_reset_zero(self):
        publisher = self.create_publisher(
            Twist,
            '/cmd_vel_collision_checked',
            10,
        )
        duration = float(
            self.get_parameter('reset_zero_duration_sec').value
        )
        frequency = float(self.get_parameter('reset_zero_rate_hz').value)
        period = 1.0 / frequency
        deadline = time.monotonic() + duration
        zero = Twist()
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                publisher.publish(zero)
                rclpy.spin_once(self, timeout_sec=period)
            publisher.publish(zero)
            future = self._reset_client.call_async(Trigger.Request())
            if not self._wait_for_future(future, 3.0):
                return False, 'reset service timed out'
            response = future.result()
            return bool(response.success), response.message
        finally:
            self.destroy_publisher(publisher)

    def _recover(self, fault_name, goal_handle):
        cancel_ok = self._cancel_goal(goal_handle)
        self._restore_fault_source(fault_name)
        reset_success, reset_message = self._publish_reset_zero()
        active_cleared = (
            self._latest_status is not None
            and self._latest_status.active_faults
            == SafetyStatus.FAULT_NONE
        )
        ready = reset_success and self._wait_for(
            self._status_is_clean_ready,
            2.0,
        )
        self._injection_ns = None
        return (
            cancel_ok,
            active_cleared,
            reset_success and ready,
            reset_message,
        )

    def _observed_input_age(self, scenario):
        if self._fault_status is None or scenario['age_field'] is None:
            return -1.0
        return float(getattr(self._fault_status, scenario['age_field']))

    def _run_trial(self, repetition, trial_index, fault_name):
        scenario = SCENARIOS[fault_name]
        failure_reasons = []
        goal_handle = None
        pre_linear = math.nan
        pre_angular = math.nan

        try:
            goal_handle, command = self._start_motion()
            pre_linear, pre_angular = command
            self._inject(fault_name)
            observation_timeout = (
                scenario['limit_ms'] / 1000.0
                + float(
                    self.get_parameter('observation_margin_sec').value
                )
            )
            observed = self._wait_for(
                lambda: (
                    self._fault_status_ns is not None
                    and self._zero_output_ns is not None
                ),
                observation_timeout,
            )
            if not observed:
                failure_reasons.append('observation timeout')
        except RuntimeError as error:
            failure_reasons.append(str(error))

        injection_ns = self._injection_ns
        fault_ns = self._fault_status_ns
        zero_ns = self._zero_output_ns
        fault_latency_ms = (
            (fault_ns - injection_ns) / 1.0e6
            if injection_ns is not None and fault_ns is not None
            else math.nan
        )
        zero_latency_ms = (
            (zero_ns - injection_ns) / 1.0e6
            if injection_ns is not None and zero_ns is not None
            else math.nan
        )

        cancel_ok, active_cleared, reset_success, reset_message = (
            self._recover(fault_name, goal_handle)
            if injection_ns is not None
            else (self._cancel_goal(goal_handle), True, False,
                  'fault was not injected')
        )
        if not cancel_ok:
            failure_reasons.append('goal cancellation failed')
        if not active_cleared:
            failure_reasons.append('active faults did not clear')
        if not reset_success:
            failure_reasons.append(f'reset failed: {reset_message}')

        status = self._fault_status
        if status is None:
            failure_reasons.append('expected fault status not observed')
            observed_state = -1
            active_faults = 0
            latched_faults = 0
        else:
            observed_state = int(status.state)
            active_faults = int(status.active_faults)
            latched_faults = int(status.latched_faults)
            if observed_state != scenario['state']:
                failure_reasons.append('unexpected safety state')
            if not active_faults & scenario['fault_bit']:
                failure_reasons.append('expected active fault bit missing')
            if not latched_faults & scenario['fault_bit']:
                failure_reasons.append('expected latched fault bit missing')
        if not math.isfinite(zero_latency_ms):
            failure_reasons.append('zero output not observed')
        elif zero_latency_ms > scenario['limit_ms']:
            failure_reasons.append('zero-command latency exceeded limit')
        if not math.isfinite(pre_linear) or not math.isfinite(pre_angular):
            failure_reasons.append('invalid pre-fault command')
        elif abs(pre_linear) <= 1.0e-4 and abs(pre_angular) <= 1.0e-4:
            failure_reasons.append('pre-fault command was zero')

        failure_reason = '; '.join(dict.fromkeys(failure_reasons))
        return {
            'schema_version': 1,
            'run_id': self._run_id,
            'repetition': repetition,
            'trial_index': trial_index,
            'fault_name': fault_name,
            'expected_fault_bit': scenario['fault_bit'],
            'configured_timeout_ms': scenario['timeout_ms'],
            'acceptance_limit_ms': scenario['limit_ms'],
            'injection_monotonic_ns': injection_ns or 0,
            'fault_status_monotonic_ns': fault_ns or 0,
            'zero_output_monotonic_ns': zero_ns or 0,
            'fault_detection_latency_ms': fault_latency_ms,
            'zero_command_latency_ms': zero_latency_ms,
            'pre_fault_linear_x': pre_linear,
            'pre_fault_angular_z': pre_angular,
            'observed_state': observed_state,
            'observed_active_faults': active_faults,
            'observed_latched_faults': latched_faults,
            'observed_input_age_sec': self._observed_input_age(scenario),
            'zero_output_observed': zero_ns is not None,
            'reset_success': reset_success,
            'reset_message': reset_message,
            'trial_pass': not failure_reason,
            'failure_reason': failure_reason,
        }

    def _write_row(self, stream, writer, row):
        writer.writerow(row)
        stream.flush()
        os.fsync(stream.fileno())
        self.get_logger().info(
            f"Saved {row['fault_name']} trial {row['trial_index']}: "
            f"zero={row['zero_command_latency_ms']:.3f} ms, "
            f"pass={row['trial_pass']}"
        )

    def run(self):
        """Execute the configured safety benchmark and return an exit code."""
        scenario_text = ', '.join(self._scenarios)
        repetitions = int(self.get_parameter('repetitions').value)
        self.get_logger().info(
            f'Safety scenarios: {scenario_text}; repetitions: {repetitions}'
        )
        if not self._wait_for_stack():
            return 2
        if self.get_parameter('dry_run').value:
            self.get_logger().info(
                'Safety benchmark dry run passed; no motion was commanded.'
            )
            return 0

        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Writing CSV results to {self._output_path}')
        all_passed = True
        trial_index = 0
        with self._output_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=CSV_FIELDS,
                lineterminator='\n',
            )
            writer.writeheader()
            stream.flush()
            os.fsync(stream.fileno())

            for repetition in range(1, repetitions + 1):
                for fault_name in self._scenarios:
                    trial_index += 1
                    self.get_logger().info(
                        f'Beginning repetition {repetition}, '
                        f'scenario {fault_name}.'
                    )
                    row = self._run_trial(
                        repetition,
                        trial_index,
                        fault_name,
                    )
                    self._write_row(stream, writer, row)
                    all_passed = all_passed and row['trial_pass']
                    if not row['reset_success']:
                        self.get_logger().error(
                            'Guarded recovery failed; stopping before the '
                            'next motion trial.'
                        )
                        return 1
        return 0 if all_passed else 1


def main(args=None):
    """Initialize ROS, execute the benchmark, and exit with its status."""
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = SafetyFaultInjectionRunner()
        exit_code = node.run()
    except (OSError, RuntimeError, ValueError) as error:
        if node is None:
            print(f'safety benchmark configuration error: {error}')
        else:
            node.get_logger().error(str(error))
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
