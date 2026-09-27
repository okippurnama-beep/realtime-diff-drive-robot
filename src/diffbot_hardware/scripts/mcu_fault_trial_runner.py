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

"""Run one fresh-stack virtual-MCU transport fault trial."""

import csv
import math
from pathlib import Path
import time

from controller_manager_msgs.srv import ListControllers
from diffbot_interfaces.msg import SafetyStatus
from diffbot_interfaces.srv import McuFaultControl
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


FIELDS = (
    'schema_version', 'run_id', 'repetition', 'trial_index',
    'fault_name', 'configured_heartbeat_timeout_ms', 'command_delay_ms',
    'acceptance_limit_ms', 'injection_monotonic_ns',
    'fault_status_monotonic_ns', 'zero_output_monotonic_ns',
    'fault_detection_latency_ms', 'zero_command_latency_ms',
    'pre_fault_linear_x', 'observed_state', 'observed_active_faults',
    'observed_latched_faults', 'trial_pass', 'failure_reason',
)

SCENARIOS = {
    'drop_commands': McuFaultControl.Request.SCENARIO_DROP_COMMANDS,
    'drop_states': McuFaultControl.Request.SCENARIO_DROP_STATES,
    'command_delay': McuFaultControl.Request.SCENARIO_COMMAND_DELAY,
    'reboot': McuFaultControl.Request.SCENARIO_REBOOT,
}


def is_zero(message, epsilon=1.0e-4):
    """Return whether both planar command components are effectively zero."""
    return (
        math.isfinite(message.linear.x)
        and math.isfinite(message.angular.z)
        and abs(message.linear.x) <= epsilon
        and abs(message.angular.z) <= epsilon
    )


class McuFaultTrialRunner(Node):
    """Drive the safe command path and record one transport failure."""

    def __init__(self):
        super().__init__('mcu_fault_trial_runner')
        self.declare_parameter('scenario', 'drop_states')
        self.declare_parameter('repetition', 1)
        self.declare_parameter('trial_index', 1)
        self.declare_parameter('run_id', '')
        self.declare_parameter('output_csv', '')
        self.declare_parameter('server_timeout_sec', 20.0)
        self.declare_parameter('observation_timeout_sec', 2.0)
        self.declare_parameter('command_delay_ms', 150)
        self.declare_parameter('acceptance_limit_ms', 260.0)

        self.scenario = self.get_parameter('scenario').value
        if self.scenario not in SCENARIOS:
            raise ValueError(f'unknown scenario: {self.scenario}')
        output_value = str(self.get_parameter('output_csv').value).strip()
        if not output_value:
            raise ValueError('output_csv cannot be empty')
        self.output_csv = Path(output_value).expanduser().resolve()
        if not str(self.get_parameter('run_id').value).strip():
            raise ValueError('run_id cannot be empty')

        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_pub = self.create_publisher(Twist, '/cmd_vel_m8', 10)
        self.safe_sub = self.create_subscription(
            Twist, '/cmd_vel_safe', self._safe_callback, 10
        )
        self.status_sub = self.create_subscription(
            SafetyStatus, '/safety/status', self._status_callback, retained_qos
        )
        self.fault_client = self.create_client(
            McuFaultControl, '/mcu/fault_control'
        )
        self.controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        self.command = Twist()
        self.command_timer = self.create_timer(0.05, self._publish_command)
        self.latest_safe = None
        self.latest_status = None
        self.injection_ns = None
        self.fault_status_ns = None
        self.fault_status = None
        self.zero_output_ns = None

    def _publish_command(self):
        self.command_pub.publish(self.command)

    def _safe_callback(self, message):
        self.latest_safe = message
        now_ns = time.monotonic_ns()
        if (
            self.injection_ns is not None
            and now_ns >= self.injection_ns
            and self.zero_output_ns is None
            and is_zero(message)
        ):
            self.zero_output_ns = now_ns

    def _status_callback(self, message):
        self.latest_status = message
        now_ns = time.monotonic_ns()
        if (
            self.injection_ns is not None
            and self.fault_status_ns is None
            and message.active_faults
            & SafetyStatus.FAULT_MCU_HEARTBEAT_LOST
        ):
            self.fault_status_ns = now_ns
            self.fault_status = message

    def _wait_for(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if predicate():
                return True
        return bool(predicate())

    def _controllers_active(self):
        if not self.controllers_client.service_is_ready():
            return False
        future = self.controllers_client.call_async(ListControllers.Request())
        if not self._wait_for(lambda: future.done(), 1.0):
            return False
        active = {
            item.name for item in future.result().controller
            if item.state == 'active'
        }
        return {
            'joint_state_broadcaster',
            'diff_drive_base_controller',
        }.issubset(active)

    def _stack_ready(self):
        timeout = float(self.get_parameter('server_timeout_sec').value)
        if not self.fault_client.wait_for_service(timeout_sec=timeout):
            return False, 'fault-control service unavailable'
        if not self.controllers_client.wait_for_service(timeout_sec=timeout):
            return False, 'controller-manager service unavailable'
        if not self._wait_for(self._controllers_active, timeout):
            return False, 'controllers did not become active'
        ready = self._wait_for(
            lambda: (
                self.latest_status is not None
                and self.latest_status.state == SafetyStatus.STATE_READY
                and self.latest_status.active_faults == SafetyStatus.FAULT_NONE
            ),
            timeout,
        )
        return (True, '') if ready else (False, 'safety did not reach READY')

    def _wait_for_motion(self):
        self.command.linear.x = 0.12
        return self._wait_for(
            lambda: (
                self.latest_safe is not None
                and not is_zero(self.latest_safe)
                and abs(self.latest_safe.linear.x - 0.12) < 1.0e-6
            ),
            2.0,
        )

    def _inject(self):
        request = McuFaultControl.Request()
        request.scenario = SCENARIOS[self.scenario]
        request.enabled = True
        request.delay_ms = (
            int(self.get_parameter('command_delay_ms').value)
            if self.scenario == 'command_delay' else 0
        )
        self.injection_ns = time.monotonic_ns()
        future = self.fault_client.call_async(request)
        if not self._wait_for(lambda: future.done(), 1.0):
            return False, 'fault-control request timed out'
        response = future.result()
        return bool(response.success), response.message

    def _record(self, failure_reason=''):
        limit_ms = float(self.get_parameter('acceptance_limit_ms').value)
        fault_latency = (
            (self.fault_status_ns - self.injection_ns) / 1.0e6
            if self.fault_status_ns and self.injection_ns else math.nan
        )
        zero_latency = (
            (self.zero_output_ns - self.injection_ns) / 1.0e6
            if self.zero_output_ns and self.injection_ns else math.nan
        )
        status = self.fault_status or SafetyStatus()
        passed = bool(
            not failure_reason
            and math.isfinite(fault_latency)
            and math.isfinite(zero_latency)
            and zero_latency <= limit_ms
            and status.state == SafetyStatus.STATE_FAULT_LATCHED
            and status.active_faults
            & SafetyStatus.FAULT_MCU_HEARTBEAT_LOST
            and status.latched_faults
            & SafetyStatus.FAULT_MCU_HEARTBEAT_LOST
        )
        if not passed and not failure_reason:
            failure_reason = 'fault/zero evidence missing or latency exceeded'
        row = {
            'schema_version': 1,
            'run_id': self.get_parameter('run_id').value,
            'repetition': self.get_parameter('repetition').value,
            'trial_index': self.get_parameter('trial_index').value,
            'fault_name': self.scenario,
            'configured_heartbeat_timeout_ms': 200.0,
            'command_delay_ms': (
                self.get_parameter('command_delay_ms').value
                if self.scenario == 'command_delay' else 0
            ),
            'acceptance_limit_ms': limit_ms,
            'injection_monotonic_ns': self.injection_ns or 0,
            'fault_status_monotonic_ns': self.fault_status_ns or 0,
            'zero_output_monotonic_ns': self.zero_output_ns or 0,
            'fault_detection_latency_ms': fault_latency,
            'zero_command_latency_ms': zero_latency,
            'pre_fault_linear_x': 0.12,
            'observed_state': int(status.state),
            'observed_active_faults': int(status.active_faults),
            'observed_latched_faults': int(status.latched_faults),
            'trial_pass': passed,
            'failure_reason': failure_reason,
        }
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        needs_header = not self.output_csv.exists()
        with self.output_csv.open('a', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(
                stream, fieldnames=FIELDS, lineterminator='\n'
            )
            if needs_header:
                writer.writeheader()
            writer.writerow(row)
        self.get_logger().info(
            f'{self.scenario}: pass={passed} zero={zero_latency:.3f} ms'
        )
        return passed

    def run(self):
        ready, reason = self._stack_ready()
        if not ready:
            return self._record(reason)
        if not self._wait_for_motion():
            return self._record('nonzero safe command not observed')
        injected, message = self._inject()
        if not injected:
            return self._record(message)
        timeout = float(self.get_parameter('observation_timeout_sec').value)
        self._wait_for(
            lambda: (
                self.fault_status_ns is not None
                and self.zero_output_ns is not None
            ),
            timeout,
        )
        return self._record()


def main(args=None):
    rclpy.init(args=args)
    node = McuFaultTrialRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
