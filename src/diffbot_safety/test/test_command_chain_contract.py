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

"""Static contract tests for the M7.2 velocity safety chain."""

from pathlib import Path
import unittest

import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SOURCE_ROOT = WORKSPACE_ROOT / 'src'


def load_yaml(path):
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


class CommandChainContractTest(unittest.TestCase):
    """Prevent a command bypass or a widened downstream speed envelope."""

    @classmethod
    def setUpClass(cls):
        cls.nav2 = load_yaml(
            SOURCE_ROOT
            / 'diffbot_navigation'
            / 'config'
            / 'nav2_params.yaml'
        )
        cls.safety = load_yaml(
            SOURCE_ROOT
            / 'diffbot_safety'
            / 'config'
            / 'safety_params.yaml'
        )['safety_supervisor']['ros__parameters']
        cls.controller = load_yaml(
            SOURCE_ROOT
            / 'robot_description'
            / 'config'
            / 'controllers.yaml'
        )['/**/diff_drive_base_controller']['ros__parameters']
        cls.navigation_launch = (
            SOURCE_ROOT
            / 'diffbot_navigation'
            / 'launch'
            / 'navigation.launch.py'
        ).read_text(encoding='utf-8')
        cls.bridge_source = (
            SOURCE_ROOT
            / 'robot_description'
            / 'scripts'
            / 'twist_to_twist_stamped_node.py'
        ).read_text(encoding='utf-8')

    def test_topics_form_one_explicit_safety_chain(self):
        collision = self.nav2['collision_monitor']['ros__parameters']
        self.assertEqual(
            collision['cmd_vel_in_topic'],
            '/cmd_vel_smoothed',
        )
        self.assertEqual(
            collision['cmd_vel_out_topic'],
            '/cmd_vel_collision_checked',
        )
        self.assertEqual(
            self.safety['input_command_topic'],
            collision['cmd_vel_out_topic'],
        )
        self.assertEqual(
            self.safety['output_command_topic'],
            '/cmd_vel_safe',
        )
        self.assertIn(
            "'input_topic': '/cmd_vel_safe'",
            self.navigation_launch,
        )
        self.assertIn("'/cmd_vel_safe'", self.bridge_source)

    def test_navigation_launch_starts_supervisor(self):
        self.assertIn("package='diffbot_safety'", self.navigation_launch)
        self.assertIn(
            "executable='safety_supervisor_node'",
            self.navigation_launch,
        )

    def test_velocity_envelopes_do_not_widen_downstream(self):
        smoother = self.nav2['velocity_smoother']['ros__parameters']
        self.assertEqual(smoother['max_velocity'][0], 0.25)
        self.assertEqual(smoother['min_velocity'][0], -0.10)
        self.assertEqual(smoother['max_velocity'][2], 1.0)
        self.assertEqual(smoother['min_velocity'][2], -1.0)

        self.assertEqual(self.safety['max_forward_velocity'], 0.25)
        self.assertEqual(self.safety['max_reverse_velocity'], 0.10)
        self.assertEqual(self.safety['max_angular_velocity'], 1.0)
        self.assertEqual(self.safety['odom_timeout_sec'], 0.50)

        self.assertEqual(
            self.controller['linear.x.max_velocity'],
            0.25,
        )
        self.assertEqual(
            self.controller['linear.x.min_velocity'],
            -0.10,
        )
        self.assertEqual(
            self.controller['angular.z.max_velocity'],
            1.0,
        )
        self.assertEqual(
            self.controller['angular.z.min_velocity'],
            -1.0,
        )
        self.assertEqual(self.controller['cmd_vel_timeout'], 0.5)


if __name__ == '__main__':
    unittest.main()
