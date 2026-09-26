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

"""Regression tests for the measured Nav2 baseline configuration."""

from pathlib import Path
import unittest

import yaml


PARAMS_PATH = (
    Path(__file__).resolve().parents[1] / 'config' / 'nav2_params.yaml'
)


class Nav2ParamsContractTest(unittest.TestCase):
    """Protect configuration relationships found during benchmark tuning."""

    @classmethod
    def setUpClass(cls):
        with PARAMS_PATH.open(encoding='utf-8') as stream:
            cls.params = yaml.safe_load(stream)

    def ros_params(self, node_name):
        return self.params[node_name]['ros__parameters']

    def test_controller_uses_filtered_odometry(self):
        controller = self.ros_params('controller_server')
        self.assertEqual(controller['odom_topic'], '/odometry/filtered')

    def test_pose_progress_checker_accepts_rotation(self):
        controller = self.ros_params('controller_server')
        checker = controller['progress_checker']
        self.assertEqual(
            checker['plugin'],
            'nav2_controller::PoseProgressChecker',
        )
        self.assertGreater(checker['required_movement_angle'], 0.0)

    def test_rpp_aligns_before_driving(self):
        controller = self.ros_params('controller_server')
        follow_path = controller['FollowPath']
        self.assertEqual(
            follow_path['plugin'],
            'nav2_regulated_pure_pursuit_controller::'
            'RegulatedPurePursuitController',
        )
        self.assertTrue(follow_path['use_rotate_to_heading'])
        self.assertLessEqual(follow_path['rotate_to_heading_min_angle'], 0.15)

    def test_static_global_and_live_local_obstacles_are_separated(self):
        global_costmap = self.params['global_costmap'][
            'global_costmap'
        ]['ros__parameters']
        local_costmap = self.params['local_costmap'][
            'local_costmap'
        ]['ros__parameters']
        collision_monitor = self.ros_params('collision_monitor')

        self.assertEqual(
            global_costmap['plugins'],
            ['static_layer', 'inflation_layer'],
        )
        self.assertIn('voxel_layer', local_costmap['plugins'])
        self.assertTrue(local_costmap['voxel_layer']['enabled'])
        self.assertIn('scan', collision_monitor['observation_sources'])
        self.assertTrue(collision_monitor['scan']['enabled'])

    def test_controller_and_smoother_limits_are_consistent(self):
        follow_path = self.ros_params('controller_server')['FollowPath']
        smoother = self.ros_params('velocity_smoother')

        self.assertLessEqual(
            follow_path['desired_linear_vel'],
            smoother['max_velocity'][0],
        )
        self.assertLessEqual(
            follow_path['rotate_to_heading_angular_vel'],
            smoother['max_velocity'][2],
        )
        self.assertLessEqual(
            follow_path['max_angular_accel'],
            smoother['max_accel'][2],
        )

    def test_sim_amcl_model_is_bounded(self):
        amcl = self.ros_params('amcl')
        for name in ('alpha1', 'alpha2', 'alpha3', 'alpha4'):
            self.assertLessEqual(amcl[name], 0.05)
        self.assertLessEqual(amcl['alpha5'], 0.01)
        self.assertGreaterEqual(amcl['max_beams'], 120)
        self.assertAlmostEqual(
            amcl['z_hit'] + amcl['z_rand'],
            1.0,
        )


if __name__ == '__main__':
    unittest.main()
