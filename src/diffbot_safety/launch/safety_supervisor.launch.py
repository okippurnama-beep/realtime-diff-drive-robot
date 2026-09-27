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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('diffbot_safety')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    package_dir, 'config', 'safety_params.yaml'
                ),
                description='Safety supervisor parameter file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                description='Use ROS time only for status timestamps.',
            ),
            Node(
                package='diffbot_safety',
                executable='safety_supervisor_node',
                name='safety_supervisor',
                parameters=[
                    params_file,
                    {'use_sim_time': use_sim_time},
                ],
                output='screen',
            ),
        ]
    )
