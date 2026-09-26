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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_dir = get_package_share_directory('diffbot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'slam': 'False',
            'use_localization': 'True',
            'use_composition': 'False',
        }.items(),
    )

    velocity_bridge = Node(
        package='robot_description',
        executable='twist_to_twist_stamped_node.py',
        name='twist_to_twist_stamped_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map',
                description='Absolute path to the occupancy-grid map YAML file.',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    navigation_dir, 'config', 'nav2_params.yaml'
                ),
                description='Diffbot-specific Nav2 parameter file to load.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='True',
                description='Use the Gazebo simulation clock.',
            ),
            DeclareLaunchArgument(
                'autostart',
                default_value='True',
                description='Automatically activate Nav2 lifecycle nodes.',
            ),
            nav2_bringup,
            velocity_bridge,
        ]
    )
