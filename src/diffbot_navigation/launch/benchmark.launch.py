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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation_dir = get_package_share_directory('diffbot_navigation')
    robot_description_dir = get_package_share_directory('robot_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    navigation_delay = LaunchConfiguration('navigation_delay')
    world_path = os.path.join(
        navigation_dir, 'worlds', 'nav_benchmark_world.sdf'
    )
    map_path = os.path.join(
        navigation_dir, 'maps', 'nav_benchmark_map.yaml'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_dir, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'spawn_x': '0.30',
            'spawn_y': '0.00',
            'spawn_z': '0.05',
            'spawn_yaw': '0.0',
            'gz_args': '-r -s -v 1',
            'headless': 'True',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='True',
                description='Use the Gazebo simulation clock.',
            ),
            DeclareLaunchArgument(
                'navigation_delay',
                default_value='8.0',
                description=(
                    'Wall-clock delay before Nav2 starts, allowing Gazebo and '
                    'ros2_control to become ready.'
                ),
            ),
            simulation,
            TimerAction(period=navigation_delay, actions=[navigation]),
        ]
    )
