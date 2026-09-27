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
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('robot_description')
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'fake_mcu.launch.py')
        ),
        launch_arguments={'headless': 'true'}.items(),
    )
    runner = Node(
        package='diffbot_hardware',
        executable='mcu_fault_trial_runner.py',
        parameters=[{
            'scenario': LaunchConfiguration('scenario'),
            'repetition': ParameterValue(
                LaunchConfiguration('repetition'), value_type=int
            ),
            'trial_index': ParameterValue(
                LaunchConfiguration('trial_index'), value_type=int
            ),
            'run_id': LaunchConfiguration('run_id'),
            'output_csv': LaunchConfiguration('output_csv'),
        }],
        output='screen',
    )
    stop_after_runner = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[EmitEvent(event=Shutdown(reason='M8.5 trial finished'))],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument('scenario'),
        DeclareLaunchArgument('repetition'),
        DeclareLaunchArgument('trial_index'),
        DeclareLaunchArgument('run_id'),
        DeclareLaunchArgument('output_csv'),
        stack,
        runner,
        stop_after_runner,
    ])
