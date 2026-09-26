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
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    navigation_dir = get_package_share_directory('diffbot_navigation')

    output_csv = LaunchConfiguration('output_csv')
    goal_names = LaunchConfiguration('goal_names')
    repetitions = LaunchConfiguration('repetitions')
    return_to_start = LaunchConfiguration('return_to_start')
    goal_timeout_sec = LaunchConfiguration('goal_timeout_sec')
    dry_run = LaunchConfiguration('dry_run')

    benchmark_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'benchmark.launch.py')
        )
    )

    runner = Node(
        package='diffbot_navigation',
        executable='nav_benchmark_runner.py',
        name='nav_benchmark_runner',
        parameters=[
            {
                'use_sim_time': True,
                'output_csv': output_csv,
                'goal_names_csv': goal_names,
                'repetitions': ParameterValue(
                    repetitions,
                    value_type=int,
                ),
                'return_to_start': ParameterValue(
                    return_to_start,
                    value_type=bool,
                ),
                'goal_timeout_sec': ParameterValue(
                    goal_timeout_sec,
                    value_type=float,
                ),
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'dry_run_wait_for_server': True,
                'server_timeout_sec': 120.0,
            }
        ],
        output='screen',
    )

    stop_gazebo = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'),
            'service',
            '-s',
            '/server_control',
            '--reqtype',
            'gz.msgs.ServerControl',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '5000',
            '--req',
            'stop: true',
        ],
        output='screen',
    )

    stop_gazebo_after_runner = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[stop_gazebo],
        )
    )

    shutdown_after_gazebo = RegisterEventHandler(
        OnProcessExit(
            target_action=stop_gazebo,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            'Navigation benchmark runner finished and '
                            'Gazebo stop was requested.'
                        )
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'output_csv',
                default_value='',
                description=(
                    'CSV path. Empty uses benchmark_results/ under the '
                    'current working directory.'
                ),
            ),
            DeclareLaunchArgument(
                'repetitions',
                default_value='1',
                description='Number of complete five-goal repetitions.',
            ),
            DeclareLaunchArgument(
                'goal_names',
                default_value='',
                description=(
                    'Optional comma-separated goal names for targeted '
                    'regression runs.'
                ),
            ),
            DeclareLaunchArgument(
                'return_to_start',
                default_value='True',
                description='Navigate back to the fixed start after each trial.',
            ),
            DeclareLaunchArgument(
                'goal_timeout_sec',
                default_value='180.0',
                description='Per-goal timeout before cancellation.',
            ),
            DeclareLaunchArgument(
                'dry_run',
                default_value='False',
                description='Validate configuration without sending goals.',
            ),
            benchmark_stack,
            runner,
            stop_gazebo_after_runner,
            shutdown_after_gazebo,
        ]
    )
