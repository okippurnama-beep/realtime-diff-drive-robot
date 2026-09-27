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
    safety_dir = get_package_share_directory('diffbot_safety')

    output_csv = LaunchConfiguration('output_csv')
    repetitions = LaunchConfiguration('repetitions')
    scenarios = LaunchConfiguration('scenarios')
    dry_run = LaunchConfiguration('dry_run')
    fault_params = os.path.join(
        safety_dir,
        'config',
        'safety_fault_injection_params.yaml',
    )

    benchmark_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'benchmark.launch.py')
        ),
        launch_arguments={
            'safety_params_file': fault_params,
        }.items(),
    )

    runner = Node(
        package='diffbot_safety',
        executable='safety_fault_injection_runner.py',
        name='safety_fault_injection_runner',
        parameters=[
            {
                'use_sim_time': True,
                'output_csv': output_csv,
                'repetitions': ParameterValue(
                    repetitions,
                    value_type=int,
                ),
                'scenarios_csv': scenarios,
                'dry_run': ParameterValue(dry_run, value_type=bool),
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

    stop_after_runner = RegisterEventHandler(
        OnProcessExit(target_action=runner, on_exit=[stop_gazebo])
    )
    shutdown_after_gazebo = RegisterEventHandler(
        OnProcessExit(
            target_action=stop_gazebo,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            'Safety benchmark runner finished and Gazebo '
                            'stop was requested.'
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
                default_value='3',
                description='Number of complete four-fault repetitions.',
            ),
            DeclareLaunchArgument(
                'scenarios',
                default_value='',
                description=(
                    'Optional comma-separated fault names for targeted '
                    'regression runs.'
                ),
            ),
            DeclareLaunchArgument(
                'dry_run',
                default_value='False',
                description='Check stack health without motion or faults.',
            ),
            benchmark_stack,
            runner,
            stop_after_runner,
            shutdown_after_gazebo,
        ]
    )
