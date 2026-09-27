from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare('robot_description')
    model_path = PathJoinSubstitution(
        [description_share, 'urdf', 'diffbot.urdf.xacro']
    )
    controllers_path = PathJoinSubstitution(
        [description_share, 'config', 'controllers.yaml']
    )
    safety_path = PathJoinSubstitution(
        [description_share, 'config', 'fake_mcu_safety.yaml']
    )
    rviz_path = PathJoinSubstitution(
        [description_share, 'rviz', 'diffbot.rviz']
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                model_path,
                ' use_fake_mcu:=true',
            ]
        ),
        value_type=str,
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        remappings=[('~/robot_description', '/robot_description')],
        output='screen',
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
            '--param-file',
            controllers_path,
        ],
        output='screen',
    )
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30',
            '--param-file',
            controllers_path,
        ],
        output='screen',
    )
    start_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_controller],
        )
    )
    safety_supervisor = Node(
        package='diffbot_safety',
        executable='safety_supervisor_node',
        parameters=[safety_path],
        output='screen',
    )
    command_adapter = Node(
        package='robot_description',
        executable='twist_to_twist_stamped_node.py',
        parameters=[
            {
                'input_topic': '/cmd_vel_safe',
                'output_topic': '/diff_drive_base_controller/cmd_vel',
            }
        ],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        condition=UnlessCondition(LaunchConfiguration('headless')),
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'headless',
                default_value='true',
                description='Skip RViz for automated M8 tests.',
            ),
            state_publisher,
            controller_manager,
            joint_state_broadcaster,
            start_diff_drive,
            safety_supervisor,
            command_adapter,
            rviz,
        ]
    )
