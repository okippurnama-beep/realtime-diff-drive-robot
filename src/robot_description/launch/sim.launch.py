from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = PathJoinSubstitution(
        [
            FindPackageShare('robot_description'),
            'urdf',
            'diffbot.urdf.xacro',
        ]
    )

    controllers_path = PathJoinSubstitution(
        [
            FindPackageShare('robot_description'),
            'config',
            'controllers.yaml',
        ]
    )
    ekf_config = PathJoinSubstitution(
        [
            FindPackageShare('robot_description'),
            'config',
            'ekf.yaml',
        ]
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare('robot_description'),
            'rviz',
            'diffbot.rviz',
        ]
    )
    default_world_path = PathJoinSubstitution(
        [
            FindPackageShare('robot_description'),
            'worlds',
            'diffbot_world.sdf',
        ]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', model_path]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('ros_gz_sim'),
                        'launch',
                        'gz_sim.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'gz_args': [
                LaunchConfiguration('gz_args'),
                ' ',
                LaunchConfiguration('world'),
            ]
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'intern_diffbot',
            '-x',
            LaunchConfiguration('spawn_x'),
            '-y',
            LaunchConfiguration('spawn_y'),
            '-z',
            LaunchConfiguration('spawn_z'),
            '-Y',
            LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            controllers_path,
        ],
        output='screen',
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        parameters=[{'override_frame_id': 'lidar_link'}],
        output='screen',
    )
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        parameters=[{'override_frame_id': 'imu_link'}],
        remappings=[('/imu', '/imu/raw')],
        output='screen',
    )
    imu_covariance_node = Node(
        package='robot_description',
        executable='imu_covariance_node.py',
        name='imu_covariance_node',
        parameters=[
            {
                'use_sim_time': True,
                'orientation_variance': 1.0e-4,
                'angular_velocity_variance': 1.0e-4,
                'linear_acceleration_variance': 1.0e-2,
            }
        ],
        output='screen',
    )
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(LaunchConfiguration('headless')),
        output='screen',
    )
    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    start_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_controller],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=default_world_path,
                description='Absolute path to the Gazebo world SDF file.',
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value='-r -v 1',
                description='Arguments passed to Gazebo before the world.',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='False',
                description='Skip RViz for automated, headless runs.',
            ),
            DeclareLaunchArgument(
                'spawn_x',
                default_value='0.0',
                description='Robot spawn X coordinate in metres.',
            ),
            DeclareLaunchArgument(
                'spawn_y',
                default_value='0.0',
                description='Robot spawn Y coordinate in metres.',
            ),
            DeclareLaunchArgument(
                'spawn_z',
                default_value='0.05',
                description='Robot spawn Z coordinate in metres.',
            ),
            DeclareLaunchArgument(
                'spawn_yaw',
                default_value='0.0',
                description='Robot spawn yaw angle in radians.',
            ),
            gazebo,
            robot_state_publisher,
            clock_bridge,
            lidar_bridge,
            imu_bridge,
            imu_covariance_node,
            ekf_filter_node,
            rviz,
            spawn_robot,
            start_joint_state_broadcaster,
            start_diff_drive_controller,
        ]
    )
