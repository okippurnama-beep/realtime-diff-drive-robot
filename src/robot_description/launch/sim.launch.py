from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare('robot_description'),
            'rviz',
            'diffbot.rviz',
        ]
    )
    world_path = PathJoinSubstitution(
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
            'gz_args': ['-r -v 1 ', world_path]
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
            '-z',
            '0.05',
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
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
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
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
            gazebo,
            robot_state_publisher,
            clock_bridge,
            lidar_bridge,
            imu_bridge,
            rviz,
            spawn_robot,
            start_joint_state_broadcaster,
            start_diff_drive_controller,
        ]
    )
