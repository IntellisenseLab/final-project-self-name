from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_name = 'qbot_description'

    world_file = os.path.join(
        get_package_share_directory(package_name),
        'sdf',
        'world1.sdf'
    )

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'qbot.rviz'
    )

    map_file = os.path.join(
        get_package_share_directory(package_name),
        'maps',
        'my_map.yaml'
    )

    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'qbot.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        # '-r' makes the simulation run immediately so ros2_control can switch controllers.
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'qbot',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/robot1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/robot1/scan', '/scan'),
        ],
        output='screen'
    )

    joint_state_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--switch-timeout', '30.0'],
                output='screen'
            )
        ]
    )

    diff_drive_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--switch-timeout', '30.0'],
                output='screen'
            )
        ]
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': True
        }],
        output='screen'
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        remappings=[
            ('/odom', '/diff_drive_controller/odom'),
        ],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        output='screen'
    )

    nav2_start_delay = TimerAction(
        period=5.0,
        actions=[map_server, amcl]
    )

    nav2_manager_delay = TimerAction(
        period=7.0,
        actions=[lifecycle_manager]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            get_package_share_directory(package_name)
        ),

        gazebo,
        bridge,
        robot_state_publisher,
        spawn_entity,

        joint_state_spawner,
        diff_drive_spawner,

        nav2_start_delay,
        nav2_manager_delay,

        rviz
    ])