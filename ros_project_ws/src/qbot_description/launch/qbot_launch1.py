from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_name = 'qbot_description'

    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'qbot.urdf')
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'qbot.rviz')
    world_file = os.path.join(get_package_share_directory(package_name), 'sdf', 'world1.sdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
       
       
       
       
       
            arguments=[
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
],
      
        output='screen'
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

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    delay_spawners = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster, diff_drive_controller]
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        remappings=[('cmd_vel', '/cmd_vel')],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(package_name), 'config', 'slam.yaml'),
            {
                'use_sim_time': True,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'map_frame': 'map',
                'scan_topic': 'scan',
                'mode': 'mapping',
                'transform_publish_period': 0.02,
            }
        ],
    )

    slam_configure = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'configure'],
                output='screen'
            )
        ]
    )

    slam_activate = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'activate'],
                output='screen'
            )
        ]
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
        delay_spawners,
        teleop,
        slam_node,
        slam_configure,
        slam_activate,
        rviz
    ])