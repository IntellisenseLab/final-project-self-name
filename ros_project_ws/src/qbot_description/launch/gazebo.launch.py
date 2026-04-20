import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import tempfile

def generate_launch_description():
    package_name = 'qbot_description'
    package_dir = get_package_share_directory(package_name)
    
    urdf_file = os.path.join(package_dir, 'urdf', 'qbot.urdf')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'qbot.rviz')
    robot_controllers = os.path.join(package_dir, 'config', 'qbot_controllers.yaml')
   
    world = os.path.join(package_dir, 'sdf', 'world1.sdf')

    with open(urdf_file, 'r') as file:
        robot_description = file.read()
   

    # Fix hardcoded path in URDF
    robot_description = robot_description.replace(
        '/home/senadi/Desktop/final-project-self-name/ros_project_ws/src/qbot_description/config/qbot_controllers.yaml',
        robot_controllers
    )

    # Write corrected URDF to a temp file so Gazebo can read it
    tmp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    tmp_urdf.write(robot_description)
    tmp_urdf.close()
    
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', package_dir),

        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            '/opt/ros/jazzy/lib'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', world, '-r'],
            output='screen'
        ),

        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                arguments=['-file', tmp_urdf.name, '-name', 'qbot', '-z', '0.1'],
                output='screen'
            )
        ]),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/diff_drive_controller/cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist'
            ],
            remappings=[
                ('/odometry', '/odom')
            ],
            output='screen'
        ),

        TimerAction(period=5.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--param-file', robot_controllers],
                output='screen'
            )
        ]),

        TimerAction(period=6.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--param-file', robot_controllers],
                output='screen'
            )
        ]),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='qbot_description',
            executable='twist_to_stamped.py',
            name='twist_to_stamped',
            output='screen',
        ),
    ])
