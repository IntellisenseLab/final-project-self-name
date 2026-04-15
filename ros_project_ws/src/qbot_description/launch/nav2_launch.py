import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'qbot_description'
    package_dir = get_package_share_directory(package_name)
    
    urdf_file = os.path.join(package_dir, 'urdf', 'qbot.urdf')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'qbot.rviz')
    robot_controllers = os.path.join(package_dir, 'config', 'qbot_controllers.yaml')
    nav2_params_path = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(package_dir, 'maps', 'map_name.yaml')
    world = os.path.join(package_dir, 'sdf', 'world1.sdf')

    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file,
            'params_file': nav2_params_path,
            'use_composition': 'False', 
            'autostart': 'True',       
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', package_dir),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),

        ExecuteProcess(cmd=['gz', 'sim', world, '-r'], output='screen'),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'qbot', '-z', '0.1'],
            output='screen'
        ),
         Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
               
                '/cmd_vel_smoothed@geometry_msgs/msg/Twist[gz.msgs.Twist'
            ],
          
            remappings=[
                ('/cmd_vel_smoothed', '/diff_drive_controller/cmd_vel')
            ],
            output='screen'
        ),


        Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster', '--param-file', robot_controllers]
        ),

        Node(
            package='controller_manager', executable='spawner',
            arguments=['diff_drive_controller', '--param-file', robot_controllers]
        ),
        
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        ),

        nav2_launch
    ])
