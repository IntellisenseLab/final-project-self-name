from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    world = "/home/senadi/Desktop/final-project-self-name/ros_project_ws/src/qbot_description/sdf/world1.sdf"

    return LaunchDescription([

        # TurtleBot3 model
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'waffle'
        ),

        # Start Gazebo with YOUR world
        ExecuteProcess(
            cmd=['gz', 'sim', world, '-r'],
            output='screen'
        ),

        # Spawn TurtleBot3 robot
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch', 'turtlebot3_gazebo',
                        'turtlebot3_world.launch.py'
                    ],
                    output='screen'
                )
            ]
        ),

        # SLAM Toolbox
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'odom_frame': 'odom',
                        'map_frame': 'map',
                        'base_frame': 'base_footprint',
                        'scan_topic': '/scan',
                        'mode': 'mapping'
                    }]
                )
            ]
        ),

        # RViz
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),

        # Teleop control
        TimerAction(
            period=14.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'teleop_twist_keyboard',
                        'teleop_twist_keyboard'
                    ],
                    output='screen'
                )
            ]
        ),
    ])