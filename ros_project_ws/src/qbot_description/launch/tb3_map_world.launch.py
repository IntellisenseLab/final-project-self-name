from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():

    world = "/home/senadi/Desktop/final-project-self-name/ros_project_ws/src/qbot_description/sdf/world1.sdf"

    slam_yaml = "/home/senadi/Desktop/final-project-self-name/ros_project_ws/src/qbot_description/config/mapper_params_online_async.yaml"

    return LaunchDescription([

        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'waffle'
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', world, '-r'],
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_yaml, {'use_sim_time': True}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'teleop_twist_keyboard',
                'teleop_keyboard'
            ],
            output='screen'
        )
    ])