from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Minimal, just kinect driver and robot_state_publisher. Everything else runs on the laptop.
 
def generate_launch_description():
 
    pkg_share = get_package_share_directory('qbot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'qbot.urdf')
 
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
 
    return LaunchDescription([
 
        # TF tree from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
 
        # Kinect v1 driver
        Node(
            package='kinect_ros2',
            executable='kinect_ros2_node',
            name='kinect_ros2',
            output='screen',
        ),
 
    ])