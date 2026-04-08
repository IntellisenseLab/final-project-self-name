from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():
 
    pkg_share = get_package_share_directory('qbot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'qbot.urdf')
    tags_config = os.path.join(
        get_package_share_directory('leader_detection'),
        'config',
        'tags.yaml'
    )
    
    package_dir = get_package_share_directory('platooning_bringup')
    params_file = os.path.join(package_dir, 'config', 'kobuki_node_params.yaml')

    with open(params_file, 'r') as f:
        kobuki_params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

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
 
        # AprilTag detection
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            remappings=[
                ('image_rect', '/image_raw'),
                ('camera_info', '/camera_info'),
            ],
            parameters=[
                tags_config,
                {
                    'approximate_sync': True,
                    'camera_frame': 'camera_link'
                }
            ],
            output='screen',
        ),
 
        # Leader detection node
        Node(
            package='leader_detection',
            executable='detection_node',
            name='leader_detection_node',
            parameters=[tags_config],
            output='screen',
        ),
 
        # PID controller
        Node(
            package='platooning_pid',
            executable='pid_node',
            name='pid_controller',
            output='screen',
        ),
 
        # Safety monitor
        Node(
            package='safety_monitor',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
        ),

        Node(
            package='kobuki_node',
            executable='kobuki_ros_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[kobuki_params],
            remappings=[
                ('/commands/velocity', '/cmd_vel')
            ]
)
    ])