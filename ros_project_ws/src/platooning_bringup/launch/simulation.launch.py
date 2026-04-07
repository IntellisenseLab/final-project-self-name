from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
 
 
def generate_launch_description():
 
    tags_config = os.path.join(
        get_package_share_directory('leader_detection'),
        'config',
        'tags.yaml'
    )
 
    # TODO: Add Gazebo world launch once qbot_description simulation is working
    # TODO: Add AprilTag model to Gazebo world
    # TODO: Remap camera topics to /rgbd_camera/image and /rgbd_camera/depth_image
 
    return LaunchDescription([
 
        # TODO: Launch Gazebo simulation
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('qbot_description'),
        #             'launch',
        #             'gazebo.launch.py'
        #         ])
        #     ])
        # ),
 
        # AprilTag detection (remapped to simulation camera topics)
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            remappings=[
                ('image_rect', '/rgbd_camera/image'),       # simulation camera topic
                ('camera_info', '/rgbd_camera/camera_info'),
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
 
        # Safety monitor (depth topic remapped to simulation)
        Node(
            package='safety_monitor',
            executable='safety_monitor',
            name='safety_monitor',
            parameters=[{
                'depth_topic': '/rgbd_camera/depth_image'   # TODO: remap in node or pass as param
            }],
            output='screen',
        ),
 
    ])
