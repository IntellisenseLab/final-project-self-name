# launch/apriltag_kinect.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    tags_config = PathJoinSubstitution([
        FindPackageShare('leader_detection'),  
        'config',
        'tags.yaml'
    ])

    return LaunchDescription([

        DeclareLaunchArgument('leader_tag_id', default_value='0'),
        DeclareLaunchArgument('tag_family', default_value='36h11'),
        DeclareLaunchArgument('tag_size', default_value='0.165'),

        Node(
            package='kinect_ros2',
            executable='kinect_ros2_node',
            name='kinect_ros2',
            output='screen',
        ),

        ComposableNodeContainer(
            name='image_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_rgb',
                    remappings=[
                        ('image', '/image_raw'),
                        ('camera_info', '/camera_info'),
                        ('image_rect', '/image_rect'),
                    ],
                ),
            ],
            output='screen',
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            remappings=[
                ('image_rect', '/image_rect'),
                ('camera_info', '/camera_info'),
            ],
            parameters=[
                tags_config,
                {
                    'family': LaunchConfiguration('tag_family'),
                    'size': LaunchConfiguration('tag_size'),
                },
            ],
            output='screen',
        ),

        Node(
            package='leader_detection',     
            executable='detection_node',    
            name='leader_detection_node',
            parameters=[{
                'leader_tag_id': LaunchConfiguration('leader_tag_id'),
                'tag_family': LaunchConfiguration('tag_family'),
            }],
            output='screen',
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_depth_view',
            arguments=['/depth/image_raw'],
            output='screen',
        ),

    ])