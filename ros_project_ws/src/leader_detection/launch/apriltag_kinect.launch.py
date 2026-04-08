from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Paths
    pkg_share = get_package_share_directory('qbot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'qbot.urdf')
    tags_config = os.path.join(
        get_package_share_directory('leader_detection'),
        'config',
        'tags.yaml'
    )

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }]
    )

    return LaunchDescription([

        # TF tree from URDF
        robot_state_publisher_node,

        # Kinect driver
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
            parameters=[tags_config,
                        {'approximate_sync': True,
                         'camera_frame': 'camera_link'},
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

        # Debug viewer
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_depth_view',
            arguments=['/depth/image_raw'],
            output='screen',
        ),

    ])