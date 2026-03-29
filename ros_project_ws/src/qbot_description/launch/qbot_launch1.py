from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_name = 'qbot_description'

    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'qbot.urdf'
    )

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'qbot.rviz'
    )

    controller_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'qbot_controllers.yaml'
    )

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'sdf',
        'qbot_world.sdf'
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
        output='screen'
    )

    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config, {
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-topic', 'robot_description',
        '-name', 'qbot'
    ],
    output='screen'
)



    spawn_delayed = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            get_package_share_directory(package_name)
        ),

        gazebo,

        robot_state_publisher,

        controller_manager,

        spawn_delayed,

        joint_state_broadcaster,
        diff_drive_controller,

        bridge,

        rviz
    ])