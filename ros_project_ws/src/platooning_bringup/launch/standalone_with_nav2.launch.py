import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('platooning_bringup')
    qbot_dir = get_package_share_directory('qbot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(qbot_dir, 'maps', 'lab_map_7.yaml')

    standalone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'standalone.launch.py')
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'map': map_file,
            'params_file': nav2_params,
            'use_composition': 'False',
            'autostart': 'True',
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qbot_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params,
            'use_composition': 'False',
            'autostart': 'True',
            'use_respawn' : 'True',
        }.items()
    )

    return LaunchDescription([
        standalone_launch,

        # Start localization after sensors initialize
        TimerAction(period=10.0, actions=[localization_launch]),

        # Start navigation after localization
        TimerAction(period=10.0, actions=[navigation_launch]),

        # Map and localization first
        TimerAction(period=15.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'], output='screen')]),
        TimerAction(period=16.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'], output='screen')]),
        TimerAction(period=17.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/amcl', 'configure'], output='screen')]),
        TimerAction(period=18.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/amcl', 'activate'], output='screen')]),

        # Global localization
        TimerAction(period=19.0, actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/reinitialize_global_localization',
                'std_srvs/srv/Empty', '{}'], output='screen')]),

        # Navigation nodes — controller MUST come before bt_navigator
        TimerAction(period=20.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'configure'], output='screen')]),
        TimerAction(period=21.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'activate'], output='screen')]),
        TimerAction(period=22.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/planner_server', 'configure'], output='screen')]),
        TimerAction(period=23.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/planner_server', 'activate'], output='screen')]),
        TimerAction(period=24.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/behavior_server', 'configure'], output='screen')]),
        TimerAction(period=25.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/behavior_server', 'activate'], output='screen')]),
        TimerAction(period=26.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/smoother_server', 'configure'], output='screen')]),
        TimerAction(period=27.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/smoother_server', 'activate'], output='screen')]),
        TimerAction(period=28.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/velocity_smoother', 'configure'], output='screen')]),
        TimerAction(period=29.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/velocity_smoother', 'activate'], output='screen')]),
        TimerAction(period=30.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/collision_monitor', 'configure'], output='screen')]),
        TimerAction(period=31.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/collision_monitor', 'activate'], output='screen')]),
        # bt_navigator LAST — needs controller_server follow_path action
        TimerAction(period=32.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/bt_navigator', 'configure'], output='screen')]),
        TimerAction(period=33.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/bt_navigator', 'activate'], output='screen')]),
        TimerAction(period=34.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/waypoint_follower', 'configure'], output='screen')]),
        TimerAction(period=35.0, actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/waypoint_follower', 'activate'], output='screen')]),
    ])