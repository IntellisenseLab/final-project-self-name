import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('qbot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(package_dir, 'maps', 'map_name.yaml')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'gazebo.launch.py')
        )
    )

    # Localization — AMCL with saved map (system version is fine)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file,
            'params_file': nav2_params,
            'use_composition': 'False',
            'autostart': 'True',
        }.items()
    )

    # Navigation — local copy with docking_server removed
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'use_composition': 'False',
            'autostart': 'True',
        }.items()
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=10.0, actions=[localization_launch]),
        TimerAction(period=10.0, actions=[navigation_launch]),
    ])