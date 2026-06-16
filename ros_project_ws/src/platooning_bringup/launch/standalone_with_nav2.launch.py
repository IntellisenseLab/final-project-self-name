import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('platooning_bringup')
    qbot_dir = get_package_share_directory('qbot_description')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(qbot_dir, 'maps', 'lab_map_new2.yaml')

    standalone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'standalone.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                qbot_dir,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'map': map_file,
            'params_file': nav2_params,
            'use_composition': 'False',
            'autostart': 'True',
        }.items()
    )

    return LaunchDescription([
        standalone_launch,
        TimerAction(period=10.0, actions=[nav2_launch]),
    ])