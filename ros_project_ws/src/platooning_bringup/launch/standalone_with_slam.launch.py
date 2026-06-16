import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('platooning_bringup')
    qbot_description = get_package_share_directory('qbot_description')
    nav2_params = os.path.join(qbot_description, 'config', 'nav2_params.yaml')
    slam_config = os.path.join(qbot_description, 'config', 'slam.yaml')

    # Include base standalone launch
    standalone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'standalone.launch.py')
        )
    )

    # SLAM Toolbox in real-time mapping mode
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'False',
        }.items()
    )

    # Nav2 using live SLAM map
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'slam': 'True',        # tells Nav2 to use live SLAM map
            'params_file': nav2_params,
            'use_composition': 'False',
            'autostart': 'True',
        }.items()
    )

    return LaunchDescription([
        standalone_launch,

        # Delay SLAM to let robot and sensors initialize
        TimerAction(period=5.0, actions=[slam_launch]),

        # Delay Nav2 further to let SLAM build initial map
        TimerAction(period=15.0, actions=[nav2_launch]),
    ])