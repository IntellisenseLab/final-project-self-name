from launch import LaunchDescription 
from launch_ros.actions import Node
import os  #Allows python to work with files 
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    #Find the path to the urdf file 
    urdf_file = os.path.join(
        get_package_share_directory('qbot_description'),    #Finds the directory of the package 
        'urdf',
        'qbot.urdf'
    )
    #Find the path to the rviz config file
    rviz_config_path = os.path.join(
    get_package_share_directory('qbot_description'),
    'rviz',
    'qbot.rviz'
)
     # Reads the urdf file and stores it as a string
    with open(urdf_file, 'r') as file:
     robot_description = file.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] #-d specifies the config file to use for rviz
        )
    ])