from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    gazebo_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("mrobot_description"), '/launch', '/gazebo.launch.py'])
    )

    mrobot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("mrobot_controller"), '/launch', '/mrobot_controller.launch.py'])
    )

    mrobot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("mrobot_localization"), '/launch', '/mrobot_localization.launch.py'])
    )
    
    return LaunchDescription([
        gazebo_simulation_launch,
        mrobot_controller_launch,
        mrobot_localization_launch,
    ])