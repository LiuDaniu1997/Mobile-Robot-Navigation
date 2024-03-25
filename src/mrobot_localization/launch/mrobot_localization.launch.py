from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():    
    # path of ekf config file
    ekf_path = os.path.join(get_package_share_directory("mrobot_localization"), "config", "ekf.yaml")
    
    # robot localization node
    robot_localization = Node(
        package = 'robot_localization',
        executable = 'ekf_node',
        name = 'ekf_filter_node',
        output = 'screen',
        parameters = [
            ekf_path
        ]
    )

    return LaunchDescription([
        robot_localization
    ])