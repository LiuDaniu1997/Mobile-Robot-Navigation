from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    feature_tracker_node = Node(
        package = 'mrobot_vio',
        executable = 'feature_tracker'
    )

    return LaunchDescription([
        feature_tracker_node
    ])