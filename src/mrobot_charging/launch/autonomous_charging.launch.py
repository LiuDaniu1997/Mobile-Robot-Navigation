from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    autonomous_charging_node = Node(
        package="mrobot_charging",
        executable="autonomous_charging_node",
    )

    return LaunchDescription([
        autonomous_charging_node
    ])