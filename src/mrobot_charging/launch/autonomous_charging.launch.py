from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    
    marker_detector_node = Node(
        package="mrobot_charging",
        executable="marker_detector_node",
    )
    
    autonomous_charging_node = Node(
        package="mrobot_charging",
        executable="autonomous_charging_node",
    )

    return LaunchDescription([
        marker_detector_node,
        autonomous_charging_node
    ])