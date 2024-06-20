from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    bt_ros_node = Node(
        package="mrobot_charging",
        executable="bt_ros_node",
    )

    return LaunchDescription([
        bt_ros_node
    ])