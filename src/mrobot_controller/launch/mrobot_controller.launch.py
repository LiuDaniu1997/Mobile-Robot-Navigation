from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    wheel_controller_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments= [
            'mrobot_controller',
            '--controller-manager',
            '/controller_manager'
        ],
    )

    move_mrobot = Node(
        package="mrobot_controller",
        executable="move_mrobot",
        name="move_mrobot"
    )

    return LaunchDescription([
        wheel_controller_spawner,
        move_mrobot
    ])