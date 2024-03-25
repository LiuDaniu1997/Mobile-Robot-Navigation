from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_cpp = Node(
        package = 'mrobot_controller',
        executable= 'noisy_controller',
        parameters= [{
            'wheel_radius': wheel_radius + wheel_radius_error,
            'wheel_separation': wheel_separation + wheel_separation_error
        }]
    )

    return [noisy_controller_cpp]

def generate_launch_description():

    dec_wheel_radius = DeclareLaunchArgument(
        name = 'wheel_radius',
        default_value = '0.14'
    )
    wheel_radius = LaunchConfiguration("wheel_radius")

    dec_wheel_radius_error = DeclareLaunchArgument(
        name = 'wheel_radius_error',
        default_value = '0.005'
    )

    dec_wheel_separation = DeclareLaunchArgument(
        name = 'wheel_separation',
        default_value = '0.52'
    )
    wheel_separation = LaunchConfiguration("wheel_separation")

    dec_wheel_separation_error = DeclareLaunchArgument(
        name = 'wheel_separation_error',
        default_value = '0.02'
    )

    dec_use_simple_controller = DeclareLaunchArgument(
        name = 'use_simple_controller',
        default_value= 'True'
    )
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    joint_state_broadcaster_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments= [
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    wheel_controller_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments= [
            'mrobot_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        condition = UnlessCondition(use_simple_controller)
    )

    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions = [
            Node(
                package = 'controller_manager',
                executable = 'spawner',
                arguments= [
                    'simple_velocity_controller',
                    '--controller-manager',
                    '/controller_manager'
                ]
            ),

            Node(
            package = 'mrobot_controller',
            executable = 'simple_controller',
            parameters= [{
                    'wheel_radius': wheel_radius,
                    'wheel_separation': wheel_separation   
            }]
            )
        ]
    )

    noisy_controller_launch = OpaqueFunction(
        function = noisy_controller
    )

    return LaunchDescription([
        dec_wheel_radius,
        dec_wheel_radius_error,
        dec_wheel_separation,
        dec_wheel_separation_error,
        dec_use_simple_controller,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,
        noisy_controller_launch
    ])