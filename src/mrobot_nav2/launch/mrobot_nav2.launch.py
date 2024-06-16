import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    map_file = os.path.join(get_package_share_directory('mrobot_mapping'), 'maps', 'small_warehouse', 'map.yaml')
    nav2_yaml_config = os.path.join(get_package_share_directory('mrobot_nav2'), 'config', 'amcl_config.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            # 'params_file': nav2_yaml_config, 
            'map': map_file
        }.items()
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                pkg_nav2_dir,
                'rviz',
                'nav2_default_view.rviz'
        )],
    )

    set_init_amcl_pose = Node(
        package="mrobot_nav2",
        executable="set_init_amcl_pose",
        name="set_init_amcl_pose",
        parameters=[{
            "x": 6.0,
            "y": 2.17,
            "theta": -3.14
        }],
    )

    return LaunchDescription([
        nav2_launch,
        rviz_launch,
        set_init_amcl_pose
    ])