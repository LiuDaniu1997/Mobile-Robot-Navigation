import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mrobot_description_dir = get_package_share_directory("mrobot_description")
    mrobot_description_prefix = get_package_prefix("mrobot_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    dec_urdf_path = DeclareLaunchArgument(name="urdf", 
                                    default_value=os.path.join(mrobot_description_dir, "urdf", "mrobot.urdf.xacro"),
                                    description="Absolute path to robot urdf file")

    dec_world_name = DeclareLaunchArgument(name="world_name", default_value="no_roof_small_warehouse")
    
    world_path = PathJoinSubstitution([
            mrobot_description_dir,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = os.path.join(mrobot_description_dir, "models")
    model_path += pathsep + os.path.join(mrobot_description_prefix, "share")
    
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("urdf")]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, 
                     "use_sim_time":use_sim_time}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_path}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "mrobot",
                            "-topic", "robot_description",
                            "-x", '6.0',
                            "-y", '2.17',
                            "-z", '0.0',
                            "-Y", '-3.14'],
                        output="screen"
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time":use_sim_time}]
    )

    return LaunchDescription([
        env_var,
        dec_urdf_path,
        dec_world_name,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        joint_state_publisher_node,
    ])