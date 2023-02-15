import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals


def generate_launch_description():
    return LaunchDescription([
        # robot config file
        DeclareLaunchArgument(
            name="robot_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("waypoint_navigator"), "configs", "robot_config.yaml"]
            )
        ),
        #
        DeclareLaunchArgument(
            name="path_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("waypoint_navigator"), "paths", "trajectory_simple_enu.yaml"]
            )
        ),        
        #
        launch_ros.actions.Node(
            name="waypoint_navigator_node",
            package="waypoint_navigator",
            executable="waypoint_navigator_node",
            output="screen",
            emulate_tty=True,
            arguments=[LaunchConfiguration("path_config_file"),
                LaunchConfiguration("robot_config_file")],
        )
    ])