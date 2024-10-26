from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    # Below is a HARDCODED path to the file type. I am okay with this - romeo.
    manager_parameters = PathJoinSubstitution(["src/bobot_bringup/bobot_launch_config.yaml"])
    # Declare the nodes that we wish to run
    bobot_manager_node = Node(
        package="bobot_manager",
        executable="manager",
        name="bobot_manager",
        emulate_tty=True,
        parameters=[manager_parameters]
    )

    # Get the bobot utilities launch file
    bobot_utils_dir = get_package_share_directory('bobot_utils')
    bobot_utils_launcher = IncludeLaunchDescription(PythonLaunchDescriptionSource(bobot_utils_dir + '/launch/bobot_utility_launcher.launch.py'))

    # Get the bobot control launch file
    bobot_control_dir = get_package_share_directory('bobot_control')
    bobot_control_launcher = IncludeLaunchDescription(PythonLaunchDescriptionSource(bobot_control_dir + '/launch/bobot_control.launch.py'))

    return LaunchDescription([
        bobot_manager_node,
        bobot_utils_launcher,
        bobot_control_launcher
    ])