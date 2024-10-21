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

    bobot_launch_config = PathJoinSubstitution(["src", "bobot_bringup", "bobot_launch_config.yaml"])
    bobot_hardware_config = PathJoinSubstitution(["src", "bobot_utils", "bobot_hardware_config.yaml"])

    bobot_servo_jerk_node = Node(
        package="bobot_utils",
        executable="servo_jerk",
        name="servo_jerker", # This overrides the name listed in the timer_node file, but that's okay because it's better here (and the same anyway)
        emulate_tty=True,
        parameters=[bobot_launch_config, bobot_hardware_config]
    )

    return LaunchDescription([
        bobot_servo_jerk_node
    ])