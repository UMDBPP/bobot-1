from controller_manager_msgs.srv import SwitchController
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

    # Declare what are the possibly cmd_line inputs to this launch file:
    bobot_name_arg = DeclareLaunchArgument(
        "bobot_name", default_value=TextSubstitution(text="bobot1")
    )
    device_type_arg = DeclareLaunchArgument(
        "device_type", default_value=TextSubstitution(text="lattepanda_v1")
    )

    # Declare the nodes thhat we want to run
    bobot_timer_node = Node(
        package="bobot_utils",
        executable="timer",
        name="BobotTimer", # This overrides the name listed in the timer_node file, but that's okay because it's better here (and the same anyway)
        parameters=[{
            "bobot_name" : LaunchConfiguration("bobot_name")
        }]
    )


    hardware_config = PathJoinSubstitution(["src", "bobot_utils", "hardware", LaunchConfiguration("bobot_name"), "hardware.yaml"])
    board_info_config = PathJoinSubstitution(["src", "bobot_utils", "hardware", LaunchConfiguration("bobot_name"), "pinouts.yaml"])

    bobot_servo_jerk_node = Node(
        package="bobot_utils",
        executable="servo_jerk",
        name="ServoJerk", # This overrides the name listed in the timer_node file, but that's okay because it's better here (and the same anyway)
        parameters=[hardware_config, board_info_config, {"bobot_name" : LaunchConfiguration("bobot_name")}]
    )

    return LaunchDescription([
        bobot_name_arg,
        device_type_arg,
        bobot_timer_node,
        bobot_servo_jerk_node
    ])