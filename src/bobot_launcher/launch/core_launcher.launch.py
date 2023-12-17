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

    # -- BOARD OPTIONS -- #
    #   "lattepanda_v1" 
    #   "arduino"
    #   "raspberry_pi"
    device_type_arg = DeclareLaunchArgument(
        "device_type", default_value=TextSubstitution(text="lattepanda_v1")
    )


    # # Save the URDF xacro file path and save it for later
    # urdf_xacro_path = ""
    # robot_description = {
    #         "robot_description": Command([
    #             FindExecutable(name="xacro"),
    #             " ",
    #             urdf_xacro_path,
    #             " "
    #         ])
    #     }

    # Declare the nodes that we wish to run
    bobot_manager_node = Node(
        package="bobot_manager",
        executable="Manager",
        name="BobotManager",
        parameters=[{
            "bobot_name": LaunchConfiguration("bobot_name"),
            "device_type" : LaunchConfiguration("device_type")
        }]
    )

    # Launch the files in the bobot_utils folder (servo jerk, timer, and atmosphere monitor)
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("bobot_utils"), "launch/bobot_utility_launcher.launch.py")
        ),
        launch_arguments=[
            ("bobot_name", LaunchConfiguration("bobot_name")), # pass the bobot_name to the other launch file 
            ("device_type", LaunchConfiguration("device_type"))] # pass the device type to the other launch file
    )


    return LaunchDescription([
        bobot_name_arg,
        device_type_arg,
        bobot_manager_node,
        launch_include
    ])