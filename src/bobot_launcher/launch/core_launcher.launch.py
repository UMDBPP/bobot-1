from controller_manager_msgs.srv import SwitchController
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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
        namespace="bobot",
        executable="Manager",
        name="BobotManagerNode"
    )

    bobot_timer_node = Node(
        package="bobot_manager",
        namespace="bobot",
        executable="Timer",
        name="BobotTimerNode"
    )


    return LaunchDescription([
        bobot_manager_node,
        bobot_timer_node
    ])