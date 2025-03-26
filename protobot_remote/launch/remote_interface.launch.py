import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

   
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )
    is_sim = LaunchConfiguration("is_sim")

    use_python_arg = DeclareLaunchArgument(
        "use_python_interface",
        default_value="False"
    )
    use_python = LaunchConfiguration("use_python_interface")

    moveit_config = (
        MoveItConfigsBuilder("protobot", package_name="protobot_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("protobot_description"),
            "urdf",
            "protobot.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/protobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()
    )

    task_server_node_py = Node(
        package="protobot_remote",
        executable="task_server.py",
        condition=IfCondition(use_python),
        parameters=[moveit_config.to_dict(), 
                    {"use_sim_time": is_sim}]
    )
    task_server_node = Node(
        package="protobot_remote",
        executable="task_server_node",
        condition=UnlessCondition(use_python),
        parameters=[moveit_config.to_dict(), 
                    {"use_sim_time": is_sim}]
    )

    return LaunchDescription(
        [
            is_sim_arg,
            use_python_arg,
            task_server_node_py,
            task_server_node,
        ]
    )