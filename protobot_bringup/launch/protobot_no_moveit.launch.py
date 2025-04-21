from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_protobot_description = get_package_share_directory('protobot_description')
    pkg_protobot_controller = get_package_share_directory('protobot_controller')

    gz_launch_path = PathJoinSubstitution([pkg_protobot_description, 'launch', 'gz.launch.py'])
    controller_launch_path = PathJoinSubstitution([pkg_protobot_controller, 'launch', 'controller.launch.py'])
    

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch_path)),
        Node(
            package="protobot_kinematics",
            executable="coord_to_joint_angles.py"),
    ])