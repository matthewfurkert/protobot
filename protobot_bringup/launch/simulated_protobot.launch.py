from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_protobot_description = get_package_share_directory('protobot_description')
    pkg_protobot_controller = get_package_share_directory('protobot_controller')
    pkg_protobot_moveit = get_package_share_directory('protobot_moveit')
    pkg_protobot_remote = get_package_share_directory('protobot_remote')

    gz_launch_path = PathJoinSubstitution([pkg_protobot_description, 'launch', 'gz.launch.py'])
    controller_launch_path = PathJoinSubstitution([pkg_protobot_controller, 'launch', 'controller.launch.py'])
    moveit_launch_path = PathJoinSubstitution([pkg_protobot_moveit, 'launch', 'moveit.launch.py'])
    remote_launch_path = PathJoinSubstitution([pkg_protobot_remote, 'launch', 'remote_interface.launch.py'])

    

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(remote_launch_path)),
    ])