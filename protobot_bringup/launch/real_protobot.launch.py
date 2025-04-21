from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_arduinobot_controller = get_package_share_directory('arduinobot_controller')
    pkg_arduinobot_remote = get_package_share_directory('arduinobot_remote')

    controller_launch_path = PathJoinSubstitution([pkg_arduinobot_controller, 'launch', 'controller.launch.py'])
    remote_launch_path = PathJoinSubstitution([pkg_arduinobot_remote, 'launch', 'remote_interface.launch.py'])

    

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch_path),
            launch_arguments={'is_sim': 'False'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(remote_launch_path)),
    ])