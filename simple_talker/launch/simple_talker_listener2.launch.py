import os

from ament_index_python import get_package_share_directory

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource

# call simple_talker.launch.py and simple_listener.launch.py
def generate_launch_description():
    simple_talker_dir = get_package_share_directory("simple_talker")
    simple_listener_dir = get_package_share_directory("simple_listener")
    
    simple_talker_description = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([simple_talker_dir+"/launch/simple_talker.launch.py"]),
    )
    simple_listener_description = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([simple_listener_dir+"/launch/simple_listener.launch.py"])
    )
        
    return launch.LaunchDescription([
        simple_talker_description,
        simple_listener_description,
    ])