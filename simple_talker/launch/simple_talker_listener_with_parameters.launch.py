import os

from ament_index_python import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

def launch_setup(context, *args, **kwargs):
    # Load yaml file
    def load_composable_node_param(param_path):
        # Convert argument to string type object by using LaunchConfiguration(<arg>).
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            data =  yaml.safe_load(f)
            return data['/**']['ros__parameters']    
        
    ns = ""
    pkg = "simple_talker"
    
    simple_talker_component = Node(
        package=pkg, 
        namespace = '',
        executable = 'talker_with_parameters',
        name='simple_talker_with_paramters',
        output = "screen",
        # remappings=,
        parameters=[load_composable_node_param("simple_talker_path")]
    )
    
    
    return [simple_talker_component]

# Launch talker_with_parameters. The prameters set as simple_talker.param.yaml
def generate_launch_description():
    # add launch arguments
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)
    
    return launch.LaunchDescription(
        [
            # register arguments
            add_launch_arg(
                "simple_talker_path", 
                [os.path.join(get_package_share_directory('simple_talker'), "config", "simple_talker.param.yaml")],   
            ),
            # Launch simple_listener_with_custom_msg node from other launch file
            launch.actions.IncludeLaunchDescription(
               PythonLaunchDescriptionSource([get_package_share_directory("simple_listener")+"/launch/simple_listener_with_custom_msg.launch.py"])
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )