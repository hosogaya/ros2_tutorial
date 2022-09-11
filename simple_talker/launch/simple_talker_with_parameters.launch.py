import os

from ament_index_python import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            data =  yaml.safe_load(f)
            return data['/**']['ros__parameters']    
        
    ns = ""
    pkg = "simple_talker"
    
    
    simple_talker_component = Node(
        package=pkg, 
        namespace = '',
        executable = 'talker_with_parameters',
        name='simple_talker',
        output = "screen",
        # remappings=,
        parameters=[load_composable_node_param("simple_talker_path")]
    )
    
    return [simple_talker_component]

# launch talker_with_parameters. The prameters set as simple_talker.param.yaml
def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)
    
    return launch.LaunchDescription(
        [
            add_launch_arg(
                "simple_talker_path", 
                [os.path.join(get_package_share_directory('simple_talker'), "config", "simple_talker.param.yaml")],   
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )