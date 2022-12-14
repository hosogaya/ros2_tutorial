import os

from ament_index_python import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def launch_setup(context, *args, **kwargs):
    # Load yaml file
    def load_node_param(param_path):
        # Convert argument to string type object by using LaunchConfiguration(<arg>).
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            data =  yaml.safe_load(f)
            return data['/**']['ros__parameters']    
        
    simple_talker_node = Node(
        package='simple_talker', 
        namespace = '',
        executable = 'talker_with_parameters',
        name='simple_talker_with_paramters',
        output = "screen",
        # remappings=,
        parameters=[load_node_param("param_path")]
    )
    
    return [simple_talker_node]

# Launch talker_with_parameters. The prameters set as simple_talker.param.yaml
def generate_launch_description():
    # add launch arguments
    param_path_arg = DeclareLaunchArgument(
        name="param_path", 
        default_value=[os.path.join(get_package_share_directory('simple_talker'), "config/simple_talker.param.yaml")]
    )
    
    return launch.LaunchDescription(
        [
            # register arguments
            param_path_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )