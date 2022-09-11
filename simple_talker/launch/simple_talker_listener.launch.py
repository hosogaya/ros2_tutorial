import launch
from launch_ros.actions import Node

def generate_launch_description():
    simple_listener_node = Node(
        package="simple_listener",
        namespace='',
        executable='listener',
        name='simple_listener',
        output="screen"
    )
    
    simple_talker_node = Node(
        package="simple_talker", 
        namespace = '',
        executable = 'talker',
        name='simple_talker',
        output = "screen",
    )
    
    return launch.LaunchDescription(
        [
            simple_talker_node, 
            simple_listener_node,
        ]
    )