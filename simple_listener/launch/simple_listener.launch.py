import launch
from launch_ros.actions import Node

def generate_launch_description():
    simple_listener_node = Node(
        package="simple_listener",
        namespace='',
        executable='listener',
        output='screen',
    )
    
    return launch.LaunchDescription([simple_listener_node])