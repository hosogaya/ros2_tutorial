import launch
from launch_ros.actions import Node

def generate_launch_description():
    simple_talker_component = Node(
        package="simple_talker",
        namespace='',
        executable='talker',
        output='screen'
    )
    
    return launch.LaunchDescription([simple_talker_component])