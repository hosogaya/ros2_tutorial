import launch
from launch_ros.actions import Node

# launch talker node
def generate_launch_description():
    simple_talker_component = Node(
        package="simple_talker", # Name of package which the executable is defined.
        name='talker', # Name of launched node
        namespace='', # Namespace of node, topic, service and action. If you set namespace, launched node names become "/<namespace>/<node_name>".
        executable='talker', # Executable name
        output='screen' # Destination of RCLCPP_INFO, RCLCPP_ERROR and so on.
    )
    
    return launch.LaunchDescription([simple_talker_component])