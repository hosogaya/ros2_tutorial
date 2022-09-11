import launch
import launch_ros
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


# Use if condition in launch file. 
def generate_launch_description():
    use_costom_msg = launch.actions.DeclareLaunchArgument(
        "use_costom_msg", 
        default_value='true',
        description="If set true, launch talker_with_costom_msg"
    )
    # If argument use_costom_msg==true => launch talker_with_custom_message node
    talker_with_costom_msg = launch_ros.actions.Node(
        package="simple_talker",
        namespace='',
        executable='talker_with_custom_message',
        name="talker_with_custom_msg",
        output='screen',
        condition=IfCondition(launch.substitutions.LaunchConfiguration("use_costom_msg")),
    )
    # If argument use_costom_msg!=true => launch talker node
    talker = Node(
        package='simple_talker',
        namespace='',
        executable='talker',
        name='talker',
        output='screen',
        condition= UnlessCondition(launch.substitutions.LaunchConfiguration("use_costom_msg")),
    )
    
    return launch.LaunchDescription([
        use_costom_msg,
        talker_with_costom_msg,
        talker,
    ])