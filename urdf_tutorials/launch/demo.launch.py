import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # declare argument
    use_sim_time = LaunchConfiguration("use_sim_time", default='false')
    
    # load urdf file
    urdf_file_name = 'r2d2.urdf'
    urdf = os.path.join(
        get_package_share_directory('urdf_tutorial'),
        'urdf', urdf_file_name
    )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    # get path to rviz2
    rviz_path = os.path.join(
        get_package_share_directory('urdf_tutorial'),
        'config/r2d2.rviz'
    )
        
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='urdf_tutorial',
            executable='state_publisher',
            name='state_publisher',
            output='screen'
        ),
        # launch robot_state_publisher and with urdf file
        Node(
            package= 'robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output = 'screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
        ),
        # launch rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_path)]
        )
    ])