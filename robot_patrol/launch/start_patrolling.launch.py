import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('robot_patrol'),
        'rviz',
        'patrol_config.rviz')

    patrol_node = Node(
        package='robot_patrol',
        executable='patrol_node', 
        name='robot_patrol_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        patrol_node,
        rviz_node
    ])