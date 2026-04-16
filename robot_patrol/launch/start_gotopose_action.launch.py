import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('robot_patrol'),
        'rviz',
        'patrol_config.rviz')

    return LaunchDescription([
        # Action Server Node
        Node(
            package='robot_patrol',
            executable='go_to_pose_action_node',
            name='go_to_pose_node',
            output='screen'
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        )
    ])