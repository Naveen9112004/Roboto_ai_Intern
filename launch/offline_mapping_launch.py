import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'gazebo'   # your package
    pkg_share = get_package_share_directory(pkg_name)

    slam_config_path = os.path.join(
        pkg_share, 'config', 'mapper_params_offline.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',   # <--- existing executable
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path,
            {'use_sim_time': True}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'one.rviz')]
    )

    return LaunchDescription([
        slam_node,
        rviz_node,
    ])
