import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'gazebo'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Process URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'dyno_atman.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # 2. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. Choose random corner (adjust values to your world size)
    # Example for 20x20 world with walls at ±9.8: corners near ±7, ±7
    corners = [
        (7.0, 7.0),    # top-right
        (-7.0, 7.0),   # top-left
        (7.0, -7.0),   # bottom-right
        (-7.0, -7.0),  # bottom-left
    ]
    spawn_x, spawn_y = random.choice(corners)
    spawn_z = 0.2   # small height above ground

    # 4. Spawn Entity with random position
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-x', str(spawn_x),
            '-y', str(spawn_y),
            '-z', str(spawn_z),
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'view_robot.rviz')]
    )


    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity
    ])
