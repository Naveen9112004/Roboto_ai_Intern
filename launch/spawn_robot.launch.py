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

    # 3. Choose random corner
    corners = [
        (7.0, 7.0),
        (-7.0, 7.0),
        (7.0, -7.0),
        (-7.0, -7.0),
    ]
    spawn_x, spawn_y = random.choice(corners)
    spawn_z = 0.2

    # 4. Spawn Entity
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

    # 5. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'one.rviz')]
    )

    # 6. Static transforms (only if you want explicit frames)

    # odom -> base_footprint (helps some Nav2 setups)
    static_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_base',
        arguments=[
            '0', '0', '0',      # x y z
            '0', '0', '0',      # roll pitch yaw
            'odom', 'base_footprint'
        ]
    )

    # base_link -> camera_link_optical (matches URDF but explicit)
    static_camera_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_optical',
        arguments=[
            '0', '0', '0',
            '0', '-1.5708', '0',
            'base_link', 'camera_link_optical'
        ]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity,
        rviz_node,
        static_odom_base,
        static_camera_optical,
    ])
