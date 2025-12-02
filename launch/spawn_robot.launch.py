import os
# Remove 'random' import if you don't need it for other purposes
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

    # 2. Robot State Publisher (RSP)
    # Publishes TFs like base_link -> lidar_link
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. FIXED SPAWN LOCATION 
    # The robot will now always spawn at the Top-Right corner (7.0, 7.0)
    spawn_x = 7.0
    spawn_y = 7.0
    spawn_z = 0.2

    # 4. Spawn Entity in Gazebo Sim
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

    # 6. CRITICAL CORRECTION: REMOVE REDUNDANT TF NODE (Kept for correctness)
    # The required TF chain is: map -> odom -> base_link -> base_footprint

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity,
        rviz_node,
    ])