import os
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
    # Publishes TFs like base_footprint -> base_link -> lidar_link
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )


    # 3. FIXED SPAWN LOCATION 
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
            # Spawn Z is now 0.2, which is > 0.1075, so the robot should sit slightly above ground for stability.
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

    # 6. STATIC TF: odom -> base_footprint
    # This node is NOT needed because the Gazebo plugin now publishes odom -> base_footprint.
    # It remains commented out.

    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity,
        rviz_node,
    ])