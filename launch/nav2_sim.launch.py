import os
import random
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# ADDED TimerAction here
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, GroupAction 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml 

# The robot's initial pose is randomized, so we declare variables here
SPAWN_X, SPAWN_Y = random.choice([(7.0, 7.0), (-7.0, 7.0), (7.0, -7.0), (-7.0, -7.0)])
SPAWN_Z = 0.2

def generate_launch_description():
    pkg_name = 'gazebo'
    pkg_share = get_package_share_directory(pkg_name)
    
    # --- 0. Configuration Paths ---
    nav2_config_file = os.path.join(pkg_share, 'config', 'nav2.yaml')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'sample_world.yaml') 

    # --- 1. Environment Setup (Asset Paths) ---
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=f"{pkg_share}/..")
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=f"{pkg_share}/..")

    # --- 2. Launch Gazebo World & Bridge (No changes here) ---
    world_file = os.path.join(pkg_share, 'worlds', 'sample.sdf')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r -g {world_file}'}.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU', 
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen'
    )
    
    # --- 3. Robot Spawning (RSP + Spawner) ---
    xacro_file = os.path.join(pkg_share, 'urdf', 'dyno_atman.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )
    
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot',
                   # Use the global variables defined outside the function
                   '-x', str(SPAWN_X), '-y', str(SPAWN_Y), '-z', str(SPAWN_Z)],
        output='screen'
    )
    
    # --- 4. RViz2 Node (Starts early for visualization) ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'one.rviz')]
    )

    # --- 5. Nav2 Stack (Delayed Start) ---
    
    # EKF Localization Node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
    )

    # Nav2 Bringup parameters (Including initial pose)
    configured_params = RewrittenYaml(
        source_file=nav2_config_file,  
        param_rewrites={
            'map_server.ros__parameters.yaml_filename': map_file,
            # Pass the robot's spawn position to AMCL for a faster start
            'amcl.ros__parameters.initial_pose.x': str(SPAWN_X), 
            'amcl.ros__parameters.initial_pose.y': str(SPAWN_Y)
        }, 
    )
    
    # Nav2 Launch
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')
        ),
        launch_arguments={'map': map_file, 'use_sim_time': 'True',
                          'params_file': configured_params}.items(),
    )
    
    # Group the heavy nodes together
    nav_group = GroupAction([
        robot_localization_node,
        nav2_bringup
    ])
    
    # Delay the Nav2 stack by 8 seconds to allow Gazebo to fully load and clock to stabilize
    nav_delay = TimerAction(period=8.0, actions=[nav_group])

    # --- Final Launch Description ---
    return LaunchDescription([
        # Phase 1: Environment and Core Robot
        ign_resource_path,
        gz_resource_path,
        gazebo_launch,
        bridge,
        node_robot_state_publisher,
        spawn_entity,
        rviz_node,
        
        # Phase 2: Delayed Nav2 and Localization
        nav_delay 
    ])