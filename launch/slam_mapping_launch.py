import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# Import the required TimerAction
from launch.actions import IncludeLaunchDescription, TimerAction 
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'gazebo' # Your package name
    pkg_share = get_package_share_directory(pkg_name)
    
    # --- 1. Get Config Path ---
    slam_config_path = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml'
    )

    # --- 2. Launch World, Bridge, Robot, RSP, and RViz ---
    world_and_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'world_launch.py')
        )
    )

    robot_and_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
        )
    )

    # --- 3. SLAM Toolbox Node (Delayed) ---
    slam_node_definition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_config_path
        }.items()
    )
    
    # Wrap the SLAM node in a 5-second delay to avoid race conditions
    slam_delay = TimerAction(
        period=5.0,
        actions=[slam_node_definition]
    )


    return LaunchDescription([
        world_and_bridge,
        robot_and_rviz,
        slam_delay, # Use the delayed action
    ])