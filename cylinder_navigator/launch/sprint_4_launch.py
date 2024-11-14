from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, LogInfo, GroupAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot3_model = "waffle_pi"

    # Path to the map file
    map_file = '/home/student/ros2_ws/src/autonomous_robot/map/real_warehouse.yaml'
    if not os.path.exists(map_file):
        print(f"ERROR: Map file {map_file} not found!")

    return LaunchDescription([
        # Set environment variable for the TurtleBot3 model
        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),

        # Log environment variable
        LogInfo(msg=f"TURTLEBOT3_MODEL set to {turtlebot3_model}"),
        
        # Log map file path
        LogInfo(msg=f"Using map file: {map_file}"),

        # Group actions to ensure TURTLEBOT3_MODEL is set before the following commands
        GroupAction([
            # 1. Launch the TurtleBot3 Gazebo world
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_warehouse.launch.py',
                    'use_sim_time:=true'
                ],
                output='screen'
            ),
            
            # 2. Execute localizer_and_navigation with arguments 0.0 0.0 0.0
            ExecuteProcess(
                cmd=['ros2', 'run', 'autonomous_robot', 'localizer_and_navigation', '0.0', '0.0', '0.0'],
                output='screen'
            ),

            # 3. Launch the cylinder_detector node from the cylinder_navigation package
            Node(
                package='cylinder_navigation',
                executable='cylinder_detector',  # Corrected to match CMakeLists.txt
                name='cylinder_detector',
                output='screen'
            ),
            
            # 4. Launch the filter_cylinder node from the cylinder_navigation package
            Node(
                package='cylinder_navigation',
                executable='cylinder_navigation_node',
                name='cylinder_navigation_node',
                output='screen'
            )
        ])
    ])
