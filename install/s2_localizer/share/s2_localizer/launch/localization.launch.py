import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    initial_pose_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='initial_pose_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[str('/home/student/ros2_ws/install/turtlebot3_description/share/turtlebot3_description/urdf/turtlebot3_' + os.getenv('TURTLEBOT3_MODEL', 'burger') + '.urdf.xacro')]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True, 'yaml_filename': '/home/student/ros2_ws/src/tejasWarehouse/updatedWarehouse_map2.yaml'}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl']}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', '/home/student/ros2_ws/src/s2_localizer/rviz_config.rviz']
        ),
        # Delay the initial pose publisher to ensure AMCL is ready
        TimerAction(
            period=5.0,
            actions=[initial_pose_cmd]
        )
    ])
