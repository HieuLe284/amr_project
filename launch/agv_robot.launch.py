import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'agv_robot'
    
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'agv_robot.urdf.xacro'
    )
    
    robot_desc = xacro.process_file(xacro_file).toxml()

    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'view_robot.rviz'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc
            }]
        ),
        Node(
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        )
    ])