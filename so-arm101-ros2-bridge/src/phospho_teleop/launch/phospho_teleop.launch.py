#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('phospho_teleop')
    
    # Launch arguments
    websocket_url_arg = DeclareLaunchArgument(
        'websocket_url',
        default_value='ws://localhost/move/teleop/ws',
        description='Phospho WebSocket URL'
    )
    
    # Joint state reader node (hardware interface)
    joint_state_reader_node = Node(
        package='jointstatereader',
        executable='joint_state_reader.py',
        name='joint_state_reader',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 1000000,
        }]
    )
    
    # TF2 broadcaster node
    tf2_broadcaster_node = Node(
        package='jointstatereader',
        executable='soarm_tf2.py',
        name='soarm_tf2_broadcaster',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM1',
            'baud_rate': 1000000,
        }]
    )
    
    # Phospho teleoperation node
    phospho_teleop_node = Node(
        package='phospho_teleop',
        executable='phospho_teleop_node',
        name='phospho_teleop_node',
        output='screen',
        parameters=[{
            'websocket_url': LaunchConfiguration('websocket_url'),
            'max_linear_velocity': 0.1,
            'max_angular_velocity': 0.5,
            'control_frequency': 50,
        }]
    )
    
    return LaunchDescription([
        websocket_url_arg,
        joint_state_reader_node,
        tf2_broadcaster_node,
        phospho_teleop_node,
    ]) 