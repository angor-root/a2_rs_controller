"""
Launch the Unitree Bridge Node for Simulation

Usage:
  ros2 launch a2_unitree_bridge robot.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_node = Node(
        package='a2_unitree_bridge',
        executable='a2_bridge_robot',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        bridge_node,
    ])
