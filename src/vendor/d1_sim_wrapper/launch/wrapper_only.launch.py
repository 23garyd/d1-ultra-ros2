"""Launch only the D1-Max vendor-interface wrapper node.

Use when the sim (d1_gazebo_sim_dog.launch.py) is already running in the same
ROS domain. See d1_vendor_sim.launch.py for the all-in-one entry point.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('d1_sim_wrapper'),
                          'config', 'wrapper_params.yaml')
    return LaunchDescription([
        Node(
            package='d1_sim_wrapper',
            executable='wrapper_node',
            name='d1_sim_wrapper',
            parameters=[params, {'use_sim_time': True}],
            output='screen',
        ),
    ])
