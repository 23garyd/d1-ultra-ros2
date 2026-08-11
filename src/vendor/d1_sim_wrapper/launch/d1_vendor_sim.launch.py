"""Full D1-Max-compatible simulation: Gazebo sim + Nav2 + cartographer + wrapper.

Runs the whole stack under ROS_DOMAIN_ID (default 24 — the domain the real
D1-Max and the cloud task executor use). Launch from the repository root so the
sim's relative IGN_GAZEBO_RESOURCE_PATH resolves:

    export ROS_DOMAIN_ID=24   # or: source env.bash
    ros2 launch d1_sim_wrapper d1_vendor_sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('d1_sim_wrapper'),
                          'config', 'wrapper_params.yaml')
    sim_launch = os.path.join(get_package_share_directory('sim_ign_dog'),
                              'launch', 'd1_gazebo_sim_dog.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('domain', default_value='24'),
        # applies to every process started below (belt-and-braces: set it in
        # your shell too so ros2-cli tools land in the same domain)
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain')),
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(sim_launch)),
        # give Gazebo/controllers/Nav2 time to settle before the wrapper probes them
        TimerAction(period=8.0, actions=[
            Node(
                package='d1_sim_wrapper',
                executable='wrapper_node',
                name='d1_sim_wrapper',
                parameters=[params, {'use_sim_time': True}],
                output='screen',
            ),
        ]),
    ])
