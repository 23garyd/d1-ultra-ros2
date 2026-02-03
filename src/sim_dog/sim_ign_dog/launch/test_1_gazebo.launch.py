"""
模块1: 只启动 Gazebo 仿真环境
测试: Gazebo 是否正常启动，时钟是否发布
验证命令:
  ign topic -l | grep clock
  ign topic -e -t /world/empty/clock  (应该有消息)
  ros2 topic hz /clock  (桥接后应该有频率)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')
    this_package_path = get_package_share_directory('sim_ign_dog')

    # 启动 Gazebo 仿真环境 (-r 表示自动运行)
    gazebo_node = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f"-r {os.path.join(this_package_path,'world','house_add.sdf')}"
        }.items()
    )
    ld.add_action(gazebo_node)

    # 桥接时钟话题
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/world/empty/clock', '/clock'),
        ],
        output='screen'
    )
    ld.add_action(clock_bridge)

    return ld
