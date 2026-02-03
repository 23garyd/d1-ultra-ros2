"""
模块2: 生成机器人模型 (需要先运行 test_1_gazebo.launch.py)
测试: 机器人是否在 Gazebo 中生成，ros2_control 是否加载
验证命令:
  ros2 topic list | grep joint  (应该看到 /joint_states)
  ros2 service list | grep controller_manager  (应该看到 controller_manager 服务)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # 加载机器人描述 (robot_state_publisher)
    dog_description_node = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('edu_description'),
                'launch',
                'sim_display_launch.py'
            )
        )
    )
    ld.add_action(dog_description_node)

    # 在 Gazebo 中生成机器人
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'd1_dog',
            '-topic', '/robot_description',
            '-x', '-4',
            '-z', '0.7',
        ],
        output='screen'
    )
    ld.add_action(spawn_robot)

    return ld
