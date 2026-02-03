"""
模块3: 启动控制器 (需要先运行 test_1 和 test_2)
测试: 控制器是否能正常激活
验证命令:
  ros2 control list_controllers  (应该看到 active 状态的控制器)
  ros2 topic echo /joint_states  (应该有关节状态)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('controller_manager_timeout', default_value='30.0'))
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout')

    # Joint State Broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='jsb_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        output='screen',
    )
    ld.add_action(jsb_spawner)

    return ld
