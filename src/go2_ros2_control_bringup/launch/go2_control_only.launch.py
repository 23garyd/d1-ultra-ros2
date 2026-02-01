from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_xacro = LaunchConfiguration('urdf_xacro')
    controllers_file = LaunchConfiguration('controllers_file')
    controller_manager = LaunchConfiguration('controller_manager')

    # Provide controller yaml path to the Gazebo ROS2 control plugin via env var
    set_env = SetEnvironmentVariable(name='GO2_CONTROLLERS_YAML', value=controllers_file)

    robot_description = ParameterValue(Command(['xacro', urdf_xacro]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
    )

    # Spawners (wait a bit for Gazebo + gz_ros2_control plugin to bring up controller_manager services)
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', controller_manager],
        output='screen',
    )

    legs_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['legs_controller', '--controller-manager', controller_manager],
        output='screen',
    )

    # Defaults: relative to this launch file (works after you copy these files into your own package)
    default_urdf = PathJoinSubstitution([ThisLaunchFileDir(), '..', 'urdf', 'go2_ros2_control.urdf.xacro'])
    default_ctrl = PathJoinSubstitution([ThisLaunchFileDir(), '..', 'config', 'go2_controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('urdf_xacro', default_value=default_urdf),
        DeclareLaunchArgument('controllers_file', default_value=default_ctrl),
        # If你的 controller_manager 实际在模型命名空间下（例如 /model/go2/controller_manager），启动时把这个参数改掉
        DeclareLaunchArgument('controller_manager', default_value='/controller_manager'),

        set_env,
        rsp,
        TimerAction(period=3.0, actions=[jsb_spawner]),
        TimerAction(period=4.0, actions=[legs_spawner]),
    ])
