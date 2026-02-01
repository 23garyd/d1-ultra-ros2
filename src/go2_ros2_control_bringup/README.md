# go2_ros2_control_bringup（四足：先“能控关节”，再走路）

这套文件做了三件事：
1) 修正 URDF 里的 `<ros2_control>`：把真正可动的 12 个关节（FL/FR/RL/RR × hip/thigh/calf）完整声明出来
2) 给出 `controllers.yaml`：joint_state_broadcaster + JointTrajectoryController
3) 给出 `launch`：启动 robot_state_publisher，等待 Gazebo 侧 gz_ros2_control 插件起来后自动加载控制器
4) 给出 `stand_pose.sh`：发一个站立姿态轨迹（用于验证关节真的能响应）

---

## A. 你需要满足的前提
- 已安装 gz_ros2_control（ROS 2 Humble）：
  sudo apt install ros-humble-gz-ros2-control

- 你的仿真模型里必须加载到这段 Gazebo system plugin（已写在 urdf/xacro 里）：
  filename="gz_ros2_control-system"
  name="gz_ros2_control::GazeboSimROS2ControlPlugin"

如果启动时提示找不到 shared library，先用下面命令查你的真实 .so 名字，然后把 filename 改成对应的：
  find /usr/lib -iname "*gz_ros2_control*system*.so" 2>/dev/null

---

## B. 文件放置建议（放进你自己的包）
把本目录内容复制到你自己的包（例如 go2_bringup）中：
- go2_bringup/config/go2_controllers.yaml
- go2_bringup/urdf/go2_ros2_control.urdf.xacro
- go2_bringup/launch/go2_control_only.launch.py
- go2_bringup/scripts/stand_pose.sh

然后 `colcon build` + `source install/setup.bash`

---

## C. 使用步骤（最小可行）
1) 先启动你的 Gazebo/ign 仿真（把机器人 spawn 进去）
2) 再启动控制器加载（本 launch 不负责 spawn）
   ros2 launch go2_bringup go2_control_only.launch.py

3) 看到 controllers 都是 active 后，发站立姿态：
   bash $(ros2 pkg prefix go2_bringup)/share/go2_bringup/scripts/stand_pose.sh

---

## D. 常用排错命令
- 看控制器状态：
  ros2 control list_controllers --controller-manager /controller_manager

- 看关节状态有没有出来：
  ros2 topic echo /joint_states

- 如果 /controller_manager 不存在：
  说明 Gazebo 侧插件没加载成功（优先检查 filename 是否匹配你的系统 .so）
