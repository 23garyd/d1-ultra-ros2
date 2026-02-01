#!/usr/bin/env bash
set -e

# Usage:
#   bash stand_pose.sh
# Optional:
#   bash stand_pose.sh /legs_controller/joint_trajectory

TOPIC="${1:-/legs_controller/joint_trajectory}"

ros2 topic pub --once "$TOPIC" trajectory_msgs/msg/JointTrajectory "{
  joint_names: [FL_hip_joint, FL_thigh_joint, FL_calf_joint, FR_hip_joint, FR_thigh_joint, FR_calf_joint, RL_hip_joint, RL_thigh_joint, RL_calf_joint, RR_hip_joint, RR_thigh_joint, RR_calf_joint],
  points: [
    {
      positions: [0.2, 0.9, -1.6, -0.2, 0.9, -1.6, 0.2, 0.9, -1.6, -0.2, 0.9, -1.6],
      time_from_start: {sec: 1, nanosec: 0}
    }
  ]
}"
echo "Sent stand pose to $TOPIC"
