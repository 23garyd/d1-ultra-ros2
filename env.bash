# D1-Max-compatible environment for the ign_robot_dog sim + vendor wrapper.
# Usage: source env.bash   (from the repository root, in every terminal)
export ROS_DOMAIN_ID=24   # domain used by the real D1-Max and the cloud executor
source /opt/ros/humble/setup.bash
if [ -f "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash" ]; then
  source "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash"
fi
# after switching domains once: ros2 daemon stop   (CLI caches discovery per domain)
