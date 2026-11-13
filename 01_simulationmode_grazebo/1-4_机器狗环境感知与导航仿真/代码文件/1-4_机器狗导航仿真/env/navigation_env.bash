# Source explicitly in each navigation terminal. Nothing is written outside 6-4.
_nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_nav_root/../6-3_robot_sport/env/lite3_env.bash"
export ROS_DOMAIN_ID=30
export ROS_LOG_DIR="$_nav_root/.runtime/ros_log"
export PYTHONDONTWRITEBYTECODE=1
_nav_overlay="$_nav_root/.runtime/slam_root/opt/ros/humble"
if [[ -d "$_nav_overlay/share/ament_index" ]]; then
  export AMENT_PREFIX_PATH="$_nav_overlay${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$_nav_overlay/lib:$_nav_root/.runtime/slam_root/usr/lib/aarch64-linux-gnu:$_nav_root/.runtime/slam_root/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export PYTHONPATH="$_nav_overlay/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
unset _nav_root _nav_overlay
