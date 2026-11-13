# Single-board Gazebo Transport profile. Source consistently in A, B, C and ign CLI.
# ROS DDS remains separate; this does not modify existing processes or networking.
_lite3_gz_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_lite3_gz_root/env/cli_env.bash" || return 1
export IGN_IP=127.0.0.1
unset _lite3_gz_root
