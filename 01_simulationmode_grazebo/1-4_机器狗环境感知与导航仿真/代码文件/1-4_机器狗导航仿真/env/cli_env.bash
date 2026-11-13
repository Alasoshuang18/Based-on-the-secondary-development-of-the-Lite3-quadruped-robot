# Source in 6-4 ROS client terminals when default Fast DDS node creation hangs.
# Existing processes and global shell files are unaffected.
export PATH="/usr/bin:$PATH"
_lite3_cli_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_lite3_cli_root/env/navigation_env.bash" || return 1
if [[ ! -r "$_lite3_cli_root/config/dds_udp_only.xml" ]]; then
  echo 'Missing 6-4 config/dds_udp_only.xml' >&2
  unset _lite3_cli_root
  return 1
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$_lite3_cli_root/config/dds_udp_only.xml"
export PYTHONUNBUFFERED=1
unset _lite3_cli_root
