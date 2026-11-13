# Source this file; it changes only this terminal's environment.
export LITE3_PKG_ROOT="$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo"
export LITE3_CHAPTER_ROOT="$LITE3_PKG_ROOT/6-3_robot_sport"

_lite3_filter_old_paths() {
  local variable="$1" result="" entry
  local -a entries=()
  IFS=':' read -r -a entries <<< "${!variable-}"
  for entry in "${entries[@]}"; do
    case "$entry" in
      ""|"$HOME/catkin_ws_lite3dog/install"|"$HOME/catkin_ws_lite3dog/install/"*|\
      "$LITE3_PKG_ROOT/install"|"$LITE3_PKG_ROOT/install/"*|\
      "$LITE3_PKG_ROOT/.lite3_runtime"|"$LITE3_PKG_ROOT/.lite3_runtime/"*) continue ;;
    esac
    if [ -n "$result" ]; then result="$result:$entry"; else result="$entry"; fi
  done
  printf -v "$variable" '%s' "$result"
  export "$variable"
}
for _lite3_variable in AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH \
                       LD_LIBRARY_PATH PYTHONPATH PATH; do
  _lite3_filter_old_paths "$_lite3_variable"
done
unset _lite3_variable
unset -f _lite3_filter_old_paths

source /opt/ros/humble/setup.bash
if [ "${1-}" = "--build" ]; then
  return 0
fi
if [ ! -f "$LITE3_CHAPTER_ROOT/.runtime/install/local_setup.bash" ]; then
  echo '请先完成 6-3_robot_sport/.runtime 内的独立构建。' >&2
  return 1
fi
source "$LITE3_CHAPTER_ROOT/.runtime/install/local_setup.bash"
export ROS_LOG_DIR="$LITE3_CHAPTER_ROOT/.runtime/ros_log"
mkdir -p "$ROS_LOG_DIR"
