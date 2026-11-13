#!/usr/bin/env bash
set -e

LAB42="$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization"
source "$LAB42/official_motion_bridge/env.sh"
RESULTS="$HOME/catkin_ws_lite3dog/results"
RUN="$(cat "$RESULTS/5_1_current_run.txt")"
if [[ "$RUN" != "$RESULTS"/5_1_official_* || ! -d "$RUN" ]]; then
  echo '本轮结果目录无效，请先核对 A 的启动输出。' >&2
  exit 1
fi
MAP_FILE="$(cat "$RUN/map_file.txt")"
test -f "$MAP_FILE"
AUDIT_DIR="$(mktemp -d "$RUN/localization_audit_XXXXXXXX")"
POINTER_TMP="$(mktemp "$RUN/.localization_audit_current_XXXXXXXX")"
printf '%s\n' "$AUDIT_DIR" > "$POINTER_TMP"
mv -f -- "$POINTER_TMP" "$RUN/localization_audit_current.txt"
printf '本轮 F 记录目录 %s\nF 启动后在 C 设置初始位姿。\n' "$AUDIT_DIR"
exec ros2 launch lite3_lab_5_1 experiment.launch.py \
  mode:=real start_runtime:=false start_hardware:=false autostart:=false \
  native_execute:=false enable_real_motion:=false rviz:=false \
  robot_length_m:="$ROBOT_LENGTH" robot_width_m:="$ROBOT_WIDTH" \
  footprint_padding_m:="$FOOTPRINT_PADDING" \
  map:="$MAP_FILE" result_dir:="$AUDIT_DIR"
