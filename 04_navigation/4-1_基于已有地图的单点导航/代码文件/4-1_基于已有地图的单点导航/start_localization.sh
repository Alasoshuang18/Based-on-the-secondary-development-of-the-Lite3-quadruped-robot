#!/usr/bin/env bash
set -e

WS="$HOME/catkin_ws_lite3dog"
LAB42="$WS/src/04_slam/4-2_mapping_and_relocalization"
MAP_FILE="$WS/results/4_2_realnav_txYDmjiD/maps/robot_dog_auto_map.yaml"

if [[ $# -gt 1 || ( $# -eq 1 && $1 != --check ) ]]; then
  printf '用法：bash %s [--check]\n' "$0" >&2
  exit 2
fi

source "$LAB42/official_motion_bridge/env.sh"
test -f "$LIDAR_CONFIG"
: "${LIDAR_XYZ:?}" "${LIDAR_RPY:?}" "${BODY_HEIGHT:?}"
: "${ROBOT_LENGTH:?}" "${ROBOT_WIDTH:?}" "${FOOTPRINT_PADDING:?}"

python3 -B - "$MAP_FILE" <<'PY'
from pathlib import Path
import math
import sys
import yaml

path = Path(sys.argv[1])
try:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or not {'image', 'resolution', 'origin'} <= data.keys():
        raise ValueError('缺少 image、resolution 或 origin；需要 4-2 保存的地图 YAML')
    image = path.parent / data['image']
    if not image.is_file() or image.stat().st_size == 0:
        raise ValueError(f'地图图像缺失或为空：{image}')
    if not math.isclose(float(data['resolution']), 0.05, abs_tol=1e-6):
        raise ValueError('地图分辨率须为 0.05 m/格')
    origin = data['origin']
    if len(origin) != 3 or not all(math.isfinite(float(value)) for value in origin):
        raise ValueError('地图 origin 须为有限的 [x, y, yaw]')
except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
    raise SystemExit(f'地图检查失败：{path}\n{error}')
print(f'本轮静态地图：{path}\n地图图像：{image}')
PY

args=(mode:=real start_runtime:=true start_hardware:=true lab42_latest_odometry:=true
      amcl_motion_model:=nav2_amcl::OmniMotionModel
      lidar_config:="$LIDAR_CONFIG" lidar_xyz:="$LIDAR_XYZ" lidar_rpy:="$LIDAR_RPY"
      initial_body_height:="$BODY_HEIGHT" robot_length_m:="$ROBOT_LENGTH"
      robot_width_m:="$ROBOT_WIDTH" footprint_padding_m:="$FOOTPRINT_PADDING"
      native_execute:=false enable_real_motion:=false autostart:=false rviz:=false
      map:="$MAP_FILE")

if [[ ${1:-} == --check ]]; then
  exec ros2 launch lite3_lab_4_2 localization.launch.py "${args[@]}" --show-args
fi

RESULTS="$WS/results"
mkdir -p "$RESULTS"
RUN="$(mktemp -d "$RESULTS/5_1_official_XXXXXXXX")"
printf '%s\n' "$MAP_FILE" > "$RUN/map_file.txt"
printf '%s\n' "$RUN" > "$RESULTS/5_1_current_run.txt"
printf '本轮目录：%s\n' "$RUN"
exec ros2 launch lite3_lab_4_2 localization.launch.py "${args[@]}" result_dir:="$RUN"
