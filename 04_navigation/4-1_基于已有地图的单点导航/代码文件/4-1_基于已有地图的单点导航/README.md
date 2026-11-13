# 5-1 已有地图定位与单目标导航｜Lite3 实机

在同一现场加载 4-2 保存的 **YAML＋PGM 二维地图（0.05 m/格）**。沿用香橙派上的 MID-360、FAST-LIO、实测安装外参和 `/scan`，由 AMCL 结合地图与里程计估计机器狗的位姿。Nav2 规划和跟踪单个目标，官方 Lite3 ROS `MotionSender` 通过 UDP 向本体发送高层速度。

```text
MID-360 → FAST-LIO → /course/odom 和 odom→base_footprint→base_link
点云 + 安装 TF → /scan；已保存的 YAML＋PGM → map_server → /map
/map + /scan + 里程计 TF → AMCL → map→odom
Nav2 → 速度平滑器 → 官方上层运动桥 → MotionSender → Lite3 本体
```

5-1 复用 4-2 的传感器、Nav2 和官方运控。定位阶段加载 4-2 保存的地图，由 AMCL 负责 `map→odom`。若本次起点和目标都在已保存地图的覆盖区内，直接运行 5-1。若换了场地或地图未覆盖目标区域，先用 4-2 补齐并保存地图。结束在线建图和探索会话后再启动 5-1。

实验保留人工初始化、粒子收敛验证、单点到达、取消停车和证据记录。粒子集中反映 AMCL 的后验分布，绝对位置误差需现场独立测量。

## 1. 各终端和网络

| 终端 | 在哪里 | 用途 |
|---|---|---|
| A | 香橙派本机或 SSH | MID-360、FAST-LIO、map_server、AMCL |
| B | 香橙派本机或 SSH | 网络核对、生成导航配置、检查、使能、发目标、停车 |
| C | **香橙派外接显示器的本机终端** | RViz，设置初始位姿和点击目标 |
| D | 香橙派本机或 SSH | Nav2 和速度平滑器 |
| E | 香橙派本机或 SSH | 官方 MotionSender 和上层运动桥 |
| F | 香橙派本机或 SSH | 原 5-1 粒子收敛和轨迹记录 |

所有命令都在香橙派运行。C 使用本机显示，避免 SSH/X11 转发。若当前有旧导航任务，先从它自己的控制终端停车并确认本体停稳，再依次退出旧 E、探索节点、D、C、A。新会话只启动一套雷达、里程计和导航。复制代码框内的命令，有 `(base)` 的终端先单独执行 `conda deactivate`。

**每个终端**都加载同一份环境。

```bash
LAB42="$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization"
source "$LAB42/official_motion_bridge/env.sh"
```

在 B 确认 `ROS_DOMAIN_ID=43`、`ROS_LOCALHOST_ONLY=1`，以及 5-1 和运动桥已安装。

```bash
printf 'ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s\n' "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY"
ros2 pkg prefix lite3_lab_5_1
ros2 pkg prefix lab42_official_motion
ip -br link show eth0
ip -br -4 addr show eth0
```

`eth0` 有 `LOWER_UP` 且已有 `192.168.1.50/24` 后，在 B 执行。

```bash
sudo ip route replace 192.168.1.3/32 dev eth0 src 192.168.1.50
ip route get 192.168.1.3
ip route get 192.168.2.1
```

| 核对项 | 预期 |
|---|---|
| MID-360 路由 | `192.168.1.3 dev eth0 src 192.168.1.50` |
| 本体路由 | `192.168.2.1 dev wlan0 src 192.168.2.233`，直连 |
| 管理网络地址 | `192.168.1.10` 用于管理 SSH。实机运动前先连本体 Wi-Fi，再启动会话 |

本体路由不符合时，可做到定位观察，运动步骤暂缓。开跑前由现场人员清理前方、两侧与后方的足迹扫掠范围，确认遥控器可以停车。

## 2. A 启动感知和静态地图定位

板端启动脚本已指定以下地图。校验记录显示 `valid=true`，图像为 `146×63` 格，分辨率 `0.05 m/格`。

```text
/home/HwHiAiUser/catkin_ws_lite3dog/results/4_2_realnav_txYDmjiD/maps/robot_dog_auto_map.yaml
/home/HwHiAiUser/catkin_ws_lite3dog/results/4_2_realnav_txYDmjiD/maps/robot_dog_auto_map.pgm
```

`config/params.yaml` 用于设置 ROS 节点参数，里面没有地图的 `image` 字段。让机器狗在实际起点站稳，FAST-LIO 初始化期间保持静止。A 直接执行这一行，无需输入地图路径或外参。

```bash
bash "$HOME/catkin_ws_lite3dog/src/05_navigation/5-1_single_goal_navigation/start_localization.sh"
```

脚本先检查 YAML 和 PGM，再调用现有的 4-2 `localization.launch.py`，沿用 4-2 的实测外参和里程计配置。本章自动选择 `nav2_amcl::OmniMotionModel`，按平移、横移和真实转角描述 Lite3 的里程计变化。`/lab_4_2` 提供本轮目录给 D 的配置生成器。`/map_server` 发布固定地图，`/amcl` 发布 `map→odom`。旧运动出口关闭，后面的 E 使用官方出口。

B 等待 A 完成 FAST-LIO 初始化，在 C 发初始位姿前复制下面的固定命令。AMCL 容差已经填好，无需自行选值。

```bash
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_1_current_run.txt")"
ros2 param get /lab_4_2 result_dir
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic info /map -v
ros2 param get /amcl use_sim_time
ros2 param get /amcl robot_model_type
ros2 param get /lite3_course_continuous_adapter execute
ros2 param get /course_velocity_adapter enable_real_motion
ros2 param set /amcl transform_tolerance 0.30
ros2 param get /amcl transform_tolerance
```

核对本轮目录与 A 一致、两个生命周期节点都为 `active`、`/map` 唯一发布者为 `/map_server`，三个布尔参数均为 `False`，运动模型读回 `nav2_amcl::OmniMotionModel`，最后读回 `0.3`。如果第一条参数查询显示 `Node not found`，等 A 的 `/lab_4_2` 出现，再单独重查 `ros2 param get /lab_4_2 result_dir`。AMCL 会用此参数给 `map→odom` 留出 0.30 秒的前向时间，与运动桥允许的 TF 时间范围配合。雷达和里程计沿用实际测量时间戳。

## 3. F 启动定位记录，C 设置初始位姿

F 加载第 1 节环境后执行，保持运行。脚本每次建立新的 F 记录目录，便于只重启 F 并保留此前记录。

```bash
bash "$HOME/catkin_ws_lite3dog/src/05_navigation/5-1_single_goal_navigation/start_localization_check.sh"
```

F 复用 5-1 实验节点采集真实粒子云、地图和估计轨迹。它收到人工初始位姿后约每 0.5 秒请求一次 AMCL 无运动观测更新，按原有规则检查加权至少 95% 的粒子区域直径 ≤0.30 m，连续保持 3 秒实际 ROS 时间。重新初始化、粒子数据过期或直径超限都会打断这段计时。

在 C 松开初始位姿箭头后，F 会立即提示“已收到初始位姿”，随后约每 2 秒显示粒子直径、连续合格秒数、粒子源年龄，以及地图、里程计、Nav2 等待项。看到“定位已连续收敛”后，再核对同行的“其余感知与导航检查”。若只重启了 F，启动后重新在 C 设置初始位姿，让这次 F 收到新的初始化消息。

F 的 Nav2 生命周期查询超过 3 秒未返回时会重新查询。D 恢复后，F 可以继续更新就绪状态。

C 在外接显示器上执行。

```bash
source "$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization/official_motion_bridge/env.sh"
AUTO="$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization/nav2_autonomy_tools"
bash "$AUTO/run_local_navigation_view.sh"
```

1. 保持 Fixed Frame 为 `map`，确认显示的是本次保存的现场地图。`Live Obstacle Scan` 订阅 `/scan`，使用 `Best Effort`、`Volatile`、`Keep Last`、Depth `1`。
2. 点击工具栏已配置的 **2D Pose Estimate**，它发布到 `/initialpose`。在地图上从机器狗实际站立位置按住鼠标，沿它实际朝向拖出箭头再松开。
3. 核对黄色实时扫描是否与现场墙边对齐，机身足迹是否落在实际位置。F 应即时提示收到位姿，其记录目录的 `raw/events.jsonl` 同时写入 `initial_pose_received`。初始位姿与现场明显不符时，在零速状态重新设置并等收敛。
4. 第一次使用全向模型时，先站稳静止观察约 1 分钟。结合 F 和 `map→base_footprint` 检查估计位置与朝向是否稳定。若机身静止而估计位置仍明显跳动，先保存该轮记录，核对当前扫描是否能匹配这张已保存地图，再进行运动测试。
5. D 启动后，取消勾选 `Local Obstacle Costmap`，勾选新增的 `Global Planner Costmap`。它显示 `/global_costmap/costmap`，用于检查全局规划的起点和路径是否被静态地图障碍及膨胀区阻断。只开启其中一层代价地图，观察结束后可切回局部层。F 的粒子收敛说明估计集中，不保证估计位置正确，也不保证全局地图允许从该处起步。

## 4. B 准备导航，D 启动 Nav2

确认 C 已设置初始位姿，B 能查询到 `map→base_footprint`，再生成本轮独立导航目录。以下用 4-2 已配置的 **0.20 m/s 前进上限、0.20 rad/s 转向上限**。控制器会按弯道、净空和接近目标的情况减速。

```bash
timeout 8s ros2 run tf2_ros tf2_echo map base_footprint
printf 'length=%s width=%s padding=%s\n' \
  "$ROBOT_LENGTH" "$ROBOT_WIDTH" "$FOOTPRINT_PADDING"
ros2 run lab42_official_motion prepare_nav.py \
  --length "$ROBOT_LENGTH" --width "$ROBOT_WIDTH" --padding "$FOOTPRINT_PADDING" \
  --max-forward-mps 0.20 --max-yaw-radps 0.20
```

本机实测参数为长 `0.779 m`、宽 `0.480 m`、四周余量 `0.10 m`。核对输出目录位于 A 的本轮 `5_1_official_...` 目录内。D 加载环境后运行以下整段。

```bash
(
  set -e
  RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_1_current_run.txt")"
  NAV_RUN="$(cat "$UPPER/current_run.txt")"
  if [[ "$(dirname "$NAV_RUN")" != "$RUN" || ! -f "$NAV_RUN/navigation.launch.py" ]]; then
    echo '本轮导航配置缺失，请先在 B 成功执行 prepare_nav.py。' >&2
    exit 1
  fi
  exec ros2 launch "$NAV_RUN/navigation.launch.py"
)
```

## 5. B 检查收敛、扫描和足迹

C 和 F 保持运行，以准备运动时的负载做检查。即使换了 B 终端，也可以直接复制下面整段。它会重新读取本轮目录，并确认 F 的日志位于该目录下。

```bash
(
  set -eo pipefail
  LAB42="$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization"
  source "$LAB42/official_motion_bridge/env.sh"
  RESULTS="$HOME/catkin_ws_lite3dog/results"
  RUN="$(cat "$RESULTS/5_1_current_run.txt")"
  if [[ "$RUN" != "$RESULTS"/5_1_official_* || ! -d "$RUN" ]]; then
    echo '本轮结果目录无效，请核对 A 的启动输出。' >&2
    exit 1
  fi
  AUDIT_DIR="$RUN/localization_audit"
  if [[ -f "$RUN/localization_audit_current.txt" ]]; then
    AUDIT_DIR="$(cat "$RUN/localization_audit_current.txt")"
  fi
  if [[ "$AUDIT_DIR" != "$RUN"/localization_audit* || ! -d "$AUDIT_DIR" ]]; then
    echo '本轮 F 记录目录无效，请核对 F 的启动输出。' >&2
    exit 1
  fi
  EVENTS="$AUDIT_DIR/raw/events.jsonl"
  if [[ ! -s "$EVENTS" ]]; then
    printf '本轮 F 日志缺失：%s\n请先按第 3 节启动 F。\n' "$EVENTS" >&2
    exit 1
  fi
  printf '本轮结果目录：%s\n' "$RUN"
  grep -m 1 '"event": "initial_pose_received"' "$EVENTS" || {
    echo 'F 尚未收到初始位姿，请保持静止并在 C 设置 2D Pose Estimate。' >&2
    exit 1
  }
  grep '"event": "health"' "$EVENTS" | tail -n 3
  python3 -B "$LAB42/fastlio_latency_fix/verify_live_sensor_chain.py" \
    --output "$RUN/sensor_chain.json"
  python3 -B "$UPPER/collision_snapshot.py" --output "$RUN/collision.json"
  python3 -B "$UPPER/collision_snapshot.py" --costmap global \
    --output "$RUN/global_collision.json"
  ros2 param get /controller_server FollowPath.desired_linear_vel
  ros2 param get /controller_server FollowPath.rotate_to_heading_angular_vel
  ros2 param get /velocity_smoother max_velocity
)
```

| 数据 | 核对条件 |
|---|---|
| F 最新 `health` | 日志时间持续推进，`map.healthy=true`，`particle_region.diameter_m ≤ 0.30`，`lifecycle_active` 全为 `true` |
| F 的 `waiting_for` | 已无初始位姿、粒子未连续收敛、地图、里程计、TF、Nav2 生命周期或 Action 未就绪项。F 仅做观察，其旧原生运动未启用、旧后端未认证两项属于旧出口状态 |
| `sensor_chain.json` | `passed=true`，`query_errors={}`，只有 `/course_sensor_adapter` 发布里程计，固定安装 TF 只有 `/course_sensor_mounts` 发布 |
| `collision.json` 和 C | `footprint_lethal_cells=[]`、`current_pose_footprint_lethal_cells=[]`，无 `cloud_error`。实时扫描、地图与现场一致，目标和整条路径有足迹净空 |
| `global_collision.json` | `costmap_scope=global`、`grid_frame=map`。核对 `planner_start.center_cell.cost` 和 `passable_neighbor_count`；253 为内切障碍膨胀区，254 为障碍，255 为未知。中心代价低于 253 且有可通行邻格只说明起点未被包围，仍须核对完整路径和足迹 |
| D 的限速 | 线速 `0.20`、转向 `0.20`、平滑器 `[0.2, 0.0, 0.2]` |

若 C 上看见路径却报 `detected collision ahead`，先查局部代价地图里足迹及转向扫掠范围附近的回波，再核对现场物体。当前足迹快照为空只说明拍摄时的当前位置。

若目标立即返回 `action_status=6`，且 `nav2_raw`、`smoothed` 都没有非零输出，查看 D 是否报 `GridBased: failed to create plan`。2026-11-11 的 `5_1_official_pt8lRav7` 三次目标已用原地图和原足迹在隔离 ROS 域重现失败，三个起点的全局代价均为 253。第二次起点距地图障碍约 0.194 m，配置的足迹内切半径为 0.340 m，周围八格也全部为 253。同一规划器换用地图中开阔的假设起点能正常规划。这是起点与地图障碍冲突的证据，不能用粒子收敛或局部足迹快照为空来排除。

恢复这类失败时，先在 E 已停止的状态下核对实际墙角、机身中心和朝向，在 C 重新设置 `2D Pose Estimate`，使实时墙边扫描与保存地图对齐，再重新采集全局快照。已有 A、D、F 可继续运行，C 需重开一次才能加载新增显示层。若确认现场周围空旷，但对齐后足迹仍与保存地图障碍冲突，说明旧地图中的障碍与当前现场不一致；此时先回到 4-2 更新并保存现场地图，再在 5-1 加载新地图。不能凭空移动地图中的估计位置来换取可规划起点。

D 偶尔记录一帧激光因 TF 被丢弃时，核对 `sensor_chain.json` 和 F 的最新健康记录。若 D 持续丢帧，或 E 报 `odom源时间戳过期`，保持停车和 E 的 `DISABLED` 状态，在 B 加载第 1 节环境后执行以下只读采样。

```bash
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_1_current_run.txt")"
test -n "$RUN" && test -d "$RUN" && \
  python3 -B "$LAB42/fastlio_latency_fix/check_perception.py" \
    --seconds 20 --output "$RUN/perception_timing.json"
```

用 `/Odometry`、`/course/odom`、`/scan` 的源年龄和 `same_stamp_adapter_delay` 区分上游处理延迟与适配转发延迟。若 F 的 `waiting_for` 仍有里程计过期或粒子未连续收敛，或者 `sensor_chain.json` 未通过，先处理这些情况，再进入使能。

## 6. E 启动官方运动桥，B 使能并发目标

到现场复核本体站稳、遥控器可停车、场地清空、本体路由直连，并通过第 5 节检查后，在 E 执行，保持运行。

```bash
ros2 launch lab42_official_motion control.launch.py \
  execute:=true target_ip:=192.168.2.1 max_forward_mps:=0.20 max_yaw_radps:=0.20 \
  enable_odom_max_age_s:=1.0 enable_scan_max_age_s:=1.0
```

B 先核对状态，再请求使能。

```bash
ros2 topic echo /lab42/upper/status --once --field data
ros2 service call /lab42/upper/enable std_srvs/srv/Trigger '{}'
ros2 topic echo /lab42/upper/status --once --field data
```

确认服务 `success=True`，状态为 `mode=ENABLED`、`reason=ENABLED_WAIT_GOAL`、`ready=true`，等待阶段速度均为零。若看到 `WAITING_FOR_FRESH_DATA`，保持零速，等恢复到上述状态。`stream_timing` 中 `/scan` 和 `/course/odom` 的源年龄与接收年龄可用于判断哪条流滞后。`hardware_mode_acknowledged=null` 时由现场确认本体状态。

`enable_odom_max_age_s:=1.0` 用于接受零速使能请求。若里程计源年龄在 `0.50–1.00 s`，且其余检查通过，E 会接受请求并零速重检。等里程计源年龄恢复到 `0.50 s` 以内、感知连续新鲜 `0.30 s` 后，才进入 `ENABLED_WAIT_GOAL`。导航中使用 `0.50 s` 的里程计源年龄上限。F 的定位收敛结果仍按第 5 节核对。

`enable_scan_max_age_s:=1.0` 同样用于使能前的零速等待，扫描的源年龄和接收年龄均允许到 `1.00 s`。如果扫描已存在且两个年龄都在该范围内，可以接受请求，显示 `WAITING_FOR_FRESH_DATA`。扫描和里程计恢复新鲜、连续通过检查后才进入 `ENABLED_WAIT_GOAL`。导航中使用 `0.50 s` 的扫描源年龄和接收年龄上限。

**第一轮验证取消与停车**。在 B 启动客户端，看到提示后才在 C 点 **Publish Point**，选择正前方约 2 米、已扫描且现场确认空闲的位置。该轮按目标提交后的计时约 5 秒发取消。

```bash
ros2 run lab42_official_motion goal_client.py --cancel-after 5
```

核对结果 `action_status=5`、`cancel_accepted=true`、`stop_request_accepted=true`，再现场确认本体停稳。客户端保存本轮 JSON 和速度链记录。`Publish Point` 选的是目标位置，当前客户端沿用发目标时的机身估计朝向作为终点朝向。

## 7. 第二轮及以后

每轮客户端提交过目标并结束后会请求停车并交还。只要 A、C、D、E、F 仍健康，可保留所有终端，用 B 重复以下步骤。

1. 查看 `/lab42/upper/status`，确认上一轮停稳、速度为零、`mode=DISABLED`。若还在动或停车响应没有确认，先调用本节的停车服务并现场确认。
2. 看 C 的机身位置、朝向和最新扫描。核对 F 最新 `health` 仍已连续收敛。如定位偏离，停稳后用 C 重新设置初始位姿，等 F 恢复。
3. 再次调用 `/enable` 并确认 `ENABLED_WAIT_GOAL`、`ready=true`。一次只运行一个目标客户端，看到提示后重新点新的空闲位置。

```bash
ros2 service call /lab42/upper/enable std_srvs/srv/Trigger '{}'
ros2 topic echo /lab42/upper/status --once --field data
```

**到点轮**在上述状态确认后单独执行。C 用 **Publish Point** 点击新目标。

```bash
ros2 run lab42_official_motion goal_client.py
```

核对 `action_status=4`、`nav2_success=true` 和 `stop_request_accepted=true`。同时看现场实际位移、`motion_evidence.velocity_chain` 和 `odometry_net_displacement_m`。目标刚好落在到达容差内时，Nav2 可能很快返回成功。路径线是计划，里程计是估计，均需和现场运动交叉核对。

需要中止当前目标时，在目标客户端按 `Ctrl+C` 请求取消与停车。现场有风险时直接用遥控停车，也可在另一个已加载环境的板端终端调用下方的停止服务。服务响应后核对实际停车和 Nav2 动作终态，再开始下一轮。

```bash
ros2 service call /lab42/upper/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /lab42/upper/status --once --field data
```

一次 `DISABLED` 通常来自上一轮正常停车，重新使能即可。持续 `WAITING_FOR_FRESH_DATA` 时先看 `health` 指出的数据流；若某个进程确实退出，先停稳，再按对应启动步骤恢复并重新验收。重新运行了 A 就需要重新走本轮目录、AMCL 初始化和 D 配置步骤。

## 8. 保存证据与退出

- **定位证据**在 F 启动时打印的 `localization_audit_...` 目录内，查看 `raw/events.jsonl` 和 `csv/estimated_trajectory.csv`。本次目录也记录在 `$RUN/localization_audit_current.txt`。保存一张初始位姿和实时扫描对齐的 RViz 截图。
- **导航结果**在本轮 `official_upper_.../upper_goal_*.json` 和对应 `.trace.jsonl`。记录每个目标、终态、实际是否移动、停车情况。5-1 原实验节点只记录定位，单目标结果以官方客户端为准。
- **地图及配置**由 `$RUN/map_file.txt`、`sensor_chain.json`、`collision.json` 和导航目录的 `setup.json` 记录。保留地图 YAML 与其引用的图像。

正式验收保留至少 3 次独立运行，包含失败记录。把粒子直径、实际到点距离和是否停车分别记录。有独立位置测量时再填写实测定位误差。

需要录包时，可在另一个板端终端加载第 1 节环境后执行。先确认录包负载下第 5 节仍能通过。

```bash
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_1_current_run.txt")"
ros2 bag record -o "$RUN/bag_$(date +%Y%m%d_%H%M%S)" \
  /map /scan /course/odom /initialpose /amcl_pose /particle_cloud /plan /tf /tf_static \
  /lab42/upper/status /lab42/upper/raw_cmd_vel /lab42/upper/smoothed_cmd_vel \
  /navigate_to_pose/_action/status /local_costmap/costmap /local_costmap/published_footprint
```

结束时在 B 请求停车并确认本体停稳，再按 **录包终端 → E → D → C → F → A** 依次 `Ctrl+C`。录包完成后可用 `ros2 bag info` 检查记录目录。
