# 5-2 RViz 手选目标、自动导航与在线建图｜Lite3 实机

本章从空地图开始，由操作者在 RViz 选择目标，Nav2 自动规划和行走，slam_toolbox 持续建图。默认不再自动选探索目标。每次 A 建立独立建图目录，每次 F 记录一个目标，保留旧地图和失败证据。

## 共用环境

所有 Linux 命令在香橙派执行。每个新终端先退出 conda 的 `(base)`，再加载：

```bash
source "$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization/official_motion_bridge/env.sh"
export AUTO="$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization/nav2_autonomy_tools"
source "$AUTO/exploration_ws/install/local_setup.bash"
export STAGE=2
```

ROS 2 Humble，域号43，`ROS_LOCALHOST_ONLY=1`，真实系统时间，`use_sim_time=false`。里程计来自 MID-360＋FAST-LIO；原始传感器时间戳和既有 TF2 修复库保留。控制链沿用 5-1：

```text
MID-360 → FAST-LIO → /course/odom、odom→base_footprint→base_link
/scan → slam_toolbox → /map、map→odom
RViz手选目标 → F检查 → NavigateToPose → Nav2 → 速度平滑器
→ 官方上层运动桥 → MotionSender → 本体
```

Nav2请求上限 `0.10 m/s、0.10 rad/s`，官方桥和实测审核上限 `0.20 m/s、0.20 rad/s`。机身包络 `0.779×0.480 m`，padding `0.10 m`，须覆盖当前足腿和负载。雷达 xyz `[0.210,0,0.161308]`、RPY `[0,0.338594,0]`，初始高度 `0.325504965 m`。雷达接口 `eth0 192.168.1.50/24`，雷达 `192.168.1.3`；本体路由应为 `192.168.2.1 dev wlan0 src 192.168.2.233`。管理 SSH `.10` 可达不代表本体路由接通。

A 保留感知、TF、SLAM和存图，移除旧 `velocity_adapter`、`continuous_adapter` 及无用旧实验计算。`/lab_4_2` 改为轻量目录登记与延迟记录，供原导航生成器、保存器查询。不启动 AMCL/map_server，不运行旧 native arm/handover；5-1 的入口和默认配置不变。

| 终端 | 用途 |
|---|---|
| A | 雷达、FAST-LIO、在线 SLAM |
| B | 生成配置、检查、enable、停车和保存 |
| C | MobaXterm RViz，手选目标 |
| D | 唯一 Nav2 与速度平滑器 |
| E | 官方运动桥与 MotionSender |
| F | 单个手选目标的检查、提交和记录 |

各终端不共享 shell 变量。切换章节先停车保存，再退出旧进程；A重启后，D/F必须重新读取目录。

## A：新建图

机器狗静止，执行并保持 A 运行：

```bash
ros2 launch lite3_lab_5_2 experiment.launch.py
```

A打印本轮目录，并写入指针。其他终端按需读取：

```bash
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_2_current_run.txt")"
cat "$RUN/session.json"
```

## B/D：生成并启动导航

等地图与TF正常后，B执行：

```bash
ros2 topic info /map -v
ros2 node list
timeout 8s ros2 run tf2_ros tf2_echo map base_footprint
ros2 run lite3_lab_5_2 session prepare-nav --stage 2
```

`/map`来自`/slam_toolbox`；无AMCL/map_server和两个旧运动适配器。新版不再启动旧适配器，对它们执行`ros2 param get`会找不到节点。D加载共用环境后执行：

```bash
(
  set -e
  RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_2_current_run.txt")"
  NAV_RUN="$(cat "$UPPER/current_run.txt")"
  printf '建图目录：%s\n导航目录：%s\n' "$RUN" "$NAV_RUN"
  if [[ -n "$RUN" && "$(dirname "$NAV_RUN")" == "$RUN" && -f "$NAV_RUN/navigation.launch.py" ]]; then
    exec ros2 launch "$NAV_RUN/navigation.launch.py"
  else
    echo '目录不匹配；在B为当前A重新生成导航配置。' >&2
    exit 1
  fi
)
```

## C：本地 MobaXterm RViz

Windows打开MobaXterm X server，SSH连接香橙派，勾选 **X11-forwarding**。C加载共用环境后执行：

```bash
echo "$DISPLAY"
ros2 run lite3_lab_5_2 session rviz --stage 2
```

DISPLAY由SSH分配，如`localhost:10.0`；为空时检查X11选项并重连，不手填`:0`。RViz仍在香橙派执行，窗口转发到Windows。此入口生成独立手选配置，不改写5-1视图。

**先启动E/F并在B放行，等F提示“在RViz选择目标”，再点击。** Fixed Frame为`map`，用工具栏 **2D Goal Pose** 在已知可通行区按下鼠标，拖出朝向后松开。也可用 **Publish Point** 单击位置，沿用当前机身朝向。橙色 `Selected Manual Goal` 箭头显示选择；是否提交以F检查结果和“提交手选目标”日志为准。

手选工具发布到`/lab5/manual_goal`，经F检查后调用Nav2。不要另开普通Nav2 Goal工具绕过F。新建图不用2D Pose Estimate。**本章不调用start服务**，该服务会提示改用RViz。

用户已确认地图能显示更新，已有GLSL sampler报错不能据此认定控制失败。轻量视图约5FPS；完整X11负载下的延迟仍需现场观察。

## E/F：官方桥与手选客户端

E执行：

```bash
ros2 launch lab42_official_motion control.launch.py execute:=true \
  max_forward_mps:=0.20 max_yaw_radps:=0.20 \
  enable_odom_max_age_s:=1.0 enable_scan_max_age_s:=1.0 \
  runtime_odom_max_age_s:=1.0 runtime_scan_max_age_s:=1.0
```

启动后仍需B显式enable。`enable_*`用于放行检查，`runtime_*`才控制运行中感知检查；旧的两个enable参数不能改变运行中的0.5秒限制。现在本章里程计和扫描的源年龄、接收年龄上限均为1.0秒，F同步采用1.0秒里程计租约。速度命令看门狗仍为0.5秒，TF与碰撞规则不变。`execute:=false`可作无本体发送器的预览，但不能通过F实机门槛。

F首次建立现场参数文件，已有文件保留：

```bash
PARAMS="$HOME/catkin_ws_lite3dog/real_configs/lab5_2_exploration.yaml"
if [[ ! -f "$PARAMS" ]]; then
  cp "$(ros2 pkg prefix --share lite3_lab_5_2)/config/exploration.yaml" "$PARAMS"
fi
printf '现场参数：%s\n' "$PARAMS"
cat "$PARAMS"
```

用MobaXterm SFTP文本编辑器打开文件。现场核对包络、扫描、外参和通行空间后，才将`enable_real_motion`、`site_configuration_verified`设为`true`，启动F前保存。旧文件的探索时长、半径不再用作手选预算；`manual_goal_timeout_s`默认600秒。每次F仅处理一个目标，不自动重试或换目标。

F每次启动执行：

```bash
(
  set -e
  RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_2_current_run.txt")"
  PARAMS="$HOME/catkin_ws_lite3dog/real_configs/lab5_2_exploration.yaml"
  test -s "$PARAMS"
  AUDIT_RUN="$(mktemp -d "$RUN/exploration_XXXXXXXX")"
  printf '建图目录：%s\n本点记录：%s\n' "$RUN" "$AUDIT_RUN"
  exec ros2 run lite3_lab_5_2 experiment --ros-args \
    --params-file "$PARAMS" -p result_dir:="$AUDIT_RUN"
)
```

B检查：

```bash
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_2_current_run.txt")"
LAB42="$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization"
ros2 param get /lite3_lab_5_2 enable_real_motion
ros2 param get /lite3_lab_5_2 site_configuration_verified
ros2 param get /lab42_upper_motion runtime_odom_max_age_s
ros2 param get /lab42_upper_motion runtime_scan_max_age_s
python3 -B "$LAB42/fastlio_latency_fix/verify_live_sensor_chain.py" --output "$RUN/sensor_chain.json"
python3 -B "$UPPER/collision_snapshot.py" --output "$RUN/collision.json"
ros2 param get /controller_server FollowPath.desired_linear_vel
ros2 param get /velocity_smoother max_velocity
```

前两项为True，两个运行阈值为1.0，感知链`passed=true`，唯一里程计及安装TF发布者；扫描、足迹与现场一致。Nav2期望线速度0.10，平滑器`[0.1,0.0,0.1]`；官方桥上限仍为0.20。C已打开时再检查，覆盖实际显示负载。现场允许运动后，B执行：

```bash
ros2 service call /lab42/upper/enable std_srvs/srv/Trigger '{}'
ros2 topic echo /lab42/upper/status --once --field data
```

等`mode=ENABLED`、`ready=true`且F连续健康至少0.30秒，再在C选点。不健康的点击会拒绝且不排队，恢复后重新点击。目标中心未知、致命或内切膨胀区，以及完整目标足迹覆盖未知/致命格，都拒绝提交；普通非致命膨胀代价值不一概视为障碍。路径、转弯扫掠仍由Nav2检查。`detected collision ahead`表示预测足迹不可通行，需核对净空、地图对齐和目标位置。

Action状态2=执行、4=成功、5=取消、6=失败。运行中健康丢失会取消目标并请求官方停车，不自动enable。到达或失败后，F等待任务终结、桥DISABLED、零速和里程计停稳，再写summary并自动退出。无法确认停车时保留`stop_confirmed=false`，不算成功。

选择下一点：A/C/D/E正常时保持运行，等旧F完全退出，重新执行F启动代码，再由B enable，最后C点新目标。地图持续累积；记录仍命名`exploration_*`以兼容保存器，内容标记`goal_selection=rviz_manual`。

## 延迟排查记录

A在`$RUN/timing.jsonl`每秒记录`/Odometry`、`/course/odom`、`/scan`的接收时源年龄、当前源年龄、接收间隔、时间戳倒退、记录节点定时器延迟及系统负载：

```bash
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_2_current_run.txt")"
tail -n 5 "$RUN/timing.jsonl"
```

若`/Odometry`收到时已晚，继续查FAST-LIO、雷达与调度；后端新鲜而`/course/odom`晚，查适配链；二者新鲜而F晚，查F回调及调度。记录节点定时器本身延迟高时，也须考虑其调度延迟。F故障事件同时记录自身源年龄、接收年龄和官方桥状态。

2026-11-12静止实测未见时间基准错位或时间戳倒退。同样额外CPU负载下，旧版里程计最大约0.076秒、扫描0.188秒；减负版约0.063秒、0.160秒。当时C未打开，未完整重现历史约0.64秒延迟。1秒阈值容忍抖动，减负和独立状态回调减少延迟；不重写消息/TF时间戳。这些测量不代替完整X11负载和行走验收。

## 停车、保存可续建状态

F运行中可在B调用`/lite3_lab_5_2/cancel`，或在F按Ctrl+C。直接请求官方停车：

```bash
ros2 service call /lab42/upper/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /lab42/upper/status --once --field data
AUDIT_RUN="$(ros2 run lite3_lab_5_2 session audit --stage 2)"
cat "$AUDIT_RUN/safety_status.json"
```

确认Action终结、桥DISABLED、零速、`stop_confirmed=true`并现场确认本体停稳。F退出后它的cancel服务也会消失，检查本点安全记录和官方桥。无法确认时用现场遥控停车，不再次放行。

保持A和E/D运行，B保存：

```bash
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_2_current_run.txt")"
ros2 run lite3_lab_5_2 session save --stage 2
cat "$RUN/latest_checkpoint.txt"
```

保存器校验当前记录和建图目录，重新取消任务、请求停车，保存YAML/PGM/PCD、`slam.posegraph`、`slam.data`。每次新建`checkpoint_*`，非空和哈希检查成功后才发布`checkpoint.json`。仅YAML/PGM不能恢复继续建图。

5-3从本章检查点恢复SLAM并核对实际起点，继续自主探索和静态避障验证；5-4继承5-3状态继续自主探索和动态避让恢复验证。后两章保留自动探索任务，C使用`session rviz --stage 3/4`。停车保存完成后依次退出F、E、D、C、A。编译和隔离测试不代替实际行走、续图对齐或避障验收。

## 2026-11-12：运动审核触发后的请求速度调整

本阶段B改用`session prepare-nav --stage 2`，仍调用原官方导航配置生成器，建立新的独立导航目录。Nav2请求上限改为0.10 m/s、0.10 rad/s；平滑器按比例缩放线速度和角速度以保持转弯曲率。E的官方桥上限和F的瞬时实测审核上限仍为0.20，未关闭超速、过期、碰撞或停车检查。5-1入口不变。

旧D不会自动加载新配置。先停车并正常退出旧F/D/E，保持A/C（若仍在运行），在B重新生成配置，D读取新目录启动，再按E/F及enable流程操作。A已经退出则按本章启动或续图步骤恢复完整会话。不要只改现场YAML而继续使用旧D配置。

F的`csv/motion.csv`逐帧保存原始body-frame vx/vy/vz、wx/wy/wz、平面速度、源时间戳和官方桥输出。超限日志会明确打印速度分量，summary保留峰值。当前配置为速度跟踪留余量，未经过真实行走复测；若再次触发超限，保留记录核对真实运动与里程计估计。安全文件另记录2秒连续odom位姿窗口的速度拟合与最大跨度：拟合平移/转向速度须分别≤0.005 m/s和0.005 rad/s，窗口内位置跨度≤0.020 m、朝向跨度≤0.020 rad，并继续满足数据新鲜、桥DISABLED、任务终结及连续稳定要求。保存器使用同一判据。原始单帧速度保留，真实停稳仍由现场确认；stop_confirmed=false不能算软件确认停车。
