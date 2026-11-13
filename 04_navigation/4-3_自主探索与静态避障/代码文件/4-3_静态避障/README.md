# 5-3 恢复 5-2 建图状态，继续探索与静态避障测量｜Lite3 实机

本入口已改为在线 SLAM 探索。原实验算法源码保留用于回归；普通启动不再走原多点巡检或 native 控制流程。每次运行建立独立目录，不覆盖旧图。

## 环境与运动链

所有 Linux 命令均在香橙派执行。每个新终端先退出 conda 的 `(base)`，再执行：

```bash
source "$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization/official_motion_bridge/env.sh"
export AUTO="$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization/nav2_autonomy_tools"
source "$AUTO/exploration_ws/install/local_setup.bash"
export STAGE=3
```

统一 ROS 2 Humble、域号 43、`ROS_LOCALHOST_ONLY=1`、真实系统时间。沿用 MID-360、FAST-LIO、原始传感器时间戳、5-1 的 TF2 修复库和最新里程计适配器。雷达网口实际为 `eth0 192.168.1.50/24`，雷达 `192.168.1.3`。运动前本体路由须为 `192.168.2.1 dev wlan0 src 192.168.2.233`；管理 SSH 地址 `.10` 不表示本体已经直连。

```text
MID-360 → FAST-LIO → /course/odom、odom→base_footprint→base_link
/scan → slam_toolbox → /map、map→odom
边界探索 → NavigateToPose → Nav2 → 速度平滑器
→ 官方上层运动桥 → MotionSender → 本体
```

机身参考包络 `0.779×0.480 m`，规划 padding `0.10 m`；雷达安装 xyz `[0.210,0,0.161308]`、RPY `[0,0.338594,0]`，初始机身高度 `0.325504965 m`。包络仍须覆盖当前足腿与负载。Nav2请求上限为 `0.10 m/s、0.10 rad/s`，官方桥和实测审核上限保留 `0.20 m/s、0.20 rad/s`。静态测量的运动误差界同步采用这两个上限。

5-1 保持不变；本章在线 SLAM 模式不启动 AMCL/map_server，不以 AMCL 粒子收敛作为门槛。不调用旧 native arm/handover，不运行旧 `nav2_real_lab.py explore`。`/lab_4_2` 只是复用的建图记录节点，不负责运动。

## 本地 MobaXterm 操作 RViz

在 Windows 启动 MobaXterm 的 X server。新建 SSH 会话，地址用香橙派当前可达地址（管理网 `.10`，连接本体 Wi-Fi 后可能是 `192.168.2.233`），用户名 `HwHiAiUser`，勾选 **X11-forwarding**。会话切换网络后需要重连。密码在 MobaXterm 提示时输入，不写进脚本。

板端已确认 `/etc/ssh/sshd_config` 的 `X11Forwarding yes` 和 `xauth` 存在。连接后的 C 终端加载上面的环境，再执行：

```bash
echo "$DISPLAY"
ros2 run lite3_lab_5_2 session rviz --stage 3
```

`DISPLAY` 应由 SSH 自动分配，例如 `localhost:10.0`。为空时重连并检查 X11 选项；不要手填 `:0`，也不要开放 `xhost +`。不再使用拒绝 SSH 的 `run_local_navigation_view.sh`。RViz 进程仍在香橙派，只有窗口转发到本地；无需 Windows 安装 ROS，也无需修改 ROS 网络范围。

视图复用 5-1 的轻量配置：Fixed Frame=`map`、约 5 FPS，只保留地图、LaserScan、TF、路径、足迹和局部代价地图。扫描 QoS 为 Best Effort/Volatile/Depth 1。GLX 报错时检查 MobaXterm X server 的 OpenGL 设置；可在 C 单独尝试 `export LIBGL_ALWAYS_SOFTWARE=1` 后重开窗口，不能据此认定所有 X server 都兼容。此次尚未实测本地 X11 的 RViz 画面或性能。

**自主探索不在 C 点击目标。** F 会自动选择已知自由区域与未知区域交界处的探索目标，Nav2负责规划和跟踪；C只观察地图、扫描、足迹、路径与 `/explore/frontiers`。地图尺寸变大本身不能证明机器狗已自主行走。

新建图不需要 2D Pose Estimate。续图使用 A 命令里的实际 `start_pose`；若扫描与旧墙体不对齐，保持停车，退出本轮并用修正起点新建会话。RViz 的 AMCL 初始位姿按钮不会替代 SLAM 位姿图恢复。自主探索期间不要手动点击 Nav2 Goal/Publish Point 插入第二个任务。可添加 MarkerArray `/explore/frontiers` 观察探索候选。

## 启动顺序与终端

| 终端 | 用途 |
|---|---|
| A | 雷达、FAST-LIO、在线 SLAM；本章下方给出启动命令 |
| B | 生成导航配置、检查、显式 enable/start、停车与保存 |
| C | MobaXterm X11 转发的 RViz |
| D | 唯一 Nav2 与速度平滑器 |
| E | 官方运动桥与 MotionSender |
| F | 探索任务及静态/动态测量 |

切换章节前先按本章停车和存图步骤完成，确认本体停稳，然后正常退出旧 F/E/D/C/A。所有终端只加载同一套环境；禁止并行启动第二套 SLAM、AMCL、LIO 或运动出口。

## A：启动本阶段建图

所有终端在加载共用环境后设置本阶段编号：

```bash
export STAGE=3
```

先找到 5-2 保存成功后打印的 `checkpoint.json`。它包含 YAML/PGM/PCD、`slam.posegraph`、`slam.data` 与校验值；只有旧 YAML/PGM 不足以续图。机器狗须位于上一张地图可匹配的覆盖区，实际位置与朝向已核对。

在 A 输入**实际路径与实际起点**，不能直接照抄示例坐标：

```bash
read -r -p '上一阶段 checkpoint.json 绝对路径：' CHECKPOINT
read -r -p '当前起点 [map_x, map_y, yaw]：' START_POSE
ros2 launch lite3_lab_5_3 experiment.launch.py \
  checkpoint:="$CHECKPOINT" start_pose:="$START_POSE" pose_confirmed:=true
```

`start_pose` 是当前机器狗相对旧地图的位置，不是新 LIO 的 odom 原点。`saved_pose_map` 只可作为未搬动时的参考。启动前会验证上一阶段编号、文件校验值及传感器/运动配置一致；恢复用 `map_file_name`、`map_start_pose` 和 mapping 模式，不是 AMCL 定位模式。恢复后核对 A 日志、`map→base_footprint` 和 C 中旧轮廓/实时扫描对齐，确认地图确实恢复后再运动。

A 打印本轮目录，并写入阶段指针。**不同终端不共享 shell 变量**；在 B 设置的 `RUN` 不会传到 D/F。各终端都要读取本轮指针，A 重启后也要重新读取，不能继续沿用旧值：

```bash
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_${STAGE}_current_run.txt")"
cat "$RUN/session.json"
```

## B/D：准备和启动导航

等待 SLAM 已有有效地图与 TF，B 核对唯一地图发布者和旧运动出口关闭，再生成与 5-1 相同的导航配置：

```bash
ros2 topic info /map -v
ros2 node list
timeout 8s ros2 run tf2_ros tf2_echo map base_footprint
ros2 run lite3_lab_5_2 session prepare-nav --stage 3
```

`/map` 应来自 `/slam_toolbox`；无 AMCL/map_server；A采用新版5-2的减负感知链，移除两个旧运动适配器，/lab_4_2只提供目录登记与timing.jsonl延迟记录。对已移除的适配器执行param get会找不到节点。D 执行：

```bash
(
  set -e
  source "$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization/official_motion_bridge/env.sh"
  RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_3_current_run.txt")"
  NAV_RUN="$(cat "$UPPER/current_run.txt")"
  printf '本轮建图目录：%s\n导航配置目录：%s\n' "$RUN" "$NAV_RUN"
  if [[ -n "$RUN" && "$(dirname "$NAV_RUN")" == "$RUN" && -f "$NAV_RUN/navigation.launch.py" ]]; then
    exec ros2 launch "$NAV_RUN/navigation.launch.py"
  else
    echo '上述目录不匹配或启动文件缺失；先核对A的本轮目录，再在B生成导航配置。' >&2
    exit 1
  fi
)
```

## E/F：运动桥与探索

E 的真实出口命令与 5-1 一致；启动后仍须 B 显式 enable，不自动站立或开始运动：

```bash
ros2 launch lab42_official_motion control.launch.py execute:=true \
  max_forward_mps:=0.20 max_yaw_radps:=0.20 \
  enable_odom_max_age_s:=1.0 enable_scan_max_age_s:=1.0 \
  runtime_odom_max_age_s:=1.0 runtime_scan_max_age_s:=1.0
```

现场未连接本体时可将 `execute:=true` 换成 `false` 做无 UDP 的预览；预览状态下探索不会通过真实运动门控。本阶段运行中的里程计和扫描源年龄、接收年龄上限均为1.0秒，任务节点与官方桥一致；原始时间戳不变，速度命令看门狗仍为0.5秒。

F 首次将本阶段探索参数复制到独立现场文件，保留已存在文件：

```bash
PARAMS="$HOME/catkin_ws_lite3dog/real_configs/lab5_${STAGE}_exploration.yaml"
if [[ ! -f "$PARAMS" ]]; then
  cp "$(ros2 pkg prefix --share lite3_lab_5_${STAGE})/config/exploration.yaml" "$PARAMS"
fi
printf '待编辑参数文件：%s\n' "$PARAMS"
cat "$PARAMS"
```

用 MobaXterm 左侧 SFTP 面板打开上面打印的文件，在文本编辑器中修改并保存；无需安装 nano。编辑失败时不要直接继续启动 F。只有现场核对完成后，才将文件内 `enable_real_motion` 和 `site_configuration_verified` 两项设为 `true`。文件必须在启动 F 前保存；F 已运行时，单独修改文件不会更新节点参数。

核对包络、扫描、外参和现场条件后才设 `site_configuration_verified: true`、`enable_real_motion: true`。探索默认 60 秒、以开始时机身位置为中心的监测半径 2 m；整机包络触及半径时请求停车。这个半径是运行中停止监测，不能保证规划路径不越界；现场范围必须留出制动和完整扫掠余量。预算结束或无候选不等于地图完整覆盖，不自动返航。

静态验证保留实体表面净空与实际规划轮廓到膨胀边缘两项 ≥0.10 m 的测量，原始 header 时间 TF、缺测/盲区和首尾观测间隔规则不变。`envelope_verified`、`sensor_coverage_verified`、`motion_bound_verified` 默认 false，必须有实测依据后再设置；默认误差上界 0.15 m 仍需现场标定。速度误差界已与实际 0.20/0.20 上限同步。只执行导航不代表静态验收通过，未核实的测量保持未评估。

F 启动并保持运行，然后 B 核对实时扫描、足迹和规划路径净空：

```bash
(
  set -e
  STAGE=3
  RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_${STAGE}_current_run.txt")"
  PARAMS="$HOME/catkin_ws_lite3dog/real_configs/lab5_${STAGE}_exploration.yaml"
  test -s "$PARAMS"
  AUDIT_RUN="$(mktemp -d "$RUN/exploration_XXXXXXXX")"
  printf '本轮建图目录：%s\n探索记录目录：%s\n探索参数文件：%s\n' "$RUN" "$AUDIT_RUN" "$PARAMS"
  exec ros2 run lite3_lab_5_${STAGE} experiment --ros-args \
    --params-file "$PARAMS" -p result_dir:="$AUDIT_RUN"
)
```

F 每2秒记录等待项。地图与Nav2准备好后，它先直接启动探索器并保持暂停，等待“探索器已准备好并保持暂停”提示；此阶段不发送导航目标，也不消耗探索时间预算。B随后完成检查、enable和start。F只接收最新里程计状态，真实源时间与接收时间的上限为1.0秒。

B 的感知与碰撞检查：

```bash
STAGE=3
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_${STAGE}_current_run.txt")"
LAB42="$HOME/catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization"
ros2 param get /lite3_lab_5_${STAGE} enable_real_motion
ros2 param get /lite3_lab_5_${STAGE} site_configuration_verified
python3 -B "$LAB42/fastlio_latency_fix/verify_live_sensor_chain.py" --output "$RUN/sensor_chain.json"
python3 -B "$UPPER/collision_snapshot.py" --output "$RUN/collision.json"
ros2 param get /controller_server FollowPath.desired_linear_vel
ros2 param get /velocity_smoother max_velocity
```

前两项应分别读回 `Boolean value is: True`；否则 F 会一直显示 `real motion/site verification not enabled`，不会启动探索，也不会提交导航目标。`start` 只在探索器已暂停就绪、全部健康条件连续满足0.30秒时接受；未就绪返回 `success=False`，不排队自动开始。检查 `passed=true`、无查询错误、单一里程计与安装 TF 发布者；当前位置无致命足迹格，扫描与现场一致。Nav2期望线速度为0.10，平滑器为 `[0.1,0.0,0.1]`；官方桥上限仍为0.20。C 的 X11 窗口和需要的录包均已运行时再次确认感知延迟满足原阈值，避免图形负载造成运动前后状态差异。

确认现场允许运动后，B 执行：

```bash
ros2 service call /lab42/upper/enable std_srvs/srv/Trigger '{}'
ros2 topic echo /lab42/upper/status --once --field data
```

只有 `mode=ENABLED`、`reason=ENABLED_WAIT_GOAL`、`ready=true` 后发起一次探索请求：

```bash
ros2 service call /lite3_lab_5_${STAGE}/start std_srvs/srv/Trigger '{}'
```

F接受start后会打印“自主探索已恢复”，随后自动生成目标，不需要在C点目标。目标状态打印为2时表示Nav2执行中，4表示成功，5表示取消，6表示失败。未准备好的start请求会明确拒绝且不排队，等等待项消失后由操作者重新发起。运动中任何健康丢失均取消并停车，不自动重使能；故障事件额外保存里程计源年龄、接收年龄和官方桥状态。F是任务节点，不直接发布速度。

5-2现为RViz手选单目标，不自动换点；5-3默认 `max_aborted_goals: 3`：Nav2终止一个目标后，保留该失败UUID，由探索器将其列入黑名单并尝试其他目标；累计3个失败目标即结束本次探索。换目标期间仍执行原有传感器、官方桥和碰撞检查，健康丢失立即停车，不自动重新enable。5-4 默认 `max_aborted_goals: 0`，首个失败目标即结束，避免把更换目标混入同一次动态横穿验收。实际使用值可通过 `ros2 param get /lite3_lab_5_${STAGE} max_aborted_goals` 读回。报告同时保留 `aborted_goals` 与 `all_observed_goals_succeeded`，不把失败目标算作成功。

若D报 `RegulatedPurePursuitController detected collision ahead`，说明本目标的预测足迹扫掠被判不可通行。当前朝向的静态足迹快照没有致命格，不能证明转身或弯道扫掠安全。核对实时障碍、地图配准和整机转弯余量；不得通过关闭碰撞检查、缩小包络或忽略未知格强行运行。

## 本轮结束后，只重启 F 继续使用当前地图

若F显示“本轮已结束”，重复enable/start不会恢复这轮。先在B请求 `/lab42/upper/stop`，确认DISABLED和现场停稳，再在旧F按Ctrl+C并等待完全退出。

A的SLAM、D的Nav2和E的官方桥仍正常时，可保持A/C/D/E运行，重复上面的F启动代码。它每次建立新的 `exploration_*` 目录，并自动更新 `$RUN/exploration_current.txt`；旧目录和失败证据完整保留。新F准备好后再由B显式enable和start。保存器和 `session audit` 自动读取新的F目录。

已结束的旧F仍运行时，如果再次enable，旧F会重新请求停车。先退出旧F，再启动新F，不能并行运行两份任务节点。A已经退出时不能只启动F，须按本章新建或恢复完整建图会话。

## 停车、保存可续建检查点与退出

可让预算结束，也可在 B 主动取消：

```bash
STAGE=3
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_${STAGE}_current_run.txt")"
ros2 service call /lite3_lab_5_${STAGE}/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /lab42/upper/status --once --field data
AUDIT_RUN="$(ros2 run lite3_lab_5_2 session audit --stage "$STAGE")"
cat "$AUDIT_RUN/safety_status.json"
```

确认 Action 已终结、探索进程已退出、桥为 DISABLED、速度为零、里程计停稳并现场确认本体停稳。`stop_confirmed` 只是软件测得的停车证据；物理停车仍须现场确认。故障或无法确认时先用原遥控停车，保留日志，不重新 enable。

保持 A 的 SLAM 与雷达运行，在 B 保存：

```bash
STAGE=3
RUN="$(cat "$HOME/catkin_ws_lite3dog/results/5_${STAGE}_current_run.txt")"
ros2 run lite3_lab_5_2 session save --stage "$STAGE"
cat "$RUN/latest_checkpoint.txt"
```

保存器再次请求取消和官方停车，确认当前建图节点属于本轮，然后调用实际 `/course/save_map` 与 `/slam_toolbox/serialize_map`。每次新建 `checkpoint_*` 目录，全部文件非空且校验成功才发布 `checkpoint.json` 和更新 `latest_checkpoint.txt`；失败留下部分目录供排查，不能用于续图。不把保存成功当作探索或避障验收通过。

下阶段使用打印出的 `checkpoint.json`，不使用旧的固定地图路径。地图重载保留原 map 坐标关系，恢复 SLAM 前传入当前实际起点；每次保存均保留父检查点路径、配置和哈希。

结束后依次 Ctrl+C 退出 F、E、D、C、A；若另有录包，先完成停车，再正常结束录包。F 被 Ctrl+C 时同样请求取消与官方停车，但程序退出不能替代现场停车确认。

## 证据与验证边界

`session.json` 记录阶段、父图和配置；`exploration_*/raw/events.jsonl` 记录探索任务与真实 Action 终态；`summary.json` 区分导航成功、边界候选耗尽与完整覆盖；`safety_status.json` 记录取消、官方出口和停车证据。5-3 另有两项净空 CSV，5-4 有真实横穿轨迹和重规划指标。

本次软件检查覆盖编译、无运动节点启动、参数传递、文件完整性与原测量算法回归。实际行走、SLAM续图对齐、避障效果和 MobaXterm RViz 显示仍须现场验证，不能据离线通过填写真机成绩。

## 2026-11-12：运动审核触发后的请求速度调整

本阶段B改用`session prepare-nav --stage 3`，仍调用原官方导航配置生成器，建立新的独立导航目录。Nav2请求上限改为0.10 m/s、0.10 rad/s；平滑器按比例缩放线速度和角速度以保持转弯曲率。E的官方桥上限和F的瞬时实测审核上限仍为0.20，未关闭超速、过期、碰撞或停车检查。5-1入口不变。

旧D不会自动加载新配置。先停车并正常退出旧F/D/E，保持A/C（若仍在运行），在B重新生成配置，D读取新目录启动，再按E/F及enable流程操作。A已经退出则按本章启动或续图步骤恢复完整会话。不要只改现场YAML而继续使用旧D配置。

F的`csv/motion.csv`逐帧保存原始body-frame vx/vy/vz、wx/wy/wz、平面速度、源时间戳和官方桥输出。超限日志会明确打印速度分量，summary保留峰值。当前配置为速度跟踪留余量，未经过真实行走复测；若再次触发超限，保留记录核对真实运动与里程计估计。安全文件另记录2秒连续odom位姿窗口的速度拟合与最大跨度：拟合平移/转向速度须分别≤0.005 m/s和0.005 rad/s，窗口内位置跨度≤0.020 m、朝向跨度≤0.020 rad，并继续满足数据新鲜、桥DISABLED、任务终结及连续稳定要求。保存器使用同一判据。原始单帧速度保留，真实停稳仍由现场确认；stop_confirmed=false不能算软件确认停车。
