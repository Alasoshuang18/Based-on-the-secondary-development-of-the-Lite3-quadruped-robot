# Lite3 第六章文件整理与仿真运行指南

本版针对你提供的 Orange Pi AI Pro 20T 工作目录，使用 ROS2 Humble、Gazebo Fortress 和现有 Lite3 URDF。源码、环境脚本、测试及独立构建均归到 `06_simulationmode_grazebo/6-3_robot_sport`；包清单和 CMakeLists.txt 保留在 ROS 包根目录。相机、基础运动和工作空间根目录文件保留。

## 1 日志中的旧路径是什么

A 明确报过：

```text
/home/HwHiAiUser/catkin_ws_lite3dog/install/leg_pd_controller/share/leg_pd_controller/leg_pd_controller.xml
```

这是工作空间根目录旧安装中的插件 XML。controller_manager 的插件发现仍扫描到了它，说明 A 的环境含有旧包索引；这不等于当前 Lite3 关节由旧 PD 控制器驱动。日志实际加载的是 joint_state_broadcaster 和 lite3_leg_controller。pluginlib 会从 ament 索引收集插件描述文件：[Humble pluginlib 源码](https://github.com/ros/pluginlib/blob/humble/pluginlib/include/pluginlib/class_loader_imp.hpp)。

新版环境脚本在当前终端过滤旧工作空间 install、包内旧 install 与 `.lite3_runtime` 的路径，然后加载系统 ROS 和本章新的 `.runtime/install`。它保留 ROS_DOMAIN_ID、RMW_IMPLEMENTATION 等已有通信设置，不修改全局 env_ros2.sh 或其他进程。

修改环境后必须重启 A/B/C/E 等旧进程。只在 D 中 source 不会改变已经运行的 A 的环境。此前 `xmlrpc...!rclpy.ok()` 是命令行 daemon 的另一类故障；不能把两种错误混为一个原因。

## 2 哪些文件保留 哪些归档

目前新版实际依赖的 src 文件是以下八个：

```text
lite3_controller.py
lite3_feedback_plumbing.py
lite3_gait_core.py
lite3_measured_core.py
lite3_feedback_sync.py
lite3_keyboard.py
lite3_check_foot_lift.py
foot_debug_core.py
```

| 位置或文件 | 本轮处理 |
| --- | --- |
| src/check_foot_lift.py、gait_control.py、keyboard_control.py、measured_gait_control.py | 旧版入口，移到本包备份，不再由新运行链导入 |
| TESTING_V4_1.md、TESTING_V5.md | 旧操作说明，移到备份；以 LITE3_ONLY_TESTING.md 为准 |
| src/__pycache__ | 可重建缓存，归档后由 Python 自动生成新缓存 |
| 旧 gait/control_spawn/walk_spawn/foot_visual_spawn/pose_debug_bridge/upstream_spawn 启动文件 | 若存在，归档；新入口只用 lite3_spawn 与 lite3_gait |
| controllers.yaml、gait.yaml、upstream_controllers.yaml | 若存在，归档；新配置为 lite3_controllers.yaml、lite3_gait.yaml |
| tests | 原测试完整备份，换入与新版源码配套的测试 |
| 包根目录 lite3_fortress_control | 若仍存在，整体归档到备份；新链不使用这些 C++ 源码 |
| source/ | 内容尚未提供，保留原样，不能仅凭名字判为无用 |
| 6-1_sim_env_setup、6-2_robot_modeling、Lite3.urdf 与 meshes | 保留 |
| 根目录 build/install/log、env_ros2.sh、Lite3_MotionSDK、Lite3_upstream_Fortress_port、Lite_csv、simulation_backups | 本轮不删除、不移动；Lite3 仿真不加载根目录旧 install |
| 01_camera_mid360、02_basic_motion_control | 保留，不修改 |

先采用可恢复归档，确认新版测试通过后，旧备份才有永久删除的依据。本次脚本不执行永久删除。备份统一在包根目录 `.lite3_backups/update_时间_编号/`，带 COLCON_IGNORE；旧文件在其中的 retired 子目录。

若需要判断 source/ 是否还能移走，先在板上列出实际内容：

```bash
find "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/source" -maxdepth 2 -type f
```

## 3 Windows 上传新版整理包

先停止旧 F、E、H、C、B、A，D 退出 echo。下载并解压此次 ZIP，目录内应有 install_lite3_only.sh、install_lite3_only.py、MANIFEST_SHA256.json、README.md、env、src、launch、config、worlds 和 tests。

先在香橙派 SSH 创建本章更新目录：

```bash
cd "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo"
mkdir -p 6-3_robot_sport/_updates
touch 6-3_robot_sport/_updates/COLCON_IGNORE
```

Windows PowerShell：

```powershell
$BoardAddress = '192.168.1.9'
$ReleaseFolder = 'C:\Users\Alaso\Documents\ChatGPT\robot_dog_env_environment\6-3_robot_sport\Lite3_only_Fortress_control'
Test-Path -LiteralPath "$ReleaseFolder\install_lite3_only.sh"
Test-Path -LiteralPath "$ReleaseFolder\env\lite3_env.bash"
scp -r "$ReleaseFolder" "HwHiAiUser@${BoardAddress}:/home/HwHiAiUser/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/_updates/"
```

两个 Test-Path 都须 True。解压位置不同只改 ReleaseFolder；目标地址固定为本章 _updates，不上传到 home 或工作空间根目录。

## 4 整理文件并在本章独立构建

香橙派 SSH：

```bash
cd "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo"
bash 6-3_robot_sport/_updates/Lite3_only_Fortress_control/install_lite3_only.sh
```

脚本校验更新文件、备份旧章节、归档已知旧入口，安装八个 Python 文件和配套 env/config/launch/worlds/tests，更新本包的安装规则。包根目录旧 Lite3_only_Fortress_control 更新副本、旧 `.lite3_runtime` 与 lite3_fortress_control 若存在，也归档。不会搬动根目录工作空间安装数据或未知 source/。

整理后的主要布局：

```text
06_simulationmode_grazebo/
├── package.xml
├── CMakeLists.txt
├── lite3_env.bash                 兼容入口，转到本章 env
├── .lite3_backups/                可恢复备份
├── 6-1_sim_env_setup/
├── 6-2_robot_modeling/
└── 6-3_robot_sport/
    ├── env/lite3_env.bash         本轮统一环境
    ├── config/lite3_controllers.yaml
    ├── config/lite3_gait.yaml
    ├── launch/lite3_spawn.launch.py
    ├── launch/lite3_gait.launch.py
    ├── worlds/lite3_walk.sdf
    ├── src/                     八个新版 Python 文件
    ├── tests/                   新版配套测试
    ├── source/                  暂留原有内容
    ├── _updates/                更新包，不参与发现
    ├── .runtime/                独立 build/install/log
    └── LITE3_ONLY_TESTING.md
```

点开头目录在普通 ls 中不显示，请用 `ls -la` 查看。第一次构建在包根目录执行：

```bash
cd "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo"
source 6-3_robot_sport/env/lite3_env.bash --build
mkdir -p 6-3_robot_sport/.runtime
touch 6-3_robot_sport/.runtime/COLCON_IGNORE

colcon --log-base 6-3_robot_sport/.runtime/log build \
  --base-paths . \
  --build-base 6-3_robot_sport/.runtime/build \
  --install-base 6-3_robot_sport/.runtime/install \
  --packages-select simulationmode_grazebo \
  --symlink-install

source 6-3_robot_sport/env/lite3_env.bash
ros2 pkg prefix simulationmode_grazebo
ros2 pkg executables simulationmode_grazebo | grep -E 'lite3_controller|lite3_only_keyboard'
```

`--build` 只过滤旧路径、加载系统 ROS，不要求首次构建前已经有新 install。正常 prefix 必须是：

```text
/home/HwHiAiUser/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/.runtime/install/simulationmode_grazebo
```

继续检查：

```bash
SIM_SHARE=$(ros2 pkg prefix --share simulationmode_grazebo)
ls -l "$SIM_SHARE/6-3_robot_sport/launch/lite3_spawn.launch.py"
ls -l "$SIM_SHARE/6-3_robot_sport/config/lite3_controllers.yaml"
printf '%s\n' "$AMENT_PREFIX_PATH" | tr ':' '\n'
```

AMENT_PREFIX_PATH 中不应再有 `/home/HwHiAiUser/catkin_ws_lite3dog/install/...` 或包根旧 `.lite3_runtime/install/...`。新运行文件以本章 .runtime 下的安装为准。以后各终端统一 source 本章 env；不要随后再 source 根目录旧 env_ros2.sh 或旧 install。

## 5 终端分工

| 终端 | 用途 | 运行方式 |
| --- | --- | --- |
| A | Gazebo 物理服务器 | 保持运行 |
| B | Gazebo 图形界面 | 保持运行 |
| C | 生成模型、描述与反馈桥接 | 保持运行 |
| D | 加载控制器、服务请求、状态观察 | 命令执行完返回 shell；echo 用 Ctrl+C 退出 |
| E | Lite3 Python 控制器 | 保持运行 |
| F | WASD 键盘 | 自由机身四腿验收通过后才启动 |
| H | 实际脚高统计 | 另开 SSH 终端，保持运行至 Ctrl+C |

每个新终端都先执行：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
```

正在运行 launch 的 E 不能同时执行 H 的统计命令。按 W 时，焦点必须在 F 的键盘程序中；在 D 或 H 输入 W 不会发速度。

## 6 A B C 启动固定机身校准

A：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
SIM_SHARE=$(ros2 pkg prefix --share simulationmode_grazebo)
ign gazebo -s -v 4 "$SIM_SHARE/6-3_robot_sport/worlds/lite3_walk.sdf"
```

B：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
ign gazebo -g -v 4
```

C：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
ros2 launch simulationmode_grazebo lite3_spawn.launch.py pin_body:=true
```

模型生成后，在 B 点击左下角 ▶ 播放。固定机身时身体保持原位是预期行为；十二个关节和足端应仍能运动。这个阶段只验收模型、逆解和关节执行，不证明自由行走稳定。

## 7 D 加载本版控制器与检查时钟

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
ros2 service list | grep '/lite3_sim/controller_manager/list_controllers'
ros2 run controller_manager spawner joint_state_broadcaster -c /lite3_sim/controller_manager --controller-manager-timeout 60 --switch-timeout 120
ros2 run controller_manager spawner lite3_leg_controller -c /lite3_sim/controller_manager --controller-manager-timeout 60 --switch-timeout 120
ros2 control list_controllers -c /lite3_sim/controller_manager
```

应看到 `joint_state_broadcaster` 和 `lite3_leg_controller` 均为 active。服务未出现时先检查 A、C；激活超时时先确认 B 正在播放。已有 active 控制器不重复加载。

```bash
timeout 10s ros2 topic echo /clock --once
timeout 10s ros2 topic echo /lite3_sim/joint_state_broadcaster/joint_states --once
timeout 10s ros2 topic echo /lite3_sim/imu --once --qos-reliability best_effort
timeout 10s ros2 topic echo /lite3_sim/model_pose tf2_msgs/msg/TFMessage --once

ros2 param get /lite3_sim/controller_manager use_sim_time
ros2 param get /lite3_sim/joint_state_broadcaster use_sim_time
ros2 param get /lite3_sim/lite3_leg_controller use_sim_time
ros2 param get /lite3_sim/robot_state_publisher use_sim_time
ros2 topic info /clock -v
```

四类反馈应有消息，四个参数应为 True，`/clock` 应只有一个发布者。位姿中模型对应 transform 应有 `frame_id: lite3_world`、`child_frame_id: lite3_control` 和有效 stamp。

再检查控制器消息：

```bash
timeout 10s ros2 topic echo /lite3_sim/lite3_leg_controller/controller_state --once
```

clock、关节、位姿和控制器的消息 stamp 应在同一仿真时间尺度。逐条查询期间世界继续运行，不要求数值完全相同；日志开头的系统日期秒数不能代替消息 header 检查。

## 8 E 启动 D 启用

E：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
ros2 launch simulationmode_grazebo lite3_gait.launch.py pin_body:=true
```

C 和 E 的 `pin_body` 必须相同，E 启动日志应包含 `lite3-only-v5.2`。E 保持运行；每次重启 E 都会回到 DISARMED，之前的测试通过结果不保留。

D：

```bash
ros2 param get /lite3_sim/lite3_controller use_sim_time
ros2 topic echo /lite3_sim/gait_status --once --field data
ros2 service call /lite3_sim/enable_gait std_srvs/srv/Trigger '{}'
ros2 topic echo /lite3_sim/gait_status --field data
```

参数应为 True，服务应返回 success=True。等待约 3 个仿真秒的站姿过渡后，状态应为 READY。此时 `lift_test=0/4`、两个通过标志为 False 表示尚未测试。READY 后在 D 按 Ctrl+C 退出 echo，继续执行服务命令。

`sync_skew_ms` 应不超过 25 ms，`sync_age_ms` 保持小于 200 ms。`sync_holds` 增加只表示等候新的同步反馈，本身不代表故障。有效配对长期不更新、时钟回退或反馈不满足条件时，节点仍会停止；不能用放宽门槛替代时间源和桥接检查。

## 9 H 实际脚高统计 D 四腿校准

H 另开 SSH 终端：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
python3 "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/src/lite3_check_foot_lift.py"
```

`sim` 和 `samples` 应递增。高度是同步位姿和关节反馈计算出的 22 mm 球形脚底距 `z=0` 平地的几何距离，不是接触力。持续等待同步数据时，先处理反馈，不评价抬脚。

D 在 READY 后执行：

```bash
ros2 service call /lite3_sim/test_foot_lift std_srvs/srv/Trigger '{}'
ros2 topic echo /lite3_sim/gait_status --field data
```

这一轮不启动 F，不发速度。模型按 **HL → FL → HR → FR** 自动逐腿抬起、保持、放下。规划抬高为 30 mm，每条腿约 1.5 个仿真秒抬升、1 秒保持、1.5 秒下降，反馈确认可能延长时间。

验收时当前腿实测离地至少 20 mm，并在新反馈中持续满足约 0.2 个仿真秒；其他三脚保持地面附近。D 的 `leg` 必须对应 H 中明显离地的脚，下降后还要确认落脚才能计数。

固定机身完成后应回到：

```text
READY; ... lift_test=4/4; ... calibration_passed=True; ... lift_passed=False
```

固定校准时 `lift_passed=False` 正常。服务 success=True 只表示接受测试请求，不能代替最终 4/4；发生 FAULT 时保留 D、E、H 输出，不提高速度跳过测试。

## 10 完整重启为自由机身

固定校准通过后，在 E、H、C、B、A 分别按 Ctrl+C，D 退出 echo。按第 6 节重新启动 A、B；C 改为：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
ros2 launch simulationmode_grazebo lite3_spawn.launch.py pin_body:=false
```

B 点击 ▶ 播放，D 按第 7 节重新加载两个控制器和检查反馈。新世界不会保留上一轮控制器状态。

E 改为：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
ros2 launch simulationmode_grazebo lite3_gait.launch.py pin_body:=false
```

D 按第 8 节重新 enable 并等待 READY；H 按第 9 节重新记录。C 和 E 本轮都为 false，固定机身的通过结果不沿用。

D 再调用 `/lite3_sim/test_foot_lift`。自由测试先转移支撑，再逐腿抬落。检查实际姿态、抬脚对应关系和其他三脚的几何支撑；关节跟踪误差接近零不能代替支撑验收。

自由机身完成后应出现：

```text
READY; ... lift_test=4/4; ... lift_passed=True
```

只有本轮自由测试通过，才进入键盘测试。若计划 HR 抬脚而 FL 离地，或节点仍为 FAULT，就保留日志并停止后续行走。

## 11 F 低速 WASD 与停止

自由测试通过后保持 A、B、C、E 运行，H 继续或重新记录。F：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
ros2 run simulationmode_grazebo lite3_only_keyboard --ros-args -p use_sim_time:=true -p linear_speed:=0.003 -p angular_speed:=0.01 -p key_timeout:=0.5
```

先按一次空格发零速度，解除测试结束后的输入屏障。之后焦点保持在 F：W 前进、S 后退、A 左转、D 右转、空格停止、Q 退出。长按依赖终端键盘重复；无新方向输入约 0.5 个真实秒后自动请求停止。

先测试 W 至少一个四腿周期，再空格停止并等待 READY。前进通过后，分别测试 S、A、D。`0.003 m/s` 与 `0.01 rad/s` 是输入请求，不能当作实测机身速度；本版没有闭环控制实际机身速度。

在空闲 D 检查输入时，可持续 echo `/lite3_sim/cmd_vel`，再切回 F 按键。D 里的 Ctrl+C 只停止 echo。测试结束后的服务停止为：

```bash
ros2 service call /lite3_sim/stop_gait std_srvs/srv/Trigger '{}'
ros2 topic echo /lite3_sim/gait_status --field data
```

正常停止会安排当前腿落脚与站立；该服务不清除 FAULT，也不能恢复已经失稳的机身。结束整轮测试时，先停止 F，再停止 E、H、C、B、A。

## 12 本轮验收记录与常见问题

| 检查项 | 合格条件 |
| --- | --- |
| 控制器 | joint_state_broadcaster 与 lite3_leg_controller 均 active |
| 时间源 | manager、两控制器、RSP 与 lite3_controller 参数均 True；clock 单发布者 |
| 同步反馈 | 四类反馈有消息；配对差不超过 25 ms，样本持续更新 |
| 固定校准 | READY、4/4、calibration_passed=True；H 四腿对应明显抬落 |
| 自由抬落 | READY、4/4、lift_passed=True；当前腿与 H 对应、其他脚保持支撑 |
| 低速行走 | 逐腿抬落、机身有净位移、能停止回到 READY，无 FAULT |

出现 `Desired controller update period ... simulation period (0 s)` 时，先确认世界正在播放和 `/clock` 推进。启动或暂停时的一次零时间步警告不能解释同步故障，也不能通过把物理步长改为零处理。

若反馈为 `Pose/joint timestamps...` 或持续过旧，检查同一轮 A/C、单 clock 发布者、消息 stamp 与节点参数；使用本版缓存配对文件，不能同时运行旧步态节点。若 `controller_rejected_old` 或 `controller_rejected_future` 持续增加，重点检查 `/lite3_sim/lite3_leg_controller` 的时钟设置。

若 E 拒绝 enable，按服务 message 处理：模型固定模式不符时完整重启 C 与 E；脚底不在地面附近、姿态异常或关节偏离初始站姿时重新创建世界和模型。若 H 正常但 D 只有 0/4，查看测试服务是否接受，以及 E 是否已经进入 FAULT。

把最终 D 状态、E 故障行、H 四脚最大高度及 Gazebo 模型前后位姿一起记录，才能区分目标变化、实际抬脚、平移滑动和正确的足端推进。单张截图或本地单元测试不能单独证明自由行走已经稳定。



## 13 环境与命令行查询的恢复

先按第 4 节构建、加载环境，再新开 A/B/C/D/E/F/H 并加载同一个本章 env。A 不再出现旧 leg_pd_controller XML 路径，才说明新服务器没有继续扫描那份旧索引。新的 CMake 和路径配置不需要安装 Unitree 控制器。

D 若仍遇到 XMLRPC 的 !rclpy.ok()，保留正在运行的仿真，在 D 重启命令行 daemon：

```bash
source "$HOME/catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/env/lite3_env.bash"
ros2 daemon stop
ros2 daemon start
ros2 control list_controllers -c /lite3_sim/controller_manager
```

若查询仍失败，用直接服务确认控制器状态：

```bash
ros2 service list --no-daemon | grep '/lite3_sim/controller_manager'
ros2 service call /lite3_sim/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
```

已经 active 的控制器不重复 spawner。ROS_DOMAIN_ID 和 RMW_IMPLEMENTATION 在 A、C、D、E 应相同；脚本不强行改这些已有设置。各终端可用下面命令对照：

```bash
printf 'ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s\n' "${ROS_DOMAIN_ID:-0}" "${RMW_IMPLEMENTATION:-默认}"
```

## 14 后续修改

以后更新源码仍放在本章 _updates，按第 4 节安装、构建；每次只选 simulationmode_grazebo。模型、launch、控制器配置或环境改变后完整重启服务器和模型；只改键盘程序时重启 F。

整理目录与过滤旧环境不会改变步態算法。本版仍先做固定机身逐腿校准，再做自由机身抬落，最后低速 WASD；只有实际验收通过，才能记录行走通过。包内备份可以恢复旧文件，source/ 与其他工作空间目录的用途尚未核实前不作永久删除。
