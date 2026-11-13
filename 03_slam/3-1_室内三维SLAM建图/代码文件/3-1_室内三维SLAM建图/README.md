# 实验原理

本课在Orange Pi上调用真实MID-360与官方FAST-LIO ROS2后端。激光提供环境几何，IMU在激光帧之间预测运动，LIO估计机身轨迹并将真实点配准到局部地图。本包Python负责健康监测、三次闭环记录和PCD保存，不生成替代里程计。

| 输入/输出 | 意义 |
|---|---|
| `/livox/lidar` | 官方CustomMsg，保留真实逐点offset，直接送LIO |
| `/course/points_raw` | 同源PointCloud2，用于观察和录制 |
| `/course/imu` | 仅转换一次的SI IMU：m/s²、rad/s；姿态不可用时明确标记 |
| `/course/odom` | LIO实际估计的base_link轨迹 |
| `/course/lio_points` | 配准点云，本课不要求二维`/map` |
| `/course/save_pcd` | 保存有界体素缓存并实际重载点数据 |

先静置初始化；教学`StreamMonitor`在开轮前及结束时检查100个IMU样本的模长均值8.5～11m/s²、标准差<0.8m/s²，四路数据还须保持新鲜。只有通过静置与SI单位检查后才能开始记录。记录期间保留IMU均值和标准差作为诊断，不把步态引起的动态加速度当作“未初始化”；继续检查有限值、有效样本、原始时间戳与四路数据新鲜度。非有限IMU会撤销本轮的初始化核对状态，原数据异常仍使该轮失败。停车后等约2秒、确认`imu_stationary_ready=true`再调用`finish_trial`。新健康记录增加`health_phase`、`imu_stationary_ready`与`health_reasons`，动态阶段的ready仅表示数据链路满足检查，不代表地图精度合格，也不是对动态IMU硬件、量程或精度的全面诊断。动态阶段ready可能为true而imu_stationary_ready为false；结束前须确认两者均为true。这只说明输入基本健康，精度和运动去畸变要另做实测。LIO局部地图采用120m窗口、30m检测参数，满足`cube/2 > 1.5×det_range`；树只存实际点，不预分配整个空窗口。

# 实验目的

1. 建立真实激光、IMU、LIO里程计和点图链路，检查单位、时钟和安装外参。
2. 用实际地面起点和朝向完成三次同路线闭环，记录失败、时延和漂移。
3. 保存ASCII PCD并重读点数和有限XYZ，区分请求发出与保存成功。

# 实验步骤

**从零创建实验包（已有交付目录跳过）。** 以下命令只供本实验目标尚不存在的空位置从零重建教学使用；现有交付目录不要重复创建、嵌套或覆盖。任一保护检查失败就使用现有交付源码，继续本节后面的环境与构建步骤；子shell的`exit 1`只停止本段，不关闭终端。

```bash
(
  set -e
  source ~/catkin_ws_lite3dog/course_env.sh
  mkdir -p ~/catkin_ws_lite3dog/src/04_slam
  cd ~/catkin_ws_lite3dog/src/04_slam
  test ! -e lite3_lab_4_1 && test ! -L lite3_lab_4_1 || { printf '%s\n' 'lite3_lab_4_1已存在，停止创建'; exit 1; }
  test ! -e 4-1_lidar_inertial_mapping && test ! -L 4-1_lidar_inertial_mapping || { printf '%s\n' '4-1_lidar_inertial_mapping已存在，停止创建'; exit 1; }
  ros2 pkg create lite3_lab_4_1 --build-type ament_python --license Apache-2.0 \
    --dependencies rclpy lite3_course_common sensor_msgs nav_msgs geometry_msgs \
    std_srvs nav2_msgs tf2_ros
  mv -- lite3_lab_4_1 4-1_lidar_inertial_mapping
  cd 4-1_lidar_inertial_mapping
  mkdir -p launch config tests rviz
)
```

`4-1_lidar_inertial_mapping`是外层课程实验目录名，方便按章节阅读；ROS包名、`package.xml`的`<name>`、`setup.py`中的`package_name`、`resource/lite3_lab_4_1`和内层Python模块目录必须仍为合法名字`lite3_lab_4_1`。重命名只改变外层目录，不把数字开头、含连字符的实验目录名写成ROS包名或Python模块名。上面的依赖逐项来自本包实际`package.xml`的ROS依赖；`python3-*`对应的NumPy、YAML等系统/Python依赖按环境安装，不放入这个创建命令。

脚手架只有包清单、安装配置、资源索引和Python模块骨架，尚未实现本实验算法。运行前必须恢复交付的Python源码、`launch/config/tests`、RViz文件及配套`package.xml/setup.py/setup.cfg`。在`setup.py`的`console_scripts`中注册`experiment = lite3_lab_4_1.experiment:main`，并用`data_files`把`resource/lite3_lab_4_1`安装到`share/ament_index/resource_index/packages`，把`package.xml`安装到`share/lite3_lab_4_1`，把`launch/config/rviz`中的资源安装到`share/lite3_lab_4_1/`下对应子目录；保留交付中其它实际入口和数据注册。恢复`launch/experiment.launch.py`的`generate_launch_description()`入口及公共启动调用后，才回到`~/catkin_ws_lite3dog`按后文执行`colcon build --packages-up-to lite3_lab_4_1`，加载`course_env.sh`与编译后的overlay，再用`ros2 launch lite3_lab_4_1 experiment.launch.py`启动。创建脚手架成功不代表节点算法、配置或硬件实验已经实现/验证。

## 环境安装、工作空间与构建

目标硬件为Orange Pi AI Pro 20T（aarch64），Ubuntu 22.04、ROS 2 Humble和Python 3。先按[课程环境安装](../../../COURSE_ENVIRONMENT.md)配置ROS软件源、官方Livox SDK2/ROS驱动及官方FAST-LIO ROS2后端；已在`~/devices_driver`安装的驱动直接使用，不重复覆盖。工作空间虽叫`catkin_ws_lite3dog`，采用ROS 2的colcon构建。

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/catkin_ws_lite3dog/src
sudo apt-get update
sudo apt-get install -y ros-humble-desktop python3-colcon-common-extensions \
  python3-numpy python3-yaml python3-pil python3-matplotlib python3-pytest \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan ros-humble-teleop-twist-keyboard
cd ~/catkin_ws_lite3dog
source ./course_env.sh
bash src/lite3_course_native/native/build_backend.sh --test
colcon build --symlink-install \
  --base-paths src/lite3_course_native src/lite3_course_common src/03_sensors src/04_slam \
  --packages-up-to lite3_lab_4_1
source ./course_env.sh
export ROS_DOMAIN_ID=43
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
```

每个新终端都加载`course_env.sh`，再选与真实驱动相同的ROS域；本教程统一使用默认域43；历史只读诊断域44不作为复现依赖。保留已有第六章进程，真机课程不需要安装、启动Gazebo、模型或四足仿真控制器。外部驱动已运行时使用`start_hardware:=false`；没有驱动时改为`true`并提供下面的JSON。`start_runtime:=false`仅适用于完整公共LIO/TF/导航栈已经存在的情况，不能仅凭原始雷达驱动已运行就设为false。

## 真机网络、外参和高度

官方驱动来源为[Livox ROS Driver2](https://github.com/Livox-SDK/livox_ros_driver2)，LIO来源为[FAST-LIO ROS2分支](https://github.com/hku-mars/FAST_LIO/tree/ROS2)。先在第三章确认真实`/livox/lidar`、`/livox/imu`正常；驱动JSON中的设备IP、主机接收IP应与当前网卡地址一致。更换网络后修改本次使用的JSON，再正常重启自己启动的驱动；开发板SSH地址不等同于雷达接收网卡地址，也不要改写机器人控制网段。

下面数值**只是可修改的教学示例，不是本机已完成的装配标定**。启动前用卷尺和轴向检查替换：`lidar_xyz/rpy`表示base_link到雷达光学原点；x前、y左、z上，米/弧度。`initial_body_height`是起始base_link距地面的实测名义高度，不是雷达高度或由LIO测出的绝对高度。MID-360内部IMU到雷达外参由公共后端配置，与机身安装外参分开。

```bash
LIDAR_CONFIG="$HOME/catkin_ws_lite3dog/src/lite3_course_common/config/MID360_config.json"
test -f "$LIDAR_CONFIG"
  # 逐项替换为现场测量；不要把以下示例直接写成标定结果。
LIDAR_XYZ='[0.22,0.0,0.175]'
LIDAR_RPY='[0.0,0.0,0.0]'
BODY_HEIGHT=0.35
ROBOT_LENGTH=1.2
ROBOT_WIDTH=0.8
FOOTPRINT_PADDING=0.15
REAL_ARGS=(mode:=real start_hardware:=true lidar_config:="$LIDAR_CONFIG" \
  lidar_xyz:="$LIDAR_XYZ" lidar_rpy:="$LIDAR_RPY" initial_body_height:="$BODY_HEIGHT" \
  robot_length_m:="$ROBOT_LENGTH" robot_width_m:="$ROBOT_WIDTH" \
  footprint_padding_m:="$FOOTPRINT_PADDING")
```

`robot_length_m/robot_width_m`须覆盖行走时的实际机身与足端包络，padding另加安全裕量；这些参数同步给Nav2及探索半径。将实测值、雷达序列号、JSON、照片、坐标轴示意和测量日期保存在本次结果目录。真机`use_sim_time=false`，所有传感器header与开发板系统时钟采用相同Unix时间域；旧数据刚到达仍是旧数据，不能重打时间戳或放宽阈值。

## 启动与静态检查

```bash
ros2 launch lite3_lab_4_1 experiment.launch.py "${REAL_ARGS[@]}" \
  native_execute:=false enable_real_motion:=false autostart:=false rviz:=true \
  result_dir:=$HOME/catkin_ws_lite3dog/results/4_1_real_run
```

本目录`launch/experiment.launch.py`选择`stack='lio'`。先静置到后端输出稳定。在另一个已加载环境并选域43的终端逐项查看，连续命令用Ctrl+C退出：

```bash
ros2 param get /course_sensor_adapter use_sim_time
ros2 topic hz /course/points_raw
ros2 topic hz /course/imu
ros2 topic hz /course/odom
ros2 topic echo /course/imu --once
cat ~/catkin_ws_lite3dog/results/4_1_real_run/health.json
```

RViz固定坐标系`odom`，添加PointCloud2显示`/course/points_raw`和`/course/lio_points`；检查墙面方向、地面高度与安装方向一致。没有本体时可进行真实传感器静态检查，但不能把静止流量或手拿雷达采集写成机器狗运动验收。

## 手动运动、arm与正常停止

先用`native_execute:=false enable_real_motion:=false`完成静态传感器/LIO检查。需要机器狗连续行走时，按[课程环境中的真实连续后端说明](../../../COURSE_ENVIRONMENT.md)构建并核对独立native_runner；开发板部署路径为`~/catkin_ws_lite3dog/src/lite3_course_native/`。静态launch先正常退出，再以相同实测外参重启，把本课命令中的两个开关改为true，并提供实际二进制哈希：

```bash
NATIVE_SHA=$(sha256sum ~/catkin_ws_lite3dog/src/lite3_course_native/native/native_runner | awk '{print $1}')
ros2 launch lite3_lab_4_1 experiment.launch.py "${REAL_ARGS[@]}" \
  native_execute:=true native_sha256:="$NATIVE_SHA" enable_real_motion:=true \
  autostart:=false result_dir:=$HOME/catkin_ws_lite3dog/results/4_1_real_run
  # 上面launch在前台运行；以下命令在另一个同域终端执行。
ros2 service call /course/native/arm std_srvs/srv/Trigger '{}'
ros2 topic echo /course/native/status --once
ros2 topic echo /course/control/status --once
```

须收到真实`armed=true`及`backend_ready=true`，且LIO/感知新鲜，才开始运动。ARM请求成功不等于机器狗状态已确认；未连接本体时只完成代码/静态传感器工作，不等待或编造运动记录。手动课使用连续Twist键盘，`i`前进、`j/l`转向、`k`停车；节点需要持续新按键，课程出口还会独立限速和检查源数据年龄：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.02 -p turn:=0.02 -r cmd_vel:=/course/cmd_vel
```

结束一项记录时先在键盘按`k`并确认停车，再调用本课结束服务。整套课程退出前先停止键盘/上游目标，再锁闭课程速度并按原native流程交还：

```bash
ros2 service call /course/emergency_stop std_srvs/srv/Trigger '{}'
ros2 service call /course/native/handover std_srvs/srv/Trigger '{}'
ros2 topic echo /course/native/status
```

`/course/emergency_stop`锁闭课程速度出口；不要把它与底层`/course/native/emergency`混淆。按原遥控交还流程完成趴下，确认native `state=CLOSED`后才Ctrl+C关闭launch。中途故障保留原日志；下次启动不要复用旧arm或旧速度。下游从不使用第二章数秒离散动作来假装连续导航。

## 三次闭环记录与保存

在实际地面标记A点和朝向；路线长度至少1m。按上节重新启动允许真实运动的本课launch、完成arm后，每次先清除上轮回到起点的确认，再开始记录：

```bash
ros2 param set /lab_4_1 returned_to_start false
ros2 service call /lab_4_1/start_trial std_srvs/srv/Trigger '{}'
  # 操作者完成同一路线，真实回到标记A及原朝向并停车后：
ros2 param set /lab_4_1 returned_to_start true
ros2 service call /lab_4_1/finish_trial std_srvs/srv/Trigger '{}'
ros2 service call /lab_4_1/save_pcd std_srvs/srv/Trigger '{}'
cat ~/catkin_ws_lite3dog/results/4_1_real_run/pcd_save_result.json
```

重复共3次，失败也调用结束并保留JSON，额外批次换新result_dir。`returned_to_start`是操作者对物理标记的确认，未完成不要设true。服务即时答复只表示异步请求；最终要求`success=true`、`reload_valid=true`和正点数。默认文件`~/catkin_ws_lite3dog/maps/robot_dog_auto_map.pcd`及`.pcd.frame.txt/.json`；本课未运行二维建图，PCD可为`odom`坐标系，不把它直接当Nav2地图。

## Python源码与注释阅读

| 位置/函数 | 如何工作、为何这样写 |
|---|---|
| `core.StreamMonitor.observe` | 比较原header与节点系统时间，拒绝未来/陈旧数据，不拿新callback覆盖旧采集时间 |
| `imu/health` | 只缓存有限SI模长；初始化后持续检查四路超时 |
| `Experiment.on_odom` | 试验期间记录LIO位置、yaw；不订阅仿真真值作为估计 |
| `start_trial/finish_trial` | 清空本轮轨迹，累加真实路程，按物理回归确认计算起终差，保留失败 |
| `core.summarize_trials` | 三次原始记录取中位数；失败不删除，不以均值掩盖失败 |
| `save_pcd/saved` | 调用公共存档服务后重新解析ASCII PCD全部点，不能只检查文件名 |
| `tick/main` | 每秒写health JSON/CSV；退出时关闭记录，默认不自动运动 |

关键实现可从`observe('cloud', stamp, now)`读起：`now-stamp`是源数据年龄；同一个旧stamp重复到达不能代表新测量。`finish_trial`的闭环差来自LIO首末位置，只有确认物理回到标记后才有漂移意义，不代表有独立绝对定位精度。

## 故障排查

IMU模长约1说明仍是g，约96说明重复换算；检查公共适配处，勿在本节点再乘9.81。原始云正常而LIO无输出时查初始化、真实CustomMsg和IMU时间；原始header已旧则先修复驱动积压/系统时钟。点图重影先查外参、时间同步和IMU轴向，不能靠删墙点掩盖。PCD失败读`pcd_save_result.json`和存档TF错误。

# 实验结果

交付`health.json/csv`、`trial_NN.json`、`trajectory_NN.csv`、`trials.json`、`three_trial_summary.json`、PCD及重载结果。报告注明设备、实测外参、驱动/后端版本、初始化时长、三次路程/漂移和源时延中位数。墙钟频率与header年龄分别记录。

当前交付的代码与纯算法检查不等于机器狗实测通过；尚未取得本课完整真机三次路线数据，不预填漂移或精度成绩。可在开发板执行`python3 -m pytest ~/catkin_ws_lite3dog/src/04_slam/4-1_lidar_inertial_mapping/tests -q`；单元测试输入是逻辑夹具，现场结果另存。

