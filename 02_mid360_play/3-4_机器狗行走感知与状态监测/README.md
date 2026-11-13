# 实验原理

本实验默认真机静态只读监测，不发布运动候选，不自动站立或arm。现场明确启用后，有限序列为3个ROS秒零速观察、3个ROS秒0.02m/s直行、3个ROS秒0.02rad/s左转、2个ROS秒停止，随后持续发送零速。请求值与课程出口默认限幅一致。真实序列要等公共反馈backend_ready=true（来自真实非mock native会话）且点云、IMU控制租约有效才开始；bootstrap期间不会提前消耗动作时间。候选只发/course/cmd_vel，由公共唯一适配器仲裁和检查超时。

“请求阶段”与“实际动作状态”分开记录。零速度观察不能替代硬件站立服务，没有实际反馈就不能声称机器狗站起。动作进度与默认15秒记录窗口使用ROS时钟。监测中的传感器频率、接收年龄和断流报警、后端状态新鲜度与总等待上限仍使用单调墙钟；延迟使用当前ROS时间减原始采样时间。统计含滑窗Hz、平均/最大延迟、年龄、时间戳倒退和实时因子RTF。仿真真值姿态单独标记为评估，不能冒充LIO里程计。

控制租约单独判断：sim要求同一ROS时钟域的原始stamp有效且0≤ROS采样年龄≤stale_timeout_s（默认0.6个ROS秒），同时接收墙钟年龄≤10秒；real继续要求接收墙钟年龄≤stale_timeout_s（默认0.6墙秒），并拒绝旧采样、未来或无效stamp。bag只监测。控制租约有效不代表传感器实时性能合格，监测报警不会因此消失。

以下RTF与停钟说明仅供bag/显式sim兼容参考，真机使用系统ROS时间且use_sim_time=false，实时性能仍以真实墙钟Hz、原header年龄和端到端延迟验收。RTF是同一观测窗口的ROS时间增量/墙钟时间增量。构造节点时若ROS时间为0，表示尚未取得有效clock观测，此时ros_elapsed_s与real_time_factor为null；首个实际非零ROS观测及其墙钟时刻才建立RTF起点。若加入仿真时它已运行55.8秒，这55.8秒不能算作本节点观测到的时间增量。JSON的clock_ros_start_s记录该首值，clock_wall_elapsed_s给出RTF实际分母，wall_elapsed_s仍从程序构造起累计用于总等待预算。首个clock观测时分母为0，RTF仍为null；之后clock暂停可如实得到RTF=0。该观测起点独立于backend_ready及动作起点，后端等待期也计入真实RTF；纯监测autostart=false同样建立观测窗口。例如RTF=0.08时，3个仿真秒需要约37.5个墙钟秒，2.5墙秒的点云间隔约对应0.2个ROS秒；此时控制输入可仍有效，但实测0.4墙钟Hz与超过0.6墙秒的接收年龄仍会记录low_frequency/stale，不能算实时通过。仿真clock暂停时动作进度暂停，本节点候选在有界租约内可能继续存在；实际出口由公共适配器的0.5墙秒停钟看门狗归零。输入超出租约或后端失联也会归零。ROS时钟倒退会锁存停止，不重新播放动作。

# 实验目的

- 在站立、直行、转弯期间观察点云和IMU。
- 定位低频、延迟、断流及时间戳问题。
- 用请求、反馈和姿态分别佐证运行状态。

# 实验步骤

**从零创建实验包（已有交付目录跳过）。** 以下命令只供本实验目标尚不存在的空位置从零重建教学使用；现有交付目录不要重复创建、嵌套或覆盖。任一保护检查失败就使用现有交付源码，继续本节后面的环境与构建步骤；子shell的`exit 1`只停止本段，不关闭终端。

```bash
(
  set -e
  source ~/catkin_ws_lite3dog/course_env.sh
  mkdir -p ~/catkin_ws_lite3dog/src/03_sensors
  cd ~/catkin_ws_lite3dog/src/03_sensors
  test ! -e lite3_lab_3_4 && test ! -L lite3_lab_3_4 || { printf '%s\n' 'lite3_lab_3_4已存在，停止创建'; exit 1; }
  test ! -e 3-4_motion_monitoring && test ! -L 3-4_motion_monitoring || { printf '%s\n' '3-4_motion_monitoring已存在，停止创建'; exit 1; }
  ros2 pkg create lite3_lab_3_4 --build-type ament_python --license Apache-2.0 \
    --dependencies rclpy sensor_msgs std_msgs geometry_msgs visualization_msgs \
    nav_msgs tf2_ros tf2_msgs lite3_course_common launch launch_ros \
    ament_index_python
  mv -- lite3_lab_3_4 3-4_motion_monitoring
  cd 3-4_motion_monitoring
  mkdir -p launch config tests rviz
)
```

`3-4_motion_monitoring`是外层课程实验目录名，方便按章节阅读；ROS包名、`package.xml`的`<name>`、`setup.py`中的`package_name`、`resource/lite3_lab_3_4`和内层Python模块目录必须仍为合法名字`lite3_lab_3_4`。重命名只改变外层目录，不把数字开头、含连字符的实验目录名写成ROS包名或Python模块名。上面的依赖逐项来自本包实际`package.xml`的ROS依赖；`python3-*`对应的NumPy、YAML等系统/Python依赖按环境安装，不放入这个创建命令。

脚手架只有包清单、安装配置、资源索引和Python模块骨架，尚未实现本实验算法。运行前必须恢复交付的Python源码、`launch/config/tests`、RViz文件及配套`package.xml/setup.py/setup.cfg`。在`setup.py`的`console_scripts`中注册`experiment = lite3_lab_3_4.experiment:main`，并用`data_files`把`resource/lite3_lab_3_4`安装到`share/ament_index/resource_index/packages`，把`package.xml`安装到`share/lite3_lab_3_4`，把`launch/config/rviz`中的资源安装到`share/lite3_lab_3_4/`下对应子目录；保留交付中其它实际入口和数据注册。恢复`launch/experiment.launch.py`的`generate_launch_description()`入口及公共启动调用后，才回到`~/catkin_ws_lite3dog`按后文执行`colcon build --packages-up-to lite3_lab_3_4`，加载`course_env.sh`与编译后的overlay，再用`ros2 launch lite3_lab_3_4 experiment.launch.py`启动。创建脚手架成功不代表节点算法、配置或硬件实验已经实现/验证。

先在机器狗保持静止、传感器固定的状态完成测量：以实际base_link原点和右手轴为基准，量到雷达光学原点的xyz（米）、安装roll/pitch/yaw（弧度），再量当前base_link到地面的高度；记录量具、日期、安装照片和误差。默认高度、机身包围范围与所有交付坐标都是未标定示例。新终端不会继承上一终端手动设置的变量，下面三项必须输入本机实测值，不能照抄示例：

```bash
read -r -p '实测雷达光学原点xyz，米，输入[x,y,z]：' LIDAR_XYZ
read -r -p '实测安装RPY，弧度，输入[r,p,y]：' LIDAR_RPY
read -r -p '实测base_link离地高度，米：' BODY_HEIGHT
export LIDAR_XYZ LIDAR_RPY BODY_HEIGHT
```

同时测量含足腿的水平完整包络，长度和宽度不能只量躯干外壳；以base_link为中心取能覆盖实际全部包络的对称外包矩形，若原点偏心则相应扩大。记录padding选择依据和实际制动/转向空间，交付1.2×0.8m与0.15m仅为未测示例，不能直接当本机尺寸：

```bash
read -r -p '实测对称包络长度，米：' BODY_LENGTH
read -r -p '实测对称包络宽度，米：' BODY_WIDTH
read -r -p '本次已核对的保守包络余量，米：' FOOTPRINT_PADDING
export BODY_LENGTH BODY_WIDTH FOOTPRINT_PADDING
```

同一组实测值用于公共TF、3-2参考比较和3-3地面高度，避免用两份相同示例“互相校验”。雷达JSON外参保持零。MID360与主机雷达网口使用当前1网段源策略；机器狗运动控制使用独立的192.168.2.1/主机192.168.2.233，不能把eth1雷达改成2网段或占用控制网地址。网络操作只按[驱动指南](../../../DRIVER_SETUP.md)逐项核查，保留现有Wi-Fi SSH与第六章会话。

已有本体URDF或外部节点发布真实安装TF时，在下述launch追加`publish_sensor_tf:=false`，避免重复发布base_link到lidar_link/imu_link；仍须把同一组实测lidar_xyz/rpy和IMU相对安装量传入，用于重力坐标计算。没有既有发布者时保留默认true，由课程按实测参数发布。不要为了本课删除本体URDF或用另一份示例TF覆盖现有标定。公共`lidar_config`默认指向lite3_course_common包的config/MID360_config.json；更换设备/主机地址时按3-1复制独立JSON并以`lidar_config:=绝对路径`指定。

按3-3的具体复制命令先创建独立参数文件，按本机实测高度填ground_z_m、按实测本体/腿部范围填body_min_m/body_max_m，静止摆放箱子检查去地面结果。填写后在本终端指定其绝对路径，后续两种启动方式都用同一文件：

```bash
read -r -p '已填写实测高度/包络的3-3参数文件绝对路径：' PERCEPTION_CFG
export PERCEPTION_CFG
test -f "$PERCEPTION_CFG"
```

环境为Ubuntu 22.04、ROS 2 Humble；真机驱动在~/devices_driver。每个终端先加载课程环境，使用同一课程ROS域（默认43，独立真机测试域44须所有终端一致）：

```bash
source ~/catkin_ws_lite3dog/course_env.sh
cd ~/catkin_ws_lite3dog
colcon build --symlink-install --base-paths src/lite3_course_native src/lite3_course_common src/03_sensors src/04_slam src/05_navigation --packages-up-to lite3_lab_3_4
source ~/catkin_ws_lite3dog/course_env.sh
ros2 launch lite3_lab_3_4 experiment.launch.py mode:=real autostart:=false enable_real_motion:=false native_execute:=false rviz:=true \
  lidar_xyz:="$LIDAR_XYZ" lidar_rpy:="$LIDAR_RPY" initial_body_height:="$BODY_HEIGHT" \
  robot_length_m:="$BODY_LENGTH" robot_width_m:="$BODY_WIDTH" footprint_padding_m:="$FOOTPRINT_PADDING" \
  processor_params_file:="$PERCEPTION_CFG" \
  result_dir:=$HOME/catkin_ws_lite3dog/results/lab_3_4/run1
```

1. 先执行上述real静态监测入口。它只订阅实际MID360、姿态/里程计与控制反馈，CSV记录现场静止状态；/course/odom没有发布者时如实标missing。使用原遥控维持已确认的合法站立状态，不能用本实验的stand阶段替代站立服务。新真机安装必须先完成3-1连接/标定记录、3-2外参核查和3-3去地面验证。后台运行本实验不会替操作者建立真实运动会话。

真实连续后端已经完成编译与无socket mock接口测试，默认仍为native_execute=false，尚未做现场运动联调。启用真实主动序列必须同时满足：参数文件allow_real_motion=true、autostart=true、公共launch显式enable_real_motion=true、native_execute=true，以及人工调用/course/native/arm后实际状态armed=true。完整接口、会话租约、互斥锁和交还规则见[部署后的真实连续后端说明](../../lite3_course_native/README.md)；本地交付对应[后端源码说明](../../../native_course_control/README.md)。没有后台自动arm，没有启动即站立；arm仅接受已站立且遥测通过的机器狗。当前未执行这些现场动作。

这个实验只发布连续候选，公共出口送往/course/cmd_vel_checked，再由独立native状态机执行，不会把Twist转换成第二章1~3秒离散动作。停止时应先按后端说明handover并确认原遥控趴下/CLOSED，再结束launch。编译/mock通过不能证明实际直行、转弯和遥控优先级通过。

**现场主动阶段：仅在上述真实监测、原始时间戳、去地面、控制网和遥测健康全部确认后执行。** 先停止本次静态launch，保留证据；不要与第二章会话、另一份native后端或另一主动实验同时运行。控制网只做只读路由核对，不修改雷达1网段：

```bash
ip -br -4 addr
ip route get 192.168.2.1 from 192.168.2.233
```

从交付默认配置创建本次独立主动参数，只有这份新文件将allow_real_motion设为true；默认交付文件仍为false。下列命令只准备配置，尚未arm：

```bash
cfg_dir="$HOME/catkin_ws_lite3dog/real_configs"
mkdir -p "$cfg_dir"
motion_cfg="$(mktemp --suffix=.yaml "$cfg_dir/lab3_4_active.XXXXXX")"
cp "$(ros2 pkg prefix --share lite3_lab_3_4)/config/params.yaml" "$motion_cfg"
python3 - "$motion_cfg" <<'PYCFG'
from pathlib import Path
import sys, yaml
p = Path(sys.argv[1])
data = yaml.safe_load(p.read_text())
data['/**']['ros__parameters']['allow_real_motion'] = True
p.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
PYCFG
runner="$(python3 -c 'from lite3_course_native.adapter import bundled_runner; print(bundled_runner())')"
runner_sha="$(sha256sum "$runner" | cut -d ' ' -f 1)"
ros2 launch lite3_lab_3_4 experiment.launch.py mode:=real motion_backend:=native \
  autostart:=true enable_real_motion:=true native_execute:=true native_sha256:="$runner_sha" \
  params_file:="$motion_cfg" processor_params_file:="$PERCEPTION_CFG" \
  lidar_xyz:="$LIDAR_XYZ" lidar_rpy:="$LIDAR_RPY" initial_body_height:="$BODY_HEIGHT" \
  robot_length_m:="$BODY_LENGTH" robot_width_m:="$BODY_WIDTH" footprint_padding_m:="$FOOTPRINT_PADDING" \
  result_dir:="$HOME/catkin_ws_lite3dog/results/lab_3_4/real_active_$(date +%Y%m%d_%H%M%S)"
```

`enable_real_motion`是公共运动总开关，也传给实验的allow_real_motion；它默认false并优先于参数文件，单改YAML不会启用。只有上面显式true并满足其余条件才可进入主动阶段。`bundled_runner()`只解析当前包实际二进制路径，不创建控制会话；记录SHA256以便核对实际使用版本。新终端加载同一课程环境和ROS域后，先确认/course/native/status为mock=false、execute=true、尚未armed；原遥控与遥测必须确认已经站立且模式满足原生守卫，再由现场操作者显式调用：

```bash
ros2 service call /course/native/arm std_srvs/srv/Trigger '{}'
ros2 topic echo /course/native/status --once
ros2 topic echo /course/control/status --once
```

arm服务success只代表请求被接受，必须实际看到native状态armed=true、mock=false，以及公共control_ready/backend_ready=true且传感器原header未超时，才进入本次主动教学。候选经/course/cmd_vel_checked连续更新，前台会话/遥测/模式/接收租约任一失效都会停车或交还；不能让旧点云靠新接收时间续租。此处仅给出现场步骤，本交付不声称已经完成真实运动或遥控接管验收。

**停止与交还。** 先锁存公共零输出，再向native请求正常交还；按照原遥控规则让机器狗趴下，反复查看状态直到CLOSED，之后才Ctrl+C结束launch：

```bash
ros2 service call /course/emergency_stop std_srvs/srv/Trigger '{}'
ros2 service call /course/native/handover std_srvs/srv/Trigger '{}'
ros2 topic echo /course/native/status --once
```

一次echo不保证已经关闭，必须核对状态CLOSED和原遥控实际反馈；不能用kill进程代替交还。公共停止锁存需显式release_stop才可解除，解除后仍要求新候选和全部健康条件。真实遥控同模式摇杆是否优先接管不能从现遥测字段自动证明，必须按native说明现场验证；没有证据就保持默认只读模式。

2. 在另一个加载课程环境的终端观察。以下命令逐条执行；tf2_echo和hz会持续运行，观察后仅Ctrl+C结束该检查，再执行下一条。只读默认/course/cmd_vel没有本实验候选，timeout到期无输出是正常现象：

```bash
ros2 topic echo /course/motion_monitor/status --once
ros2 topic echo /course/control/status --once
ros2 run tf2_ros tf2_echo base_footprint base_link
timeout 2s ros2 topic echo /course/cmd_vel --once
ros2 topic hz /course/imu
```

站立时观察平面与姿态稳定性；直行时场景相对机身后移；转弯时点云方位改变。动作反馈缺失时显示unknown。/course/odom尚无LIO发布者时missing属于预期，原始传感器统计仍有效。

3. 逐函数读algorithms.py。WindowMetrics使用最多300样本的deque，累计总数和时间戳倒退次数。summary以(N-1)/(末次-首次接收时间)计算Hz，避免把启动等待算入采样率。motion_command是无ROS的有限时间表，11秒后一直complete与零速。

4. 逐函数读experiment.py。receive同时记录墙钟与ROS同域延迟。imu仅在orientation_covariance首项非负时使用姿态；真实MID360无姿态估计，所以没有LIO里程计时attitude_rpy_rad可以为null，不能用identity冒充水平。posture独立订阅真实/tf中的base_footprint→base_link，JSON的body_tilt_tf只记录roll/pitch、原stamp、来源及两种年龄/fresh，不给出绝对yaw；无LIO时该TF来自公共适配器的低动态重力估计，受安装标定和运动比力影响，不是完整惯导。odom更新里程计姿态；truth只写评估字段。action保存公共反馈，backend_is_ready还检查墙钟年龄，control_input_status调用纯函数sample_freshness，分别复核点云/IMU原始ROS采样年龄与接收墙钟租约。enabled区分sim、real显式开关和bag只监测。control由STEADY_TIME驱动，调用MotionTimeline，动作阶段读取ROS进度；输入租约过期、后端丢失、时钟倒退或总墙钟超限时发零速，公共出口另行处理clock暂停。

algorithms.py中的MotionTimeline分别记录程序构造墙钟起点、首个有效非零ROS观测及对应墙钟起点、后端/输入都就绪后的动作序列起点。_observe_clock由update和snapshot调用，先收到clock但尚未backend_ready时也能统计RTF；不得拿动作首次开始时间替代首次clock观测。update按ROS秒推进有限动作，recording_done仍在序列15个ROS秒后关闭CSV，或包括无clock等待和bootstrap在内总墙钟达到max_wall_duration_s后关闭。snapshot的ros_elapsed_s从有效观测起累计，real_time_factor只除以clock_wall_elapsed_s；未有有效观测时二者为null。sequence_ros_elapsed_s和sequence_wall_elapsed_s继续只描述动作序列。达到总上限后不会在无CSV的情况下再启动动作。

report每墙钟秒写JSON与CSV，照实标注missing、stale、low_frequency、high_latency、clock_domain_or_future_stamp和stamp_regression；JSON另有control_input_freshness，包含各输入的控制判定、原因、ROS年龄和墙钟年龄，不替代原始监测指标。CSV默认等待最长1800墙钟秒，序列开始后记录15个ROS秒；11个ROS秒后控制已完成并持续发零速，CSV继续记录到15秒。关闭CSV不会关闭安全零速定时器。close在ROS可用时发零速并关文件。

5. config/params.yaml中的record_duration_s单位为序列ROS秒，max_wall_duration_s单位为包括bootstrap的总墙钟秒，默认1800；backend_status_timeout_s默认1.5墙钟秒。minimum_cloud_hz/minimum_imu_hz是接收墙钟Hz报警阈值，不修改设备频率；max_latency_s单位为ROS同域秒。stale_timeout_s仍是墙钟stale报警阈值；控制时sim使用同数值的ROS秒并额外限制接收墙钟年龄≤10秒，real仍使用0.6墙秒租约。慢RTF/半速bag会降低墙钟Hz，必须保留实际RTF、帧间隔和全部报警，不能将此解释成实时性能通过，也不能下调minimum_cloud_hz等频率判定阈值来掩盖资源限制。公共launch支持params_file、result_dir、start_runtime，bag模式只监测已有播放器。

静态/回放按Ctrl+C停止；主动会话先停止、交还并确认CLOSED，再结束launch。一直waiting先查点云/IMU与ROS域；有请求但不动先查control/status的real_motion_disabled、传感器原header年龄和真实后端状态；显式sim参考再查clock是否暂停；负延迟先查mode/use_sim_time。每次重启才重置序列，使用run1/run2/run3独立目录。


**真实短包复查。** 本实验JSON/CSV不代替原始bag。静态监测期间由3-2同域以start_runtime=false复用现有唯一传感器运行时，按3-2实测参考YAML录制最多30墙秒。结束本次真实运行时后，由3-2启动唯一真实bag播放器，再在本实验终端仅做监测：

```bash
ros2 launch lite3_lab_3_4 experiment.launch.py mode:=bag bag_source:=real \
  start_runtime:=false autostart:=false rviz:=true
```

bag不启动native、不发送候选，原TF/IMU和点云由播放器提供；低倍率下墙钟Hz下降须如实记录。

**可选兼容参考。** 只有主动研究旧Fortress参考时才显式使用`mode:=sim motion_backend:=gait`；它不替代本章真机验收，不与real采集同时启动。真实录包回放用`mode:=bag bag_source:=real`及唯一播放器。先结束自己启动的当前实例，再选参考模式；不要结束其它课程进程。

# 实验结果

本轮实际执行状态及资源限制见[验证记录](../../../VERIFICATION.md)。下面的预期现象和判定阈值不代表已经取得该成绩。

monitor.csv和monitor.json记录程序总墙钟、首次有效clock观测后的ROS增量、序列墙钟/ROS进度、请求阶段、实际output_state、backend_ready、运动开关、各流Hz、延迟、年龄、姿态及异常。JSON另含clock_ros_start_s、RTF分母clock_wall_elapsed_s和实时因子RTF。首次clock前JSON的ROS增量/RTF为null，CSV对应ROS增量单元格为空；不能用0或加入仿真前的历史时间冒充有效观测。既有实测报告保留其原始数值，旧版零clock起点导致的RTF虚高应在分析中说明，不能回写修饰原证据。独立重复3次并比较站立、直行、转弯期间最大延迟和断流；requested_phase=straight不等于机器人已移动，还要结合输出状态、公共反馈与评估姿态。Fortress真值只属可选参考评估，不得计入本章真实站立、直行或转弯验收。

本地纯算法测试验证滑窗、时间戳倒退、有限动作、慢RTF双时钟、零clock等待/晚加入仿真/纯监测/后端等待期的RTF观测锚定、控制租约与墙钟报警分开、ROS采样过期/未来/无效时间戳、10墙秒硬限、real墙钟租约、后端等待、长bootstrap记录、总墙钟上限和输入/后端丢失清零：`python3 -m pytest src/03_sensors/3-4_motion_monitoring/tests -q`。未在指定后端执行不能声称实际站立、直行或转弯通过。
