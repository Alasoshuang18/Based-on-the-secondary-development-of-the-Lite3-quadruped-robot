# 实验原理

安装外参由米制平移和xyzw四元数构成。q与-q代表同一旋转，不能逐元素相减；本实验以归一化四元数绝对内积计算角误差`2*acos(abs(dot))`，平移误差用欧氏距离。

本实验主线为真实MID360静态外参检查与有界录包。rosbag2保存消息与时间戳，回放并非重新采样。真实记录包含/livox/lidar逐点时间CustomMsg、/course/points_raw、已换算为SI单位的/course/imu、/tf和/tf_static；real没有仿真clock属于正常现象。仿真录制显式添加--use-sim-time，让bag接收时间也与header同域，不能只录入clock却让录制器使用系统时间。静态TF使用transient_local QoS，使晚加入的录制器也能接收安装外参。代码强制1~30墙秒录制、每缓冲区1~64MiB，默认30墙秒/16MiB；rosbag双缓冲最坏占两倍参数值，另有DDS/SQLite开销。每0.5秒检查磁盘总量，默认512MiB是停止阈值，文件可能多出一个检查周期和缓存的余量。

录包前的输入就绪与录制时长使用不同判断。real要求接收年龄<1墙秒，不依赖/clock；就绪只允许开始记录，不表示原header延迟或标定已通过，须另查3-1/专项诊断。以下sim策略仅供兼容参考：sim要求点云和IMU保留有效且大于零的原始header.stamp，0≤当前同域ROS时间减采样时间≤1.5个ROS秒，同时接收墙钟年龄≤10秒；独立订阅/clock并记录实际推进回调的接收墙钟，必须先收到至少两次clock采样并确认时间增加；最近一次推进接收超过0.5墙秒则阻止开始录包。/clock使用BEST_EFFORT、depth=1，录包tick只读取这个时刻，不把迟到的轮询重新当成时钟推进。real保持原有接收年龄<1墙秒，不要求仿真clock。TF同时可用且连续就绪record_wait_s（默认2墙秒）才启动；bag模式不录包。ROS时钟倒退锁存为错误，重启前先核查单一clock。

可选Fortress参考中，RTF=0.08时，raw的5个仿真Hz可能只有约0.4个墙钟Hz，两帧相隔2.5墙秒但仅约0.2个ROS秒，不能用1墙秒接收年龄认定sim输入已经过期。30墙秒在此RTF下理论上约2.4个ROS秒，但本节点的截止时间从Popen返回开始计算，包含录制器导入、DDS发现和订阅启动等待；资源繁忙时真正写入的消息窗口可能远短于这个估计。上限始终为30墙秒，不为凑够点云而重置计时。不能把慢仿真的录包时长当成等长仿真时长或实时性能通过。

回放不同时输出录入的clock与生成的clock：本launch只回放点云/IMU/TF白名单，由`ros2 bag play --clock`生成唯一时钟。bag模式不会启动Gazebo或真实驱动。

# 实验目的

- 区分“TF存在”与“TF符合实际测量”。
- 录制资源有界、索引完整的短数据集。
- 回放原始点云和IMU，正确管理use_sim_time与单一clock。

# 实验步骤

**从零创建实验包（已有交付目录跳过）。** 以下命令只供本实验目标尚不存在的空位置从零重建教学使用；现有交付目录不要重复创建、嵌套或覆盖。任一保护检查失败就使用现有交付源码，继续本节后面的环境与构建步骤；子shell的`exit 1`只停止本段，不关闭终端。

```bash
(
  set -e
  source ~/catkin_ws_lite3dog/course_env.sh
  mkdir -p ~/catkin_ws_lite3dog/src/03_sensors
  cd ~/catkin_ws_lite3dog/src/03_sensors
  test ! -e lite3_lab_3_2 && test ! -L lite3_lab_3_2 || { printf '%s\n' 'lite3_lab_3_2已存在，停止创建'; exit 1; }
  test ! -e 3-2_frames_and_recording && test ! -L 3-2_frames_and_recording || { printf '%s\n' '3-2_frames_and_recording已存在，停止创建'; exit 1; }
  ros2 pkg create lite3_lab_3_2 --build-type ament_python --license Apache-2.0 \
    --dependencies rclpy sensor_msgs rosgraph_msgs std_msgs geometry_msgs visualization_msgs \
    nav_msgs tf2_ros lite3_course_common launch launch_ros \
    ament_index_python rosbag2_transport ros2bag
  mv -- lite3_lab_3_2 3-2_frames_and_recording
  cd 3-2_frames_and_recording
  mkdir -p launch config tests rviz
)
```

`3-2_frames_and_recording`是外层课程实验目录名，方便按章节阅读；ROS包名、`package.xml`的`<name>`、`setup.py`中的`package_name`、`resource/lite3_lab_3_2`和内层Python模块目录必须仍为合法名字`lite3_lab_3_2`。重命名只改变外层目录，不把数字开头、含连字符的实验目录名写成ROS包名或Python模块名。上面的依赖逐项来自本包实际`package.xml`的ROS依赖；`python3-*`对应的NumPy、YAML等系统/Python依赖按环境安装，不放入这个创建命令。

脚手架只有包清单、安装配置、资源索引和Python模块骨架，尚未实现本实验算法。运行前必须恢复交付的Python源码、`launch/config/tests`、RViz文件及配套`package.xml/setup.py/setup.cfg`。在`setup.py`的`console_scripts`中注册`experiment = lite3_lab_3_2.experiment:main`，并用`data_files`把`resource/lite3_lab_3_2`安装到`share/ament_index/resource_index/packages`，把`package.xml`安装到`share/lite3_lab_3_2`，把`launch/config/rviz`中的资源安装到`share/lite3_lab_3_2/`下对应子目录；保留交付中其它实际入口和数据注册。恢复`launch/experiment.launch.py`的`generate_launch_description()`入口及公共启动调用后，才回到`~/catkin_ws_lite3dog`按后文执行`colcon build --packages-up-to lite3_lab_3_2`，加载`course_env.sh`与编译后的overlay，再用`ros2 launch lite3_lab_3_2 experiment.launch.py`启动。创建脚手架成功不代表节点算法、配置或硬件实验已经实现/验证。

环境为Ubuntu 22.04和ROS 2 Humble，驱动位于`~/devices_driver`。每个终端先加载课程环境，各终端使用同一课程ROS域（默认43，独立测试域44需显式统一）：

```bash
source ~/catkin_ws_lite3dog/course_env.sh
cd ~/catkin_ws_lite3dog
colcon build --symlink-install --base-paths src/lite3_course_native src/lite3_course_common src/03_sensors src/04_slam src/05_navigation --packages-up-to lite3_lab_3_2
source ~/catkin_ws_lite3dog/course_env.sh
```

先在机器狗保持静止、传感器固定的状态完成测量：以实际base_link原点和右手轴为基准，量到雷达光学原点的xyz（米）、安装roll/pitch/yaw（弧度），再量当前base_link到地面的高度；记录量具、日期、安装照片和误差。默认高度、机身包围范围与所有交付坐标都是未标定示例。新终端不会继承上一终端手动设置的变量，下面三项必须输入本机实测值，不能照抄示例：

```bash
read -r -p '实测雷达光学原点xyz，米，输入[x,y,z]：' LIDAR_XYZ
read -r -p '实测安装RPY，弧度，输入[r,p,y]：' LIDAR_RPY
read -r -p '实测base_link离地高度，米：' BODY_HEIGHT
export LIDAR_XYZ LIDAR_RPY BODY_HEIGHT
```

同一组实测值用于公共TF、3-2参考比较和3-3地面高度，避免用两份相同示例“互相校验”。雷达JSON外参保持零。MID360与主机雷达网口使用当前1网段源策略；机器狗运动控制使用独立的192.168.2.1/主机192.168.2.233，不能把eth1雷达改成2网段或占用控制网地址。网络操作只按[驱动指南](../../../DRIVER_SETUP.md)逐项核查，保留现有Wi-Fi SSH与第六章会话。

已有本体URDF或外部节点发布真实安装TF时，在下述launch追加`publish_sensor_tf:=false`，避免重复发布base_link到lidar_link/imu_link；仍须把同一组实测lidar_xyz/rpy和IMU相对安装量传入，用于重力坐标计算。没有既有发布者时保留默认true，由课程按实测参数发布。不要为了本课删除本体URDF或用另一份示例TF覆盖现有标定。公共`lidar_config`默认指向lite3_course_common包的config/MID360_config.json；更换设备/主机地址时按3-1复制独立JSON并以`lidar_config:=绝对路径`指定。

1. **测量外参。** 按右手坐标测量雷达原点相对base_link的x/y/z，填写`config/params.yaml`的lidar_translation_m与lidar_quaternion_xyzw。交付平移[0.22,0,0.175]米和零旋转均为未标定示例，不能当成真实安装尺寸；必须现场测量并填写，也要与公共启动的lidar_xyz/lidar_rpy保持一致。本包参考值只用于核对TF，不发布或修改安装外参。应把实测平移和RPY转换后的xyzw四元数写入本次独立参数文件，再用params_file传入；matches_expected=true仅说明两份配置一致，仍需附独立实测记录，不能用默认示例自证标定。matches_expected=false表示TF与填写值不符，需核对安装尺寸、坐标方向和容差，不能直接判驱动错误。

2. **构建与录制。** 先复制并编辑本次真实外参参考，填写上面独立测得的平移与四元数，保留默认duration_s=30和缓存/磁盘上界。新文件路径通过params_file实际传入：

```bash
cfg_dir="$HOME/catkin_ws_lite3dog/real_configs"
mkdir -p "$cfg_dir"
FRAMES_CFG="$(mktemp --suffix=.yaml "$cfg_dir/lab3_2_frames.XXXXXX")"
cp "$(ros2 pkg prefix --share lite3_lab_3_2)/config/params.yaml" "$FRAMES_CFG"
${EDITOR:-nano} "$FRAMES_CFG"
export FRAMES_CFG
```

实测RPY转xyzw可用同一终端的纯数学工具，输出后填入该文件，不从现有TF反推冒充实测：`python3 -c 'import os,yaml; from lite3_course_common.geometry import rpy_quaternion; print(rpy_quaternion(yaml.safe_load(os.environ["LIDAR_RPY"])).tolist())'`。

```bash
ros2 launch lite3_lab_3_2 experiment.launch.py mode:=real native_execute:=false autostart:=true rviz:=true \
  lidar_xyz:="$LIDAR_XYZ" lidar_rpy:="$LIDAR_RPY" initial_body_height:="$BODY_HEIGHT" \
  params_file:="$FRAMES_CFG" \
  result_dir:=$HOME/catkin_ws_lite3dog/results/lab_3_2/run1
```

录制入口没有单独的start服务，必须显式autostart:=true（只启动只读录包，不启用运动）。节点等待点云、IMU和三条TF就绪；sim还要确认ROS时钟正在推进。按上述模式对应的租约连续就绪2墙秒后启动录制器，默认30墙秒到时自动SIGINT保存metadata。可在本次使用的参数文件中缩短duration_s，节点仍强制最多30墙秒。节点不会反复创建新bag。本次为静态只读采集，不启用运动；已有真实运行时可加start_runtime=false，使用独立目录。提前结束按Ctrl+C，节点仍会等待录制器关闭。

```bash
ros2 bag info ~/catkin_ws_lite3dog/results/lab_3_2/run1/raw_bag
cat ~/catkin_ws_lite3dog/results/lab_3_2/run1/record_result.json
```

3. **停止本次真实采集后离线回放。**

```bash
ros2 launch lite3_lab_3_2 experiment.launch.py mode:=bag bag_source:=real rviz:=true \
  bag_path:=$HOME/catkin_ws_lite3dog/results/lab_3_2/run1/raw_bag \
  playback_rate:=0.5 \
  result_dir:=$HOME/catkin_ws_lite3dog/results/lab_3_2/replay1
```

播放器运行时，另开同域课程终端只读核对唯一clock：

```bash
ros2 topic info /clock -v
```

bag_path必须是包含metadata.yaml的目录。默认单次播放，播完停住正常；播放器不使用--delay，因为板端Humble在延迟阶段clock先走、起播时又跳回，实测出现约0.109秒回拨。短包应先启动订阅节点再由外部播放器播放（play_bag:=false），而非延迟或重打原时间。倍率允许0.05~4倍，预读队列为100条。若外部已启动播放器，设play_bag=false，只起检查节点。外部播放器也必须排除录入的clock消息，再生成唯一clock。真实硬件驱动停止后才回放；回放域中只保留播放器生成的clock，不能同时启动第二个时钟源。

4. **逐函数读源码。** `algorithms.py::recording_limits`夹紧时长与缓存；`recording_clock_arguments`仅为仿真加入--use-sim-time；`quaternion_error`处理归一化、正负号等价与非法四元数。`RecordingReadiness`保存各流的原始采样时间和接收单调墙钟；observe_clock只在真实/clock回调中更新推进接收时刻，status每tick只读并检查模式对应的租约及停钟年龄；它只判断能否开始，不改变录制时长和资源上界。`experiment.py::cloud_ready/imu_ready`同时保存消息原始stamp与墙钟接收时间，不以收到消息的时刻替换stamp。`check_frames`查base_link<-lidar_link、base_link<-imu_link、base_footprint<-base_link，并输出实际数值、参考误差或明确错误。

`tick`用STEADY_TIME墙钟，仿真暂停也会检查截止时间：先写TF证据，再评估输入/clock租约和连续就绪等待，启动录制后仍按墙钟检查时长、进程与磁盘。recording_readiness.json记录各输入的ROS/墙钟年龄、拒绝原因、clock状态、TF状态和已等待墙秒，便于区分慢仿真、真断流与停钟。`start_recording`使用参数列表调用subprocess，不经shell；包目录存在时拒绝覆盖；通过ament包share定位bag_qos.yaml。`stop_recording`只向自己创建的进程组发SIGINT，等10秒写完索引；超时才升级终止。关闭后用PyYAML读取真实metadata，由`algorithms.py::recording_metadata`核对持续时间、每个话题消息数和总数，连同退出码、墙钟用时、缓存和请求时长写入record_result.json。只有exit0、内容结构有效、持续时间大于0且点云和IMU消息数均大于0，`valid_recording`才为true；它还没有验证回放。节点绝不pkill所有ROS进程，`main`的finally负责关包。

`launch/experiment.launch.py::playback`验证目录/倍率，topic白名单在bag_source=real时包含/livox/lidar、course点云、SI IMU、tf、tf_static，排除录入的clock，再由播放器生成clock。`config/bag_qos.yaml`显式设置CustomMsg reliable/depth5、课程点云best_effort、IMU reliable与静态TF transient_local。公共launch处理模式、use_sim_time、runtime、参数文件和结果路径。

5. **排查。** 没有raw_bag先看frames.json和recording_readiness.json：missing表示缺输入，ros_stale/future_stamp表示采样过期或时钟域问题，wall_hard_limit表示超过10墙秒硬限，clock_not_advancing/clock_stalled表示不能开始录包的停钟状态；同时核对点云/IMU与单一clock；没有metadata先看rosbag_record.log、返回码和磁盘空间。录制时间/总大小达到阈值会提前或到时正常关包。回放缺TF看bag是否有tf_static以及QoS；时间外推错误看是否存在第二个clock。每次录制/回放都使用独立结果目录，按Ctrl+C停止。


**可选兼容参考。** 只有主动研究旧Fortress参考时才显式使用`mode:=sim motion_backend:=gait`；它不替代本章真机验收，不与real采集同时启动。真实录包回放用`mode:=bag bag_source:=real`及唯一播放器。先结束自己启动的当前实例，再选参考模式；不要结束其它课程进程。

# 实验结果

本轮实际执行状态及资源限制见[验证记录](../../../VERIFICATION.md)。下面的预期现象和判定阈值不代表已经取得该成绩。

录包启动后生成raw_bag目录、record_command.json、record_result.json、rosbag_record.log；frames.json与recording_readiness.json保留TF和录包前置条件证据。先检查`valid_recording`、`topic_message_counts`、`bag_duration_s`和`validation_errors`，再用ros2 bag info核对/livox/lidar、课程点云、SI IMU与TF实际消息数；真实raw CustomMsg缺失时不能称已具备后续真实FAST_LIO复算输入。metadata_exists=true和returncode=0本身不能证明录到了点云；`no_pointcloud_messages`表示本次没有有效点云数据，必须保留失败结果并在新的目录重新录制。真实模式没有仿真clock属于正常现象。通过录制内容校验后，还须实际回放检查消息与TF，不能把valid_recording=true当作回放或SLAM通过。

独立录制run1、run2、run3三次，保留三份原bag，再分别回放比较数量、持续时间与外参。未实际录制不能声称bag通过。本地纯测试验证资源限幅、四元数误差、慢RTF下连续就绪、ROS采样过期/未来/无效时间戳、10墙秒硬限、real的1墙秒租约、缺输入、bag禁录、sim停钟/倒退阻止开始录制，以及录包轮询不能为已暂停的clock续租：`python3 -m pytest src/03_sensors/3-2_frames_and_recording/tests -q`。

录制行为依据[rosbag2官方Humble实现](https://github.com/ros2/rosbag2/tree/humble)。


