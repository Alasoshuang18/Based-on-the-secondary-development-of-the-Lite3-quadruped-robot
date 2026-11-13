# 实验原理

MID360用激光回波生成点云。ROS 2的`PointCloud2`包含XYZ及可选的强度、时间等字段；XYZ单位为米，坐标属于`header.frame_id`。本实验原样转发原始消息至`/course/mid360/preview`，统计时只读XYZ，不删掉原消息字段。

数据路径是“以太网设备 → 官方Livox驱动 → 课程适配器 → `/course/points_raw` → 本节点/RViz”。安装外参由`base_link -> lidar_link`及`base_link -> imu_link`表示；机身姿态由`base_footprint -> base_link`表示。正x向前、正y向左、正z向上。RViz依靠采样时刻的TF把点云放到固定坐标，不能只把frame名称改成base_link。

# 实验目的

- 学会区分网络、驱动、话题、TF和显示问题。
- 识别空点云、NaN、过期消息与错误外参。
- 用JSON和RViz截图保留可核查的诊断证据。

# 实验步骤

**从零创建实验包（已有交付目录跳过）。** 下列命令仅用于本实验目标尚不存在的空位置，帮助理解包结构；现有交付目录不要重复创建、嵌套或覆盖。保护检查失败时直接使用现有交付源码，继续后面的环境与构建步骤。子shell中的`exit 1`只停止本段，不关闭终端。

```bash
(
  set -e
  source ~/catkin_ws_lite3dog/course_env.sh
  mkdir -p ~/catkin_ws_lite3dog/src/03_sensors
  cd ~/catkin_ws_lite3dog/src/03_sensors
  test ! -e lite3_lab_3_1 && test ! -L lite3_lab_3_1 || { printf '%s\n' 'lite3_lab_3_1已存在，停止创建'; exit 1; }
  test ! -e 3-1_mid360 && test ! -L 3-1_mid360 || { printf '%s\n' '3-1_mid360已存在，停止创建'; exit 1; }
  ros2 pkg create lite3_lab_3_1 --build-type ament_python --license Apache-2.0 \
    --dependencies rclpy sensor_msgs std_msgs geometry_msgs visualization_msgs \
    nav_msgs tf2_ros lite3_course_common launch launch_ros ament_index_python
  mv -- lite3_lab_3_1 3-1_mid360
  cd 3-1_mid360
  mkdir -p launch config tests rviz
)
```

`3-1_mid360`是课程的外层实验目录名；ROS包名、`package.xml`的`<name>`、`setup.py`的`package_name`、`resource/lite3_lab_3_1`和内层Python模块目录仍须保持`lite3_lab_3_1`，不能改成含数字开头和连字符的实验目录名。以上ROS依赖来自本包实际`package.xml`；NumPy等`python3-*`系统依赖按环境安装，不塞进`ros2 pkg create --dependencies`。

创建命令只产生脚手架，尚未实现雷达诊断算法。要运行本课，必须把交付的Python源码、配置、launch、测试及RViz文件恢复到对应位置，并采用交付的`setup.py/setup.cfg/package.xml`。`setup.py`的`console_scripts`须注册`experiment = lite3_lab_3_1.experiment:main`；`data_files`须把`resource/lite3_lab_3_1`安装到`share/ament_index/resource_index/packages`，把`package.xml`安装到`share/lite3_lab_3_1`，把`launch/config/rviz`资源安装到`share/lite3_lab_3_1/`下对应子目录。恢复`launch/experiment.launch.py`及其`generate_launch_description()`公共启动入口后，才可回到`~/catkin_ws_lite3dog`执行后面的`colcon build --packages-up-to lite3_lab_3_1`，并通过`ros2 launch lite3_lab_3_1 experiment.launch.py`启动；脚手架创建成功不代表算法已实现或设备已验证。

环境为Ubuntu 22.04、ROS 2 Humble、Python 3和NumPy。真机驱动安装在`~/devices_driver`，由课程环境加载。每个终端先执行：

```bash
source ~/catkin_ws_lite3dog/course_env.sh
cd ~/catkin_ws_lite3dog
colcon build --symlink-install --base-paths src/lite3_course_native src/lite3_course_common src/03_sensors src/04_slam src/05_navigation --packages-up-to lite3_lab_3_1
source ~/catkin_ws_lite3dog/course_env.sh
```

课程环境默认ROS_DOMAIN_ID=43，各终端必须一致；本轮独立真机探针曾用44，复现时在source之后显式export ROS_DOMAIN_ID=44并让所有相关节点同域。IGN_PARTITION仅影响可选仿真，不是连接硬件的条件。真机use_sim_time=false，不运行Gazebo或仿真clock。

先在机器狗保持静止、传感器固定的状态完成测量：以实际base_link原点和右手轴为基准，量到雷达光学原点的xyz（米）、安装roll/pitch/yaw（弧度），再量当前base_link到地面的高度；记录量具、日期、安装照片和误差。默认高度、机身包围范围与所有交付坐标都是未标定示例。新终端不会继承上一终端手动设置的变量，下面三项必须输入本机实测值，不能照抄示例：

```bash
read -r -p '实测雷达光学原点xyz，米，输入[x,y,z]：' LIDAR_XYZ
read -r -p '实测安装RPY，弧度，输入[r,p,y]：' LIDAR_RPY
read -r -p '实测base_link离地高度，米：' BODY_HEIGHT
export LIDAR_XYZ LIDAR_RPY BODY_HEIGHT
```

同一组实测值用于公共TF、3-2参考比较和3-3地面高度，避免用两份相同示例“互相校验”。雷达JSON外参保持零。MID360与主机雷达网口使用当前1网段源策略；机器狗运动控制使用独立的192.168.2.1/主机192.168.2.233，不能把eth1雷达改成2网段或占用控制网地址。网络操作只按[驱动指南](../../../DRIVER_SETUP.md)逐项核查，保留现有Wi-Fi SSH与第六章会话。

已有本体URDF或外部节点发布真实安装TF时，在下述launch追加`publish_sensor_tf:=false`，避免重复发布base_link到lidar_link/imu_link；仍须把同一组实测lidar_xyz/rpy和IMU相对安装量传入，用于重力坐标计算。没有既有发布者时保留默认true，由课程按实测参数发布。不要为了本课删除本体URDF或用另一份示例TF覆盖现有标定。公共`lidar_config`默认指向lite3_course_common包的config/MID360_config.json；更换设备/主机地址时按3-1复制独立JSON并以`lidar_config:=绝对路径`指定。

1. **供电、设备IP与主机网络。** 本节按用户提供的《Livox Mid-360用户手册》“设计外部电源/连线”（印刷页10—11）说明，官方资料入口为[Livox MID360下载页](https://www.livoxtech.com/mid-360/downloads)。雷达使用9~27V直流，推荐12V，注意线序与极性；稳态约6.5W，但低于35℃时启动峰值可达18W、持续约8秒，低温还可能自加热。电源与线缆需留出启动及压降余量，不能只按稳态功率供电，也不能将插上网线当成已经供电。

**先区分出厂IP与当前配置。** 出厂地址为`192.168.1.1XX`，XX是雷达SN最后两位数字，例如03对应`.103`。本课程这台雷达已配置成`192.168.1.3`，并非所有MID360出厂都为`.3`。换设备时先按SN在Livox Viewer 2确认真实地址，再改主机与驱动。

当前板端Wi-Fi为`wlan0/192.168.1.9`，SSH走该接口；雷达有线为`eth1/192.168.1.50/24`。本轮确认**SSH客户端Windows工作站与有线雷达恰好都为192.168.1.3**，但位于不同链路。不能给main表加一条全局`.3/32 dev eth1`，否则会把SSH回复也送向雷达。现板采用源策略：`.9 -> .3`走table361/wlan0，`.50 -> .3`走table360/eth1。先只读检查：

```bash
printf '%s\n' "${SSH_CONNECTION:-not-an-ssh-session}"
ip -br -4 addr
ip -4 rule show
ip -4 route show table main
ip -4 route show table 360
ip -4 route show table 361
ip route get 192.168.1.3 from 192.168.1.9
ip route get 192.168.1.3 from 192.168.1.50
ping -c 3 -W 2 -I 192.168.1.50 192.168.1.3
ros2 pkg prefix --share livox_ros_driver2
```

两条指定源查询应分别显示`dev wlan0 table 361`与`dev eth1 table 360`；main中不应有旧`.3/32 dev eth1`。不指定源的ping可能只是在访问Windows，不能证明雷达可达。现板已配置，先查看`nmcli connection show mid360-eth1`，不要重复创建或直接激活含旧main路由的profile。新板及路由变化须逐项执行[驱动准备指南的同IP源策略步骤](../../../DRIVER_SETUP.md#同ip的ssh客户端与雷达先保护wi-fi再隔离eth1)：先保护SSH并开第二个会话确认，再把有线路由放入table360，配置源`.50`到雷达的规则，最后仅对eth1尝试reapply并检查内核实际路由。不得通过关闭wlan0、清空路由或重启NetworkManager试错。

never-default避免有线抢默认路由，高metric不能抵消一条更具体的全局/32。雷达IP变化时，table360中的/32、对应源策略及JSON须同步修改；SSH保护表不跟随雷达修改。无法ping时结合驱动日志及`sudo timeout 10s tcpdump -ni eth1 -c 20 udp`确认实际UDP来源，不能只凭ping判定设备无数据。

**真正修改雷达IP要在设备侧完成。** 先停止本次ROS采集，在Livox Viewer 2中按SN选择设备，进入设备参数设置，修改其静态IP/掩码/网关并确认新地址生效。只改JSON的`lidar_configs[0].ip`是告诉驱动连接哪个现有地址，不会把雷达从`.103`改成`.3`。Viewer里的目标推送地址与设备自身IP也是两项不同配置。退出Viewer后再开一份ROS驱动，避免同时控制设备。

**修改实际生效的JSON。** 本包`config/MID360_example.json`只是示例；公共真机launch默认把`$(ros2 pkg prefix --share lite3_course_common)/config/MID360_config.json`传给官方驱动的user_config_path，当前为设备.3、主机eth1的.50。旧`03_camera_mid360`目录不再是本课默认配置来源。不要直接覆盖已安装文件；完成上面构建后，复制出本次独立配置，按实际网络编辑并检查JSON语法：

```bash
cfg_dir="$HOME/catkin_ws_lite3dog/real_configs"
mkdir -p "$cfg_dir"
LIDAR_CFG="$(mktemp --suffix=.json "$cfg_dir/MID360_site.XXXXXX")"
cp "$(ros2 pkg prefix --share lite3_course_common)/config/MID360_config.json" "$LIDAR_CFG"
${EDITOR:-nano} "$LIDAR_CFG"
python3 -m json.tool "$LIDAR_CFG"
export LIDAR_CFG
```

采用此副本时，在下面real launch末尾追加`lidar_config:="$LIDAR_CFG"`；当前默认网络不变可直接使用公共默认文件。每次证据保留实际采用的JSON和路径，不能只编辑未被launch选中的示例。

| JSON字段 | 本课程示例与含义 |
|---|---|
| lidar_configs[0].ip | 192.168.1.3，实际设备地址，不是改IP命令 |
| MID360.host_net_info的cmd_data_ip、push_msg_ip、point_data_ip、imu_data_ip | 均为192.168.1.50，即本机雷达网卡地址；不是Wi-Fi的.9 |
| MID360.host_net_info.log_data_ip | 当前为空；需要日志时再按真实接收地址配置 |
| lidar_configs[0].pcl_data_type / pattern_mode | 保留1 / 0；分别为设备32位笛卡尔格式与非重复扫描，不等于ROS xfer_format |
| lidar_configs[0].extrinsic_parameter | roll/pitch/yaw/x/y/z保持零；安装外参只在课程TF/模型中设置一次 |

| 数据 | lidar_net_info设备端口 | host_net_info主机接收端口 |
|---|---:|---:|
| 控制 | 56100 | 56101 |
| 状态推送 | 56200 | 56201 |
| 点云 | 56300 | 56301 |
| IMU | 56400 | 56401 |
| 日志 | 56500 | 56501 |

不要把两侧端口全部改成同一个值。完整锁定版本下载、SDK→ROS驱动安装顺序和JSON说明见[驱动准备指南](../../../DRIVER_SETUP.md)；现板驱动已安装，不重复覆盖。

**网络变化按依赖顺序操作并保留回退。**

- 仅主机地址变化：先保留旧eth1地址并增加同网段新地址，确认雷达/32仍走eth1，再改全部启用的host_net_info IP；设备IP没改就不改lidar_configs.ip。验证点云和IMU后再持久化主机地址、移除旧地址。若只把主机迁到另一网段，雷达却留原网段，应保留旧次地址，不能以改JSON代替网络可达性。
- 雷达也换网段：先备份旧设备IP/掩码/网关、NetworkManager连接与JSON；给eth1增加新网段次地址，保留旧连接；从旧网段用Viewer真正修改雷达IP；增加新雷达/32路由；改JSON的设备IP和主机接收IP；确认UDP/点云/IMU后再持久化新连接并移除旧地址。可用独立192.168.10.0/24示例，避开机器狗控制的192.168.2.0/24。全过程保持Wi-Fi SSH。
- 回退：只改主机/JSON就恢复原地址、/32和JSON备份；若设备IP已改变，先从新网段用Viewer把设备改回记录的旧IP，再恢复主机连接与JSON。只恢复JSON而设备仍在新网段不会恢复连接。具体次地址、连接克隆、持久化及回退命令在上述驱动指南中给出，不盲目恢复出厂。

2. **构建与启动。**

```bash
ros2 launch lite3_lab_3_1 experiment.launch.py mode:=real native_execute:=false rviz:=true \
  lidar_xyz:="$LIDAR_XYZ" lidar_rpy:="$LIDAR_RPY" initial_body_height:="$BODY_HEIGHT"
```

以上默认只采集/显示，不启动真实运动。若已有唯一真实驱动与适配器，使用start_runtime:=false只启动本实验，避免两份Livox同时连接同一雷达。

真机默认`real_livox_serialized:=true`：课程适配器以`raw=True`取得官方`CustomMsg`的CDR_LE字节，再用NumPy只读视图转换为`PointCloud2`，避免为每帧数万个点创建Python对象。它只改变解码方式，**不降采样、不删点、不改原header采样时间，不缩放逐点offset_time**；仍输出同一22字节XYZ/intensity/offset_time/line/tag布局。frame沿用课程统一的lidar_link，安装关系仍由TF表达。`timebase`只作诊断，不能拿接收时间替换原header。

解析不支持的CDR、截断、声明点数不一致或非法stamp时，该帧不发布；适配器逐帧累计`received/published/rejected`，首次及随后最多每5墙秒记录拒绝日志，不能把无输出当正常空云。若需与旧typed路径对照，先Ctrl+C停止本次launch，再用新结果目录重启：

```bash
ros2 launch lite3_lab_3_1 experiment.launch.py mode:=real rviz:=false \
  real_livox_serialized:=false native_execute:=false \
  lidar_xyz:="$LIDAR_XYZ" lidar_rpy:="$LIDAR_RPY" initial_body_height:="$BODY_HEIGHT" \
  result_dir:=/tmp/mid360_typed_comparison
```

该参数在节点构造时选择订阅方式；运行中改参数不会重建订阅，切换必须重启launch，不并行启动第二份硬件驱动。CDR异常不能静默改走typed后继续宣称成功；保留日志并核对驱动锁定版本，使用false只是明确选择的诊断对照。CDR真实ROS序列化回归、读数与性能结果见验证记录，不能把本地字节测试当板端速率成绩。

**安装外参只设一次。** JSON中的extrinsic_parameter保持零；课程launch的lidar_xyz/rpy描述base_link到雷达**光学原点**的安装关系，单位分别为米和弧度。真实模式发布这组实测安装TF。可选仿真中的模型补偿只用于兼容参考，不是现场标定方法。不要再手改第二份静态TF或在驱动里重复旋转/平移。停止上一实例后继续沿用上面输入的实测变量，不把交付示例写成实测值。

交付默认数值是未标定示例；RPY不能直接填角度数。MID360内部IMU与光学原点存在杆臂，公共imu_lidar_xyz/rpy另行组合，不能把二者当同一点。

公共launch还提供`mode:=bag`、`autostart:=false`、`start_runtime:=false`、`params_file:=绝对路径`和`result_dir:=绝对路径`。已有传感器运行时时设start_runtime=false。real使用系统时间；真实录包回放才生成单一`/clock`。

3. **核对话题和外参。** 新终端加载课程环境后逐条执行；hz与tf2_echo持续输出，观察后仅Ctrl+C退出该检查，再执行下一条：

```bash
ros2 topic info /course/points_raw -v
ros2 topic hz /course/points_raw
ros2 topic echo /course/imu --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 topic echo /course/mid360/status --once
```

**MID360的identity四元数是占位，不是姿态测量。** 本锁定官方驱动的InitImuMsg只填原stamp、gyro和acc，wire IMU也只有这六轴数据；ROS Quaternion默认w=1，因此看到[0,0,0,1]与全零姿态协方差，不能据此认定雷达或机身水平。按照[Humble Imu消息约定](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/Imu.msg)，课程适配器在real模式复制消息后把orientation_covariance[0]设为-1，明确姿态不可用；/livox/imu及收到的原对象保持原样。加速度仍按既有配置只做一次g到m/s²换算，gyro、原stamp和安装外参不变。

非LIO感知课先把比力按真实IMU安装旋转转到机身坐标，再据重力方向估计roll/pitch，发布base_footprint→base_link的倾斜；不能从加速度恢复绝对yaw，运动加速度也会干扰此估计，应静置核对。LIO模式不拿占位四元数生成该TF，由原LIO里程计路径处理；仿真仍使用其真实orientation。此规则来自已知MID360驱动来源，不能泛化为“所有identity姿态都不可用”。

RViz固定坐标为base_link，启用Raw points和TF frames。地面应形成连续平面，前方物体应位于正x。整体旋转90度先核对安装轴向和旋转外参；上下偏移先核对米/毫米和安装高度。一条TF父子关系只能有一个发布者，适配旧TORSO/mid360_link时不能给同一frame接两个父节点。

4. **逐函数读源码。** `algorithms.py::cloud_statistics`将输入整理为N×3数组，筛除NaN/Inf，计算距离与XYZ范围，空数组返回None。`experiment.py::cloud`记录墙钟接收时间，用公共`cloud_xyz`读XYZ，以消息width×height恢复原始总点数，再调用`transform_xyz`实际检查带采样时间的TF，异常保存到tf_error。`imu`只记录接收时间，不重复把g转换成m/s²。`report`每秒发布String状态并保存JSON：有效点、消息新鲜、TF可用都满足才是healthy。`main`负责Ctrl+C和节点关闭。

`config/params.yaml`中的stale_timeout_s单位为墙钟秒，target_frame是TF检查目标。新增calibration_measured默认false、calibration_record_path默认空：先复制本包params.yaml到本次独立参数文件（例如`cp "$(ros2 pkg prefix --share lite3_lab_3_1)/config/params.yaml" "$HOME/catkin_ws_lite3dog/real_configs/lab3_1_calibration.yaml"`，先创建real_configs并确认目标不存在），完成实测记录并由3-2比较后，才填写true和记录文件绝对路径，再用`params_file:=$HOME/catkin_ws_lite3dog/real_configs/lab3_1_calibration.yaml`追加到上述real启动命令。report的healthy只描述点云连接与TF连通，healthy_scope明确限定此含义；calibration_status=UNMEASURED表示未实测，MISSING_RECORD表示声明测量但文件缺失，DECLARED_WITH_RECORD仅表示操作者声明且记录文件存在，仍须人工核查内容，不能自动当作标定或现场验收通过。`rviz/experiment.rviz`保存显示配置。默认结果在`~/catkin_ws_lite3dog/results/`按包和时间分目录；指定result_dir可固定本次证据目录。

5. **停止与排查。** 按Ctrl+C停止。话题不存在先看驱动与模式；发布者为零先看设备、供电和网络；有发布者收不到先看QoS和ROS域；RViz报错看Fixed Frame与TF。不要用假身份TF掩盖安装误差。重试时保留上次JSON和日志，使用新结果目录。


**真实短包复查。** 保持本次静态传感器运行时，由3-2以start_runtime=false复用并录制最多30墙秒；按其步骤检查CustomMsg、SI IMU与TF消息数。结束本次真实采集后先由3-2启动唯一真实bag播放器，再开本课只读诊断：

```bash
ros2 launch lite3_lab_3_1 experiment.launch.py mode:=bag bag_source:=real \
  start_runtime:=false rviz:=true
```

回放只证明原数据可再次解码/变换，不能把已有录包的距离或TF一致性当成新硬件标定。

**可选兼容参考。** 只有主动研究旧Fortress参考时才显式使用`mode:=sim motion_backend:=gait`；它不替代本章真机验收，不与real采集同时启动。真实录包回放用`mode:=bag bag_source:=real`及唯一播放器。先结束自己启动的当前实例，再选参考模式；不要结束其它课程进程。

# 实验结果

本轮实际执行状态及资源限制见[验证记录](../../../VERIFICATION.md)。下面的预期现象和判定阈值不代表已经取得该成绩。

预期得到可见点云、连续IMU和明确安装外参，`mid360_status.json`记录点数、范围、frame、TF检查和消息年龄。独立重复3次，保存每次JSON、驱动日志、安装测量记录和RViz截图，比较点数、header年龄及距离波动。未实测时必须保留calibration_status=UNMEASURED，不能因为TF存在和healthy=true就声称标定通过。

本地数值测试覆盖NaN、空输入以及连接健康与未测/缺失标定记录状态的区分；它不能证明设备网络或真机已通过。运行：`python3 -m pytest src/03_sensors/3-1_mid360/tests -q`。现场以当次话题、JSON和画面验收。

配置依据[Livox官方ROS 2驱动](https://github.com/Livox-SDK/livox_ros_driver2)。
