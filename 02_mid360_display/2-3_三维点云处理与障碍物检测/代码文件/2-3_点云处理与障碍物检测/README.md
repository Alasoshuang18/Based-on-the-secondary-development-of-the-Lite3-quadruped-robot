# 实验原理

机身俯仰时地面也会在雷达坐标中抬高，因此不能直接对原始z做阈值避障。流程为“按采样时间查TF到base_link → 有限范围与机身mask → 体素降采样 → 重力对齐 → 地面估计 → 离地障碍 → XY栅格连通”。重力对齐用于判断高度，发布点仍保持实际base坐标，不能把level坐标冒充base。

重力首选采样时刻的base_footprint<-base_link姿态TF。缺少姿态TF时，可使用与点云时间接近、模长接近9.81m/s²的新鲜IMU比力，并先按IMU安装TF旋转到base。加速度过大、IMU过期或缺TF时丢弃本帧，允许下游超时停止。

体素每格保留一个实际点，默认边长8cm。地面在level坐标中用预期地面高度附近的点做两轮鲁棒最小二乘，同时约束最大坡度20度；墙面不能被拟合成地面。平面残差±9cm内为ground，离地12cm~1.8m为障碍。占用格8邻域连通避免构建N×N距离矩阵，适合ARM。该教学算法适用于近地平面，台阶、陡坡与站立高度变化需另行验证。

# 实验目的

- 理解TF、重力方向与地面分类之间的关系。
- 分开机身、地面与离地障碍，调整体素和栅格参数。
- 发布base坐标障碍点云及可解释的包围框。

# 实验步骤

**从零创建实验包（已有交付目录跳过）。** 以下命令只供本实验目标尚不存在的空位置从零重建教学使用；现有交付目录不要重复创建、嵌套或覆盖。任一保护检查失败就使用现有交付源码，继续本节后面的环境与构建步骤；子shell的`exit 1`只停止本段，不关闭终端。

```bash
(
  set -e
  source ~/catkin_ws_lite3dog/course_env.sh
  mkdir -p ~/catkin_ws_lite3dog/src/03_sensors
  cd ~/catkin_ws_lite3dog/src/03_sensors
  test ! -e lite3_lab_3_3 && test ! -L lite3_lab_3_3 || { printf '%s\n' 'lite3_lab_3_3已存在，停止创建'; exit 1; }
  test ! -e 3-3_pointcloud_obstacles && test ! -L 3-3_pointcloud_obstacles || { printf '%s\n' '3-3_pointcloud_obstacles已存在，停止创建'; exit 1; }
  ros2 pkg create lite3_lab_3_3 --build-type ament_python --license Apache-2.0 \
    --dependencies rclpy sensor_msgs std_msgs geometry_msgs visualization_msgs \
    nav_msgs tf2_ros lite3_course_common launch launch_ros \
    ament_index_python
  mv -- lite3_lab_3_3 3-3_pointcloud_obstacles
  cd 3-3_pointcloud_obstacles
  mkdir -p launch config tests rviz
)
```

`3-3_pointcloud_obstacles`是外层课程实验目录名，方便按章节阅读；ROS包名、`package.xml`的`<name>`、`setup.py`中的`package_name`、`resource/lite3_lab_3_3`和内层Python模块目录必须仍为合法名字`lite3_lab_3_3`。重命名只改变外层目录，不把数字开头、含连字符的实验目录名写成ROS包名或Python模块名。上面的依赖逐项来自本包实际`package.xml`的ROS依赖；`python3-*`对应的NumPy、YAML等系统/Python依赖按环境安装，不放入这个创建命令。

脚手架只有包清单、安装配置、资源索引和Python模块骨架，尚未实现本实验算法。运行前必须恢复交付的Python源码、`launch/config/tests`、RViz文件及配套`package.xml/setup.py/setup.cfg`。在`setup.py`的`console_scripts`中注册`experiment = lite3_lab_3_3.experiment:main`，并用`data_files`把`resource/lite3_lab_3_3`安装到`share/ament_index/resource_index/packages`，把`package.xml`安装到`share/lite3_lab_3_3`，把`launch/config/rviz`中的资源安装到`share/lite3_lab_3_3/`下对应子目录；保留交付中其它实际入口和数据注册。恢复`launch/experiment.launch.py`的`generate_launch_description()`入口及公共启动调用后，才回到`~/catkin_ws_lite3dog`按后文执行`colcon build --packages-up-to lite3_lab_3_3`，加载`course_env.sh`与编译后的overlay，再用`ros2 launch lite3_lab_3_3 experiment.launch.py`启动。创建脚手架成功不代表节点算法、配置或硬件实验已经实现/验证。

环境为Ubuntu 22.04、ROS 2 Humble、NumPy；真机驱动位于~/devices_driver。每个终端先执行课程环境，使用同一课程ROS域（环境默认43，现场独立测试若改为44则所有终端一起改）：

```bash
source ~/catkin_ws_lite3dog/course_env.sh
cd ~/catkin_ws_lite3dog
colcon build --symlink-install --base-paths src/lite3_course_native src/lite3_course_common src/03_sensors src/04_slam src/05_navigation --packages-up-to lite3_lab_3_3
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

1. 先完成3-1连接与3-2外参检查。配置ground_z_m：ground_z_m应填本次实测BODY_HEIGHT的负值；交付-0.40仅是未标定示例。这是机身原点高度，不是雷达安装高度。body_min_m/body_max_m表示机身在base中的实测包络，交付bbox仅示例，须包含本体并核对腿部姿态，不能大到遮掉腿外障碍。把这些实测值填入独立params_file，保留测量记录。体素越大越省算力，也越易丢失细杆。

2. 创建独立真实感知参数，按上一步实测值编辑ground_z_m和body_min_m/body_max_m；保留本次文件作为证据，3-4/3-5也引用它：

```bash
cfg_dir="$HOME/catkin_ws_lite3dog/real_configs"
mkdir -p "$cfg_dir"
PERCEPTION_CFG="$(mktemp --suffix=.yaml "$cfg_dir/lab3_3_perception.XXXXXX")"
cp "$(ros2 pkg prefix --share lite3_lab_3_3)/config/params.yaml" "$PERCEPTION_CFG"
${EDITOR:-nano} "$PERCEPTION_CFG"
export PERCEPTION_CFG
```

构建与运行：

```bash
ros2 launch lite3_lab_3_3 experiment.launch.py mode:=real native_execute:=false rviz:=true \
  lidar_xyz:="$LIDAR_XYZ" lidar_rpy:="$LIDAR_RPY" initial_body_height:="$BODY_HEIGHT" \
  params_file:="$PERCEPTION_CFG"
```

以上只做静态感知，不发运动。若已有3-2唯一真实bag播放器，可另开终端只启动处理：

```bash
ros2 launch lite3_lab_3_3 experiment.launch.py mode:=bag bag_source:=real start_runtime:=false rviz:=true \
  params_file:="$PERCEPTION_CFG"
```

在另一同域终端逐条观察，hz/tf2_echo持续输出时仅Ctrl+C退出该检查：

```bash
ros2 topic hz /course/points_filtered
ros2 topic hz /course/obstacles
ros2 topic echo /course/obstacle_markers --once
ros2 run tf2_ros tf2_echo base_footprint base_link
```

MID360的课程IMU姿态cov[0]=-1，不能把identity当水平姿态；优先使用按实测安装旋转得到的重力倾角TF，低动态时才允许比力回退。RViz固定base_link，绿色为Ground、红色为Obstacles，半透明立方体为包围框。在前方摆箱子并移动，地面不应形成红色障碍；倾斜机身后先检查姿态TF，再看分类。小于min_cluster_points的点仍保留在障碍点云中，只是不画框，便于下游保守停车。

3. 逐函数读algorithms.py。gravity_rotation使用Rodrigues公式把base中的向上向量转到+Z，处理平行、反向和零向量。process_cloud先删非有限点；超出120000点按确定步长抽样；用距离平方筛选；排除机身mask；通过整数体素键和np.unique保留实测点；在level中拟合受坡度与高度约束的地面，计算离地高度并筛出两类点。connected_boxes把level中的占用XY格放进字典，以队列遍历8邻域，但用逐点对应的原base坐标计算包围框；无需PCL/Open3D。

4. 逐函数读experiment.py。quaternion_matrix把TF四元数转成旋转矩阵。imu缓存新鲜消息。up_direction先查采样时刻姿态TF，再检查IMU时间差、模长与安装旋转。cloud只缓存一帧待处理与一帧最新消息；process_pending每0.05秒运行，让TF listener先处理消息，按原采样时间can_transform，最多等待tf_wait_s后明确丢帧，不用最新TF替代。process_cloud_message最高10Hz实际处理，公共transform_xyz真实查TF，输出保留采样时间。地面不足minimum_ground_points（默认12点）、无有效点或变换失败都不发布空障碍。publish_boxes先DELETEALL清理旧框，再画最多存活1秒的CUBE。obstacles.json每秒覆盖点数、耗时、重力来源、estimated_ground_z_m和框；地面标称默认-0.40m，允许在±2×ground_tolerance_m范围内拟合，但必须按真实站高核对。

5. 参数全在config/params.yaml，显示在rviz/experiment.rviz。每次只改一项，复制参数文件并用params_file:=绝对路径传入；用result_dir:=绝对路径保留独立证据。全地面变红先查ground_z、单位和up_source；机身变红先查mask与外参；细杆消失先减小voxel；无输出查TF/IMU警告，不能用假身份旋转替代。IMU比力回退只适用于低动态，不等同完整姿态融合。

按Ctrl+C停止。默认结果保存在~/catkin_ws_lite3dog/results/包名/时间戳。最新JSON不是完整时序，需要时用3-2保留原包后回放。


**可选兼容参考。** 只有主动研究旧Fortress参考时才显式使用`mode:=sim motion_backend:=gait`；它不替代本章真机验收，不与real采集同时启动。真实录包回放用`mode:=bag bag_source:=real`及唯一播放器。先结束自己启动的当前实例，再选参考模式；不要结束其它课程进程。

# 实验结果

本轮实际执行状态及资源限制见[验证记录](../../../VERIFICATION.md)。下面的预期现象和判定阈值不代表已经取得该成绩。

输出/course/points_filtered、/course/ground、/course/obstacles三个frame_id=base_link的PointCloud2，以及/course/obstacle_markers。对同一箱子独立重复3次，保存原bag、参数、JSON及截图，比较位置、地面误检和处理耗时。

本地测试覆盖平地、倾斜地面、机身mask、NaN、重力退化与连通分组：`python3 -m pytest src/03_sensors/3-3_pointcloud_obstacles/tests -q`。数值通过不能替代ARM实时耗时、ROS话题和现场分割验收。
