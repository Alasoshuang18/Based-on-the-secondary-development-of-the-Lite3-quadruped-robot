# 3-3 三维点云处理与障碍物检测

环境：Ubuntu 22.04 + ROS 2 Humble。以下命令在机器狗的 Ubuntu 终端执行。

## 实验原理

点云先按采样时间转换到机身坐标系 `base_link`，去掉无效点、范围外点和机身自身的点，再降采样、按重力方向判断地面，最后提取离地障碍并画包围框。

机身或雷达倾斜时，不能直接按原始点云的 z 值判断障碍。程序优先使用姿态 TF，缺少时才在低动态条件下使用新鲜 IMU 加速度估计向上方向；缺少可靠姿态或坐标变换时丢弃该帧。分类时做重力对齐，发布的点仍使用真实 `base_link` 坐标。

默认体素边长 8 cm，地面残差范围约 ±9 cm，离地 12 cm～1.8 m 的点作为障碍。该方法适合近似平坦地面，台阶、陡坡和站立高度变化需另行验证。

## 实验目标

- 将地面、机身和外部障碍分开。
- 在 RViz 中看到地面、障碍点云及障碍物包围框。
- 对同一箱子独立测试三次，保存参数、结果、截图及对应原始数据。

## 需要操作的地方

按顺序完成：**编译 → 加载安装参数 → 修改感知参数 → 启动 → 摆箱测试 → 保存结果**。

### 1. 编译（首次使用或源码更新后执行）

已有课程源码，不需要重新创建实验包。先退出上一章自己启动的录制或回放，避免占用同一运行环境。

```bash
source ~/catkin_ws_lite3dog/course_env.sh
cd ~/catkin_ws_lite3dog
colcon build --symlink-install --base-paths src/lite3_course_native src/lite3_course_common src/03_sensors src/04_slam src/05_navigation --packages-up-to lite3_lab_3_3
source ~/catkin_ws_lite3dog/course_env.sh
```

编译成功再继续。不要结束其他章节已有的仿真或控制程序，也不要为本章修改雷达网络配置。

### 2. 加载上一章确认的安装参数

```bash
source ~/catkin_ws_lite3dog/mid360_env.sh

echo "雷达位置：$LIDAR_XYZ"
echo "雷达角度：$LIDAR_RPY"
echo "机身高度：$BODY_HEIGHT"
```

应分别为：

| 参数 | 当前采用值 |
| --- | --- |
| 雷达位置，米 | `[0.210,0,0.161308]` |
| 雷达 RPY，弧度 | `[0,0.338594,0]` |
| 机身原点离地高度，米 | `0.325504965` |

本章沿用用户确认的这组实测参数；安装或姿态高度变化后重新核对。雷达 JSON 外参保持零。

### 3. 创建并修改感知参数（首次设置时执行）

使用固定文件名，方便关闭终端后重新找到。已有文件则保留，不覆盖：

```bash
mkdir -p "$HOME/catkin_ws_lite3dog/real_configs"
export PERCEPTION_CFG="$HOME/catkin_ws_lite3dog/real_configs/lab3_3_perception.yaml"

if [ ! -e "$PERCEPTION_CFG" ]; then
  cp "$(ros2 pkg prefix --share lite3_lab_3_3)/config/params.yaml" "$PERCEPTION_CFG"
fi

vi "$PERCEPTION_CFG"
```

在原有 YAML 结构中修改下表三项，保留缩进和其他参数：

| 参数 | 怎么填写 |
| --- | --- |
| `ground_z_m` | 填 `-0.325504965`，即机身离地高度的负值 |
| `body_min_m` | `[-0.336517,-0.187001,-0.336179]`，模型默认站姿下含四腿的最小坐标，单位米 |
| `body_max_m` | `[0.278800,0.187001,0.072864]`，同一模型站姿下的最大坐标，单位米 |

这三项直接填写：

```yaml
ground_z_m: -0.325504965
body_min_m: [-0.336517, -0.187001, -0.336179]
body_max_m: [0.278800, 0.187001, 0.072864]
```

机身范围按 Lite3 官方 URDF 与 STL 重新计算：坐标系为 `base_link`（与官方 `TORSO` 重合），包含躯干、四条腿和足部的可视网格与碰撞几何，采用模型预览的默认站姿：每条腿 `HipX=0`、`HipY=-0.7`、`Knee=1.4` 弧度。数值向外保留 6 位小数，未另加过滤余量；未包含加装的雷达与支架。

这是指定站姿的模型计算值。实际腿姿或附件变化后核对自体点是否仍被覆盖；单个长方体也会过滤腿间空隙内的点，不能一味扩大范围。模型脚底约为 `z=-0.336179 m`，与当前地面参考 `-0.325504965 m` 不完全一致；地面高度仍沿用用户确认值，不由模型站姿替换。

来源：本地 `Lite3_mid360_v15.urdf` 中保留的官方关节与网格，已逐项核对原始 `Lite3/urdf/Lite3.urdf` 的关节参数及四份 STL 的一致性。官方模型版本为 `75824b516ecc8fa3f8f7ce5d6577e1e86d6b614f`。完整坐标范围和分部件结果见随附的 `Lite3_模型包围范围测量.json`。

vi 操作：按 `i` 编辑，完成后按 `Esc`，输入 `:wq` 并回车保存。首次测试其他参数先保留默认值。

### 4. 启动真机处理（每次测试执行）

机器狗保持静止，只启动感知，不发运动命令。每次自动选择下一个未使用的 `run1、run2、run3…`：

```bash
source ~/catkin_ws_lite3dog/course_env.sh
source ~/catkin_ws_lite3dog/mid360_env.sh
export PERCEPTION_CFG="$HOME/catkin_ws_lite3dog/real_configs/lab3_3_perception.yaml"

BASE="$HOME/catkin_ws_lite3dog/results/lab_3_3"
n=1
while [ -e "$BASE/run$n" ] || [ -L "$BASE/run$n" ]; do
  n=$((n + 1))
done
DETECTION_DIR="$BASE/run$n"
echo "本次结果目录：$DETECTION_DIR"

if [ -f "$PERCEPTION_CFG" ] && [ -n "$LIDAR_XYZ" ] && [ -n "$LIDAR_RPY" ] && [ -n "$BODY_HEIGHT" ]; then
  ros2 launch lite3_lab_3_3 experiment.launch.py \
    mode:=real native_execute:=false rviz:=true \
    lidar_xyz:="$LIDAR_XYZ" \
    lidar_rpy:="$LIDAR_RPY" \
    initial_body_height:="$BODY_HEIGHT" \
    params_file:="$PERCEPTION_CFG" \
    result_dir:="$DETECTION_DIR"
else
  echo "缺少参数或文件，请先完成第 2、3 步。"
fi
```

已有本体 URDF 或其他程序发布真实传感器安装 TF 时，给启动命令追加 `publish_sensor_tf:=false`，避免重复发布。

### 5. 摆箱子，检查 RViz

RViz 的 Fixed Frame 使用 `base_link`。先看空地，再在前方放箱子、移动箱子位置，检查：

| 观察内容 | 预期现象 |
| --- | --- |
| 地面 | 绿色，不应大片变成红色障碍 |
| 箱子等离地物体 | 红色障碍点 |
| 足够点数的障碍物 | 出现半透明包围框，位置随物体变化 |
| 机身 | 不应大量出现在障碍点云中 |
| 状态 | 无持续 TF 或 IMU 错误 |

小簇点数少于 `min_cluster_points` 时，仍会保留在障碍点云中，但不画框，因此“没有框”不一定是没有检测到障碍。

基线测试通过后再检查不同机身倾角；倾斜测试先确认姿态 TF 有效，再判断分类效果，不要为了获得倾斜姿态而启用运动控制。

![检测图](C:\Users\Alaso\Desktop\mid360\3-3_三维点云处理与障碍物检测\检测图.png)

![避障](C:\Users\Alaso\Desktop\mid360\3-3_三维点云处理与障碍物检测\避障.png)

### 6. 检查输出（另开终端，逐条执行）

```bash
source ~/catkin_ws_lite3dog/course_env.sh
ros2 topic hz /course/points_filtered
ros2 topic hz /course/obstacles
ros2 topic echo /course/obstacle_markers --once
ros2 run tf2_ros tf2_echo base_footprint base_link
```

`hz` 和 `tf2_echo` 会持续运行，每看完一项按 `Ctrl+C` 结束该检查，再执行下一条；不会因此停止另一个终端的主程序。查看包围框消息时应先摆好箱子。

输出话题：

| 话题 | 内容 |
| --- | --- |
| `/course/points_filtered` | 过滤后的点云 |
| `/course/ground` | 地面点云 |
| `/course/obstacles` | 障碍点云 |
| `/course/obstacle_markers` | 包围框 |

三个输出点云的 `frame_id` 应为 `base_link`。程序最高处理频率为 10 Hz，实际频率取决于输入和板端耗时，不能只凭话题存在判断通过。

![圈出障碍物](C:\Users\Alaso\Desktop\mid360\3-3_三维点云处理与障碍物检测\圈出障碍物.png)

### 7. 保留对应原始数据

本章启动命令只处理点云，**不会自动录制原始 bag**。需要保存原始证据时，在本章真机程序运行期间另开终端，加载上一章设置，并输入第 4 步显示的本次结果目录：

```bash
source ~/catkin_ws_lite3dog/course_env.sh
source ~/catkin_ws_lite3dog/mid360_env.sh
read -r -p '粘贴本次结果目录的绝对路径：' DETECTION_DIR

if [ -d "$DETECTION_DIR" ] && [ -f "$FRAMES_CFG" ]; then
  ros2 launch lite3_lab_3_2 experiment.launch.py \
    mode:=real start_runtime:=false native_execute:=false \
    autostart:=true rviz:=false \
    lidar_xyz:="$LIDAR_XYZ" lidar_rpy:="$LIDAR_RPY" \
    initial_body_height:="$BODY_HEIGHT" \
    params_file:="$FRAMES_CFG" \
    result_dir:="$DETECTION_DIR/source_record"
else
  echo "结果目录或 3-2 参数文件不存在，请核对路径。"
fi
```

这里复用正在运行的本章真机环境，不能同时存在上一章的离线播放器。录包开始后约 30 秒自动保存，再在录包终端按 `Ctrl+C`，检查：

```bash
cat "$DETECTION_DIR/source_record/record_result.json"
```

应为 `valid_recording: true`。每次只录一次，不重复使用同一个 `source_record` 目录。录制期间完成本次箱子场景的观察和截图。

### 8. 保存结果，重复三次

在 RViz 中保存测试截图。每次测试完成，回主程序终端按 `Ctrl+C`，等待退出，然后在该终端执行：

```bash
cp "$PERCEPTION_CFG" "$DETECTION_DIR/params_used.yaml"
cat "$DETECTION_DIR/obstacles.json"
```

重点记录点数、处理耗时、重力来源 `up_source`、估计地面高度 `estimated_ground_z_m` 和包围框。JSON 每秒覆盖，只代表最近一次状态，不是完整过程录像；需结合截图和原始 bag 判断。

对同一箱子独立重复三次，每次重新执行第 4～8 步，保存到不同 run 目录。后续 3-4、3-5 使用同一份已核对的感知配置。

## 用上一章的 bag 测试（可选）

先退出本章真机程序。在一个终端按 3-2 的方法启动唯一的真实数据包播放器，再另开终端运行：

```bash
source ~/catkin_ws_lite3dog/course_env.sh
export PERCEPTION_CFG="$HOME/catkin_ws_lite3dog/real_configs/lab3_3_perception.yaml"

ros2 launch lite3_lab_3_3 experiment.launch.py \
  mode:=bag bag_source:=real start_runtime:=false rviz:=true \
  params_file:="$PERCEPTION_CFG" \
  result_dir:="$HOME/catkin_ws_lite3dog/results/lab_3_3/replay_$(date +%Y%m%d_%H%M%S)"
```

播放期间按第 5、6 步观察结果。若启动处理程序时旧包已经播完，需退出本次处理和播放器，再重新回放检查。不要同时运行真实驱动或第二个时钟源。

旧包只包含录制当时的场景，看不到现在新摆的箱子。回放测试结束后，分别按 `Ctrl+C` 退出处理程序和播放器。

## 遇到问题先看这里

| 问题 | 先怎么处理 |
| --- | --- |
| `lidar_xyz必须为3个有限数` | 加载 `mid360_env.sh`，确认位置、角度、高度不为空 |
| 参数文件找不到 | 重新设置 `PERCEPTION_CFG` 为第 3 步的固定路径，确认文件存在 |
| 公共运行时被占用 | 先退出自己上一轮录制或回放；残留锁按 3-2 文档检查，不直接删锁 |
| 全地面变红 | 核对 `ground_z_m=-0.325504965`、单位和 `up_source` |
| 机身变红 | 核对安装外参及 `body_min_m/body_max_m` |
| 附近箱子被滤掉 | 检查机身过滤范围是否过大，以及距离、高度过滤条件 |
| 细杆或小物体消失 | 尝试减小体素边长，每次只修改一个参数并换新结果目录 |
| 有红色点但没有框 | 检查是否达到 `min_cluster_points` |
| 没有输出 | 看 TF、IMU 警告及输入点云，地面点不足或变换失败时可能丢帧 |
| IMU 的姿态为单位四元数 | 若 `orientation_covariance[0]=-1`，代表不提供姿态，不能当作水平；检查姿态 TF 或低动态比力回退 |
| 开头短暂等待 TF，随后正常输出 | 订阅建立阶段的等待；持续报错才需排查 |
| `Stereo is NOT SUPPORTED` | 不影响普通点云显示 |

默认算法要求至少 12 个地面点，地面拟合最大坡度 20°。不要用假的单位旋转强行跳过姿态检查。加速度过大、IMU 过期或 TF 缺失时丢帧，应排查原因；无输出不能当作“没有障碍”。

## 实验结果与完成检查

本文是操作说明，尚未记录本章现场测试通过的结果。完成后逐项确认：

- [ ] 地面高度按本机确认值配置，机身过滤范围已采用上述模型站姿值，并现场核对腿部和附件。
- [ ] 输出点云、地面、障碍与包围框正常，坐标系正确。
- [ ] 同一箱子独立测试三次，地面无明显误检，机身过滤合理。
- [ ] 每次保存配置、JSON、截图和对应有效原始 bag。
- [ ] 比较三次障碍位置、误检和板端处理耗时。

通过这些检查说明本章静态感知流程可用，不代表台阶、陡坡、动态行走或自动避障已验证。

