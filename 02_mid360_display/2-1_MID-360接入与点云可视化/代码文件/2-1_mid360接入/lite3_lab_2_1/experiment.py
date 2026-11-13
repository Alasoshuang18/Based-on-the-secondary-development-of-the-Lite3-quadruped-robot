"""3-1：订阅原始MID360点云，保留原消息用于可视化，输出连接诊断。"""
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from lite3_course_common.ros_utils import cloud_xyz, transform_xyz, result_directory, write_json
from .algorithms import cloud_statistics


class Mid360Experiment(Node):
    def __init__(self):
        super().__init__('mid360_experiment')
        for name, default in [('mode', 'real'), ('autostart', True), ('result_dir', ''),
                              ('input_topic', '/course/points_raw'), ('stale_timeout_s', 1.0),
                              ('target_frame', 'base_link'), ('calibration_measured', False),
                              ('calibration_record_path', '')]:
            self.declare_parameter(name, default)
        self.output = result_directory(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.stats, self.last_receive, self.last_imu = {}, None, None
        self.frames, self.received = set(), 0
        self.status_pub = self.create_publisher(String, '/course/mid360/status', 10)
        self.preview_pub = self.create_publisher(PointCloud2, '/course/mid360/preview', 5)
        self.create_subscription(PointCloud2, self.get_parameter('input_topic').value,
                                 self.cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/course/imu', self.imu, qos_profile_sensor_data)
        self.create_timer(1., self.report)
        self.get_logger().info('等待MID360点云。原始消息不改字段，RViz固定坐标使用base_link。')

    def imu(self, msg):
        self.last_imu = time.monotonic()

    def cloud(self, msg):
        self.last_receive = time.monotonic()
        self.received += 1
        self.frames.add(msg.header.frame_id)
        try:
            points = cloud_xyz(msg)
            self.stats = cloud_statistics(points)
            self.stats['points'] = int(msg.width * msg.height)
            self.stats['invalid_points'] = max(0, self.stats['points'] - self.stats['finite_points'])
            self.stats['frame_id'] = msg.header.frame_id
            # 真实查TF，失败时不会把传感器坐标冒充base坐标。
            transform_xyz(self, self.buffer, points[:1], msg.header,
                          self.get_parameter('target_frame').value)
            self.stats['tf_ready'] = True
            self.stats.pop('tf_error', None)
        except Exception as exc:
            self.stats['tf_ready'] = False
            self.stats['tf_error'] = str(exc)
        self.preview_pub.publish(msg)

    def report(self):
        now = time.monotonic()
        age = None if self.last_receive is None else now - self.last_receive
        summary = dict(self.stats, received_clouds=self.received,
                       seen_frames=sorted(self.frames), age_s=age,
                       imu_age_s=None if self.last_imu is None else now - self.last_imu)
        summary['healthy'] = (age is not None and age < self.get_parameter('stale_timeout_s').value
                              and self.stats.get('finite_points', 0) > 0
                              and self.stats.get('tf_ready', False))
        # TF连通只能说明配置可用，不能证明示例尺寸已经过现场测量。
        measured = bool(self.get_parameter('calibration_measured').value)
        record = str(self.get_parameter('calibration_record_path').value)
        record_exists = bool(record and Path(record).expanduser().is_file())
        summary.update(healthy_scope='pointcloud_connection_and_tf_only',
                       calibration_measured_declared=measured, calibration_record_path=record,
                       calibration_record_exists=record_exists,
                       calibration_status='UNMEASURED' if not measured else
                                          'DECLARED_WITH_RECORD' if record_exists else 'MISSING_RECORD')
        self.status_pub.publish(String(data=json.dumps(summary, ensure_ascii=False)))
        write_json(self.output / 'mid360_status.json', summary)
        if not summary['healthy']:
            self.get_logger().warning('点云未就绪/过期/外参缺失：检查IP、驱动、QoS和TF；详见mid360_status.json')


def main(args=None):
    rclpy.init(args=args)
    node = Mid360Experiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
