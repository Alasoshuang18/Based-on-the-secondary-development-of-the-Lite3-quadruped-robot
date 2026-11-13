"""3-3：按测量时间查TF，缺姿态时停止输出，供下游看门狗安全停车。"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from lite3_course_common.ros_utils import cloud_xyz, cloud_message, transform_xyz, result_directory, write_json
from .algorithms import process_cloud, gravity_rotation


def quaternion_matrix(q):
    a = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-8:
        raise ValueError('TF四元数为零')
    x, y, z, w = a / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class ObstacleExperiment(Node):
    def __init__(self):
        super().__init__('pointcloud_obstacles_experiment')
        defaults = [('mode', 'real'), ('autostart', True), ('result_dir', ''),
                    ('input_topic', '/course/points_raw'), ('voxel_m', .08),
                    ('min_range_m', .35), ('max_range_m', 8.), ('max_points', 120000),
                    ('body_min_m', [-.45, -.25, -.45]), ('body_max_m', [.45, .25, .3]),
                    ('ground_z_m', -.40), ('ground_tolerance_m', .09),
                    ('ground_max_slope_deg', 20.), ('obstacle_min_height_m', .12),
                    ('obstacle_max_height_m', 1.8), ('grid_cell_m', .18),
                    ('min_cluster_points', 6), ('max_processing_hz', 10.), ('imu_timeout_s', .4),
                    ('tf_wait_s', .2), ('minimum_ground_points', 12)]
        for name, default in defaults:
            self.declare_parameter(name, default)
        self.output = result_directory(self)
        self.buffer, self.imu_data = Buffer(), None
        self.listener = TransformListener(self.buffer, self)
        self.cloud_publishers = [self.create_publisher(PointCloud2, topic, 5) for topic in
                           ['/course/points_filtered', '/course/ground', '/course/obstacles']]
        self.markers = self.create_publisher(MarkerArray, '/course/obstacle_markers', 5)
        self.last_processed, self.last_warning, self.last_report = 0., 0., 0.
        self.counter, self.dropped = 0, 0
        self.pending, self.latest = None, None
        self.create_subscription(PointCloud2, self.get_parameter('input_topic').value,
                                 self.cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/course/imu', self.imu, qos_profile_sensor_data)
        from rclpy.clock import Clock, ClockType
        self.create_timer(.05, self.process_pending, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def imu(self, msg):
        self.imu_data = (msg, time.monotonic())

    def up_direction(self, header):
        try:
            tf = self.buffer.lookup_transform('base_footprint', 'base_link', Time.from_msg(header.stamp))
            return quaternion_matrix(tf.transform.rotation).T @ np.array([0., 0., 1.]), 'posture_tf'
        except Exception:
            # 低动态时，加速度计测到向上的比力。只接收新鲜、模长近g的样本。
            if self.imu_data is None or time.monotonic() - self.imu_data[1] > self.get_parameter('imu_timeout_s').value:
                raise RuntimeError('缺少姿态TF，且IMU未就绪或过期')
            imu = self.imu_data[0]
            stamp_delta = abs((header.stamp.sec - imu.header.stamp.sec) +
                              (header.stamp.nanosec - imu.header.stamp.nanosec) * 1e-9)
            if stamp_delta > self.get_parameter('imu_timeout_s').value:
                raise RuntimeError('IMU与点云时间差过大')
            a = imu.linear_acceleration
            up = np.array([a.x, a.y, a.z])
            if not 7.8 <= np.linalg.norm(up) <= 11.8:
                raise RuntimeError('IMU动态加速度过大，不能可靠估计重力')
            tf = self.buffer.lookup_transform('base_link', imu.header.frame_id, Time.from_msg(imu.header.stamp))
            return quaternion_matrix(tf.transform.rotation) @ up, 'imu_specific_force'

    def cloud(self, msg):
        # callback只缓存，不阻塞TF listener；最多持有2帧，输入快也不会无限积压。
        item = (msg, time.monotonic())
        if self.pending is None:
            self.pending = item
        else:
            self.latest = item

    def process_pending(self):
        if self.pending is None:
            return
        now = time.monotonic()
        hz = max(.1, self.get_parameter('max_processing_hz').value)
        if now - self.last_processed < 1. / hz:
            return
        msg, received = self.pending
        age = now - received
        if age < .03:
            return
        wait_limit = max(.05, min(.5, self.get_parameter('tf_wait_s').value))
        stamp = Time.from_msg(msg.header.stamp)
        sensor_ready = msg.header.frame_id == 'base_link' or self.buffer.can_transform('base_link', msg.header.frame_id, stamp)
        posture_ready = self.buffer.can_transform('base_footprint', 'base_link', stamp)
        if not sensor_ready or (not posture_ready and age < wait_limit):
            if age < wait_limit:
                return
            if not sensor_ready:
                self.dropped += 1
                self.pending, self.latest = self.latest, None
                if now-self.last_warning > 2.:
                    self.last_warning = now
                    self.get_logger().warning('等待采样时刻的雷达外参超时，明确丢帧；不使用最新TF替代')
                return
        # 已让executor处理过TF回调，can_transform就绪后公共lookup不再阻塞等待。
        self.pending, self.latest = self.latest, None
        self.process_cloud_message(msg)

    def process_cloud_message(self, msg):
        now = time.monotonic()
        self.last_processed = now
        try:
            base = transform_xyz(self, self.buffer, cloud_xyz(msg), msg.header, 'base_link')
            up, up_source = self.up_direction(msg.header)
            p = lambda name: self.get_parameter(name).value
            filtered, ground, obstacles, boxes = process_cloud(
                base, up=up, voxel=p('voxel_m'), min_range=p('min_range_m'), max_range=p('max_range_m'),
                body_min=p('body_min_m'), body_max=p('body_max_m'), ground_z=p('ground_z_m'),
                ground_tolerance=p('ground_tolerance_m'), ground_max_slope_deg=p('ground_max_slope_deg'),
                obstacle_min_height=p('obstacle_min_height_m'), obstacle_max_height=p('obstacle_max_height_m'),
                cell_size=p('grid_cell_m'), min_cluster_points=p('min_cluster_points'), max_points=p('max_points'))
            header = Header(stamp=msg.header.stamp, frame_id='base_link')
            # 空原始帧不代表道路畅通：不发布“空障碍”，让避障超时停止。
            if len(filtered) == 0:
                raise RuntimeError('有效点云为空，不能据此宣告前方畅通')
            if len(ground) < max(1, p('minimum_ground_points')):
                raise RuntimeError('地面有效点不足，无法确认分割质量；检查ground_z、外参和视场')
            for publisher, cloud in zip(self.cloud_publishers, [filtered, ground, obstacles]):
                publisher.publish(cloud_message(cloud, header))
            self.publish_boxes(boxes, header)
            self.counter += 1
            if now - self.last_report >= 1.:
                self.last_report = now
                write_json(self.output / 'obstacles.json', {'frames': self.counter, 'tf_drops': self.dropped,
                           'input_points': len(base), 'filtered_points': len(filtered),
                           'ground_points': len(ground), 'obstacle_points': len(obstacles),
                           'estimated_ground_z_m': float(np.median((ground @ gravity_rotation(up).T)[:, 2])),
                           'up_source': up_source, 'processing_ms': (time.monotonic()-now)*1000, 'boxes': boxes})
        except Exception as exc:
            self.dropped += 1
            if now - self.last_warning > 2.:
                self.last_warning = now
                self.get_logger().warning('本帧不发布：' + str(exc))

    def publish_boxes(self, boxes, header):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for i, box in enumerate(boxes):
            lo, hi = np.asarray(box['min']), np.asarray(box['max'])
            marker = Marker(header=header, ns='obstacles', id=i, type=Marker.CUBE, action=Marker.ADD)
            center, size = (lo+hi)/2, np.maximum(hi-lo, .05)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, center)
            marker.pose.orientation.w = 1.
            marker.scale.x, marker.scale.y, marker.scale.z = map(float, size)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .2, .1, .45
            marker.lifetime.sec = 1
            array.markers.append(marker)
        self.markers.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleExperiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
