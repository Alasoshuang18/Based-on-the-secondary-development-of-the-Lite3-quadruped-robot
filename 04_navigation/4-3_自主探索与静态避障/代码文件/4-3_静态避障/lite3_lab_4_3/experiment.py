"""真实障碍点云与完整足腿包络的净空；规划膨胀仅作为独立证据。"""
import csv
import math
import time
import numpy as np
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from lite3_course_common.ros_utils import cloud_xyz, transform_xyz
from lite3_lab_5_1.runtime import NavigationExperiment, run_node
from lite3_lab_5_1.safety import fresh_source
from lite3_lab_5_1.geometry import yaw_from_quaternion
from .clearance import sensor_clearance, footprint_covers_envelope, costmap_edge_clearance, ClearanceAudit


class StaticAvoidanceExperiment(NavigationExperiment):
    def __init__(self):
        super().__init__('lite3_lab_5_3', {
            'required_clearance_m': .1, 'envelope_verified': False,
            'sensor_coverage_verified': False, 'motion_bound_verified': False,
            'obstacle_topic': '/course/obstacles', 'sensor_error_bound_m': .15,
            'max_linear_speed_mps': .02, 'max_yaw_rate_rps': .02,
            'max_sensor_gap_s': .15, 'costmap_inflation_radius_m': .9,
            'costmap_cost_scaling_factor': 3.0, 'inflation_edge_required_clearance_m': .1,
            'inflation_cost_threshold': 1, 'max_costmap_gap_s': 1., 'max_costmap_footprint_skew_s': .15})
        self.clearance_audit = ClearanceAudit(self.p('required_clearance_m'),
                self.p('envelope_verified'), self.p('sensor_coverage_verified'))
        self.clearance_audit.coverage_valid = self.p('motion_bound_verified')
        self.inflation_audit = ClearanceAudit(self.p('inflation_edge_required_clearance_m'),
                self.p('envelope_verified'), True)
        self.inflation_audit.coverage_valid = self.p('motion_bound_verified')
        self.latest_footprint_msg = None
        self.last_inflation_stamp = self.last_inflation_clearance = None
        self.sensor_stamp = self.sensor_wall = self.last_clearance_stamp = self.last_clearance_value = None
        self.first_navigation_stamp = None
        self.footprint_stamp = self.footprint_wall = None
        self.footprint_valid = False
        self.costmap_stamp = self.costmap_wall = None
        self.costmap_snapshot = None
        self.measurement_failures = []
        self.clearance_file = (self.result_dir / 'csv' / 'clearance.csv').open('w', newline='', encoding='utf-8')
        self.clearance_csv = csv.writer(self.clearance_file)
        self.clearance_csv.writerow(['sim_s', 'wall_monotonic_s', 'surface_clearance_m',
                'motion_bound_m', 'gap_s', 'points', 'source', 'physical_length_m', 'physical_width_m'])
        self.inflation_file = (self.result_dir / 'csv' / 'inflation_edge_clearance.csv').open('w', newline='', encoding='utf-8')
        self.inflation_csv = csv.writer(self.inflation_file)
        self.inflation_csv.writerow(['ros_s','wall_monotonic_s','footprint_to_inflation_lower_bound_m',
                'motion_bound_m','sample_gap_s','footprint_stamp_skew_s','grid_half_diagonal_m',
                'unknown_cells','threshold_occupancy_cost'])
        self.create_subscription(PointCloud2, self.p('obstacle_topic'), self.obstacle_cloud, qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint', self.footprint, 10)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap, durable)

    def invalidate(self, reason):
        if self.started:
            self.clearance_audit.coverage_valid = False
        if reason not in self.measurement_failures:
            self.measurement_failures.append(reason)
            self.event('sensor_clearance_unavailable', reason=reason)

    def footprint(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        wall = time.monotonic()
        if not fresh_source(stamp, wall, self.sim_now(), wall, 1.5, 1.5, self.footprint_stamp):
            return
        try:
            p = np.asarray([[p.x, p.y, p.z] for p in msg.polygon.points])
            base = transform_xyz(self, self.tf_buffer, p, msg.header, self.p('base_frame'))
            valid = footprint_covers_envelope(base[:, :2], self.p('robot_length_m'), self.p('robot_width_m'))
        except Exception as exc:
            self.invalidate('footprint_tf_or_geometry: '+str(exc))
            return
        self.footprint_stamp, self.footprint_wall, self.footprint_valid = stamp, wall, valid
        self.latest_footprint_msg = msg
        self.event('nav2_published_footprint', base_xy=base[:, :2].tolist(), covers_physical_envelope=valid,
                   configured_padding_m=self.p('footprint_padding_m'), padding_is_not_entity_surface=True)

    def costmap(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        wall = time.monotonic()
        if not fresh_source(stamp, wall, self.sim_now(), wall, 1.5, 1.5, self.costmap_stamp):
            return
        if len(msg.data) != msg.info.width*msg.info.height or msg.info.resolution <= 0:
            self.invalidate('invalid_local_costmap')
            return
        self.costmap_stamp, self.costmap_wall = stamp, wall
        self.costmap_snapshot = {'stamp_ros_s': stamp, 'frame': msg.header.frame_id,
              'resolution_m': msg.info.resolution, 'width': msg.info.width, 'height': msg.info.height,
              'origin': [msg.info.origin.position.x,msg.info.origin.position.y,
                         yaw_from_quaternion(msg.info.origin.orientation)],
              'inflated_grid_is_not_physical_surface_measurement': True}
        footprint = self.latest_footprint_msg
        skew = None if self.footprint_stamp is None else abs(stamp-self.footprint_stamp)
        if footprint is None or skew > self.p('max_costmap_footprint_skew_s'):
            if self.started:
                self.inflation_audit.coverage_valid = False
            self.event('inflation_edge_measurement_unavailable',reason='missing_or_unsynchronized_real_footprint')
            return
        try:
            polygon = np.asarray([[p.x,p.y,p.z] for p in footprint.polygon.points])
            points = transform_xyz(self,self.tf_buffer,polygon,footprint.header,msg.header.frame_id)
            origin = self.costmap_snapshot['origin']
            result = costmap_edge_clearance(msg.data,msg.info.width,msg.info.height,msg.info.resolution,
                     origin[:2],origin[2],points[:,:2],self.p('inflation_cost_threshold'))
        except Exception as exc:
            if self.started:
                self.inflation_audit.coverage_valid = False
            self.event('inflation_edge_measurement_unavailable',reason=str(exc))
            return
        self.costmap_snapshot['inflation_edge_measurement'] = result
        if not result['inflation_edge_observed'] or result['conservative_clearance_m'] is None:
            if self.started:
                self.inflation_audit.coverage_valid = False
            return
        if not self.started or stamp < self.first_navigation_stamp:
            return
        # 不同实际消息时刻的轮廓另扣运动界，不能假装是同时间真值。
        clearance = max(0.,result['conservative_clearance_m']-skew*self.motion_rate_bound())
        first = self.last_inflation_stamp is None
        gap = stamp-(self.first_navigation_stamp if first else self.last_inflation_stamp)
        if not 0 <= gap <= self.p('max_costmap_gap_s'):
            self.inflation_audit.coverage_valid = False
        bound = max(0.,gap)*self.motion_rate_bound()
        self.inflation_audit.add(clearance,bound,max(0.,gap))
        if first:
            self.inflation_audit.add_endpoint_gap(clearance,gap,self.motion_rate_bound(),self.p('max_costmap_gap_s'),'start')
        self.last_inflation_stamp,self.last_inflation_clearance = stamp,clearance
        self.inflation_csv.writerow([stamp,wall,clearance,bound,gap,skew,result['cell_half_diagonal_m'],
                                     result['unknown_cells'],result['threshold_occupancy_cost']])
        self.inflation_file.flush()
        if clearance-bound < self.p('inflation_edge_required_clearance_m'):
            self.cancel_navigation('actual_costmap_inflation_edge_clearance_below_requirement')

    def obstacle_cloud(self, msg):
        if self.finished:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        wall = time.monotonic()
        if not fresh_source(stamp, wall, self.sim_now(), wall, 1.5, 1.5, self.sensor_stamp):
            self.invalidate('original_cloud_header_stale_future_or_reordered')
            return
        try:
            xyz = transform_xyz(self, self.tf_buffer, cloud_xyz(msg), msg.header, self.p('base_frame'))
            clearance = sensor_clearance(xyz, self.p('robot_length_m'), self.p('robot_width_m'),
                                         self.p('sensor_error_bound_m'))
        except Exception as exc:
            self.invalidate('cloud_tf_or_measurement: '+str(exc))
            return
        self.sensor_stamp, self.sensor_wall = stamp, wall
        if not self.started or stamp < self.first_navigation_stamp:
            return
        first = self.last_clearance_stamp is None
        gap = stamp-(self.first_navigation_stamp if first else self.last_clearance_stamp)
        if not 0 <= gap <= self.p('max_sensor_gap_s'):
            self.invalidate('obstacle_observation_gap')
        motion = max(0., gap)*self.motion_rate_bound()
        self.clearance_audit.add(clearance, motion, max(0., gap))
        if first:
            self.clearance_audit.add_endpoint_gap(clearance, gap, self.motion_rate_bound(),
                                                  self.p('max_sensor_gap_s'), 'start')
        self.last_clearance_stamp, self.last_clearance_value = stamp, clearance
        self.clearance_csv.writerow([stamp, wall, clearance, motion, gap, len(xyz), self.p('obstacle_topic'),
                                     self.p('robot_length_m'), self.p('robot_width_m')])
        self.clearance_file.flush()
        if clearance-motion < self.p('required_clearance_m'):
            self.event('conservative_sensor_clearance_below_requirement', lower_bound_m=clearance-motion)
            self.cancel_navigation('measured_clearance_below_requirement')

    def health(self):
        reasons = super().health()
        if not hasattr(self, 'sensor_stamp'):
            return reasons+['实测净空评估尚未初始化']
        ros, wall = self.sim_now(), time.monotonic()
        for name, stamp, received in (('obstacle_cloud', self.sensor_stamp, self.sensor_wall),
                ('local_costmap', self.costmap_stamp, self.costmap_wall),
                ('published_footprint', self.footprint_stamp, self.footprint_wall)):
            if not fresh_source(stamp, received, ros, wall, 1.5, 1.5):
                reasons.append(name+'缺失或过期')
        if not self.footprint_valid:
            reasons.append('Nav2实际footprint未覆盖实测完整足腿包络')
        return reasons

    def begin_experiment(self):
        self.first_navigation_stamp = self.sim_now()
        self.send_goal(self.goal_pose())

    def motion_rate_bound(self):
        radius = math.hypot(self.p('robot_length_m'), self.p('robot_width_m'))/2
        return self.p('max_linear_speed_mps')+radius*self.p('max_yaw_rate_rps')

    def summary_extra(self):
        terminal_inputs_fresh = all(fresh_source(stamp, received, self.sim_now(), time.monotonic(), 1.5, 1.5)
                for stamp, received in ((self.sensor_stamp, self.sensor_wall),
                (self.costmap_stamp, self.costmap_wall), (self.footprint_stamp, self.footprint_wall)))
        if not terminal_inputs_fresh or not self.footprint_valid:
            self.clearance_audit.coverage_valid = False
            self.inflation_audit.coverage_valid = False
        if self.last_clearance_stamp is None:
            self.clearance_audit.coverage_valid = False
        elif self.started:
            self.clearance_audit.add_endpoint_gap(self.last_clearance_value,
                    self.sim_now()-self.last_clearance_stamp, self.motion_rate_bound(), self.p('max_sensor_gap_s'), 'end')
        result = self.clearance_audit.result()
        result['acceptance_pass'] = bool(result['acceptance_pass'] and self.navigation_success and self.footprint_valid)
        if self.last_inflation_stamp is None:
            self.inflation_audit.coverage_valid = False
        elif self.started:
            self.inflation_audit.add_endpoint_gap(self.last_inflation_clearance,self.sim_now()-self.last_inflation_stamp,
                    self.motion_rate_bound(),self.p('max_costmap_gap_s'),'end')
        inflation = self.inflation_audit.result()
        inflation['acceptance_pass'] = bool(inflation['acceptance_pass'] and self.navigation_success and self.footprint_valid)
        return {'clearance_audit': result, 'measurement_source': self.p('obstacle_topic'),
                'inflation_edge_clearance_audit': inflation,
                'static_avoidance_acceptance_pass': bool(result['acceptance_pass'] and inflation['acceptance_pass']),
                'inflation_edge_outline_source': '/local_costmap/published_footprint_including_planning_padding',
                'clearance_kind': 'conservative_observed_surface_not_ground_truth',
                'physical_envelope_m': [self.p('robot_length_m'), self.p('robot_width_m')],
                'sensor_error_bound_m': self.p('sensor_error_bound_m'),
                'measurement_failures': self.measurement_failures, 'local_costmap': self.costmap_snapshot,
                'published_footprint_covers_measured_envelope': self.footprint_valid,
                'terminal_measurement_inputs_fresh': terminal_inputs_fresh,
                'planning_padding_m': self.p('footprint_padding_m'),
                'configured_inflation_snapshot_only': {'radius_m': self.p('costmap_inflation_radius_m'),
                                  'cost_scaling_factor': self.p('costmap_cost_scaling_factor')},
                'costmap_inflation_is_not_measured_entity_clearance': True}

    def close_experiment(self):
        super().close_experiment()
        self.clearance_file.close()
        self.inflation_file.close()


def main(args=None):
    run_node(StaticAvoidanceExperiment, args)
