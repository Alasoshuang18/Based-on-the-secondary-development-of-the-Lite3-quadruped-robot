"""实体障碍与保守足腿扫掠包络的二维净空，重叠返回0。"""
import math
import numpy as np


def costmap_edge_clearance(data, width, height, resolution, origin_xy, origin_yaw,
                          footprint_xy, threshold=1, max_cells=250000):
    """真实OccupancyGrid膨胀区域到实际发布footprint的保守距离。

    Humble costmap占据消息将膨胀代价映射为1..98，99/100为内切/致命；
    默认所有>=1格均视为膨胀/障碍区域，unknown也视为不可通过。
    到格中心的距离减去半对角，得到包含格面积的保守下界，不能当实体表面。
    """
    values=np.asarray(data,float)
    p=np.asarray(footprint_xy,float)
    if (width < 1 or height < 1 or width*height > max_cells or values.size != width*height
            or not np.isfinite(values).all() or np.any((values < -1)|(values > 100))
            or not math.isfinite(resolution) or resolution <= 0 or not 1 <= threshold <= 100
            or p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all()
            or not np.isfinite(origin_xy).all() or not math.isfinite(origin_yaw)):
        raise ValueError('invalid_or_over_budget_costmap_geometry')
    c,s=math.cos(origin_yaw),math.sin(origin_yaw)
    # map/odom中的真实轮廓转换到栅格局部坐标，使用origin平移和偏航。
    p=(p-np.asarray(origin_xy)) @ np.asarray([[c,-s],[s,c]])
    if np.any(p < 0) or np.any(p > np.asarray([width,height])*resolution):
        raise ValueError('footprint_outside_observed_costmap')
    values=values.reshape(height,width)
    occupied=values >= threshold
    unknown=values < 0
    yy,xx=np.nonzero(occupied|unknown)
    if not len(xx):
        return {'conservative_clearance_m':None,'inflation_edge_observed':False,'unknown_cells':0,
                'threshold_occupancy_cost':threshold,'cell_half_diagonal_m':resolution/math.sqrt(2)}
    centers=np.c_[xx+.5,yy+.5]*resolution
    minimum=np.full(len(centers),np.inf)
    positive=np.ones(len(centers),bool); negative=positive.copy()
    for a,b in zip(p,np.roll(p,-1,axis=0)):
        edge=b-a; delta=centers-a
        cross=edge[0]*delta[:,1]-edge[1]*delta[:,0]
        positive &= cross >= -1e-9; negative &= cross <= 1e-9
        ratio=np.clip((delta@edge)/max(float(edge@edge),1e-15),0.,1.)
        minimum=np.minimum(minimum,np.linalg.norm(delta-ratio[:,None]*edge,axis=1))
    minimum[positive|negative]=0.
    return {'conservative_clearance_m':max(0.,float(minimum.min())-resolution/math.sqrt(2)),
            'inflation_edge_observed':bool(occupied.any()),'unknown_cells':int(unknown.sum()),
            'threshold_occupancy_cost':threshold,'unknown_handling':'forbidden',
            'cell_half_diagonal_m':resolution/math.sqrt(2)}


def sensor_clearance(points, length, width, error_m, max_points=120000):
    """被观测表面到全身矩形的最近距离，扣除实测误差；遮挡覆盖另行核验。"""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(p) > max_points:
        raise ValueError('point_budget_exceeded')
    if not len(p) or not np.isfinite(p).all():
        raise ValueError('missing_or_invalid_obstacle_points')
    if min(length, width) <= 0 or not math.isfinite(error_m) or error_m < 0:
        raise ValueError('invalid_measured_envelope_or_error')
    distance = np.linalg.norm(np.maximum(np.abs(p[:, :2]) - [length/2, width/2], 0.), axis=1)
    return max(0., float(distance.min()) - error_m)


def footprint_covers_envelope(points, length, width):
    """实际凸footprint须包含测量矩形四角，顺逆时针均可。"""
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all():
        return False
    corners = rectangle(0., 0., 0., length, width)
    edges = np.roll(p, -1, axis=0) - p
    if abs(float(np.sum(p[:, 0]*np.roll(p[:, 1], -1)-p[:, 1]*np.roll(p[:, 0], -1)))) < 1e-8:
        return False
    for point in corners:
        delta = point - p
        cross = edges[:, 0]*delta[:, 1]-edges[:, 1]*delta[:, 0]
        if not (np.all(cross >= -1e-6) or np.all(cross <= 1e-6)):
            return False
    return True


def rectangle(x, y, yaw, length, width):
    if min(length, width) <= 0:
        raise ValueError('矩形尺寸必须正数')
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.asarray([[-length / 2, -width / 2], [length / 2, -width / 2],
                        [length / 2, width / 2], [-length / 2, width / 2]])
    return local @ np.asarray([[c, s], [-s, c]]) + [x, y]


def point_segment_distance(point, a, b):
    direction = b - a
    norm = float(np.dot(direction, direction))
    ratio = 0.0 if norm < 1e-15 else float(np.clip(np.dot(point - a, direction) / norm, 0, 1))
    return float(np.linalg.norm(point - (a + ratio * direction)))


def polygon_clearance(first, second):
    """凸多边形先SAT判相交，再算全部顶点到对侧边的最近距离。"""
    a, b = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    separated = False
    for poly in (a, b):
        for i in range(len(poly)):
            edge = poly[(i + 1) % len(poly)] - poly[i]
            axis = np.asarray([-edge[1], edge[0]])
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() - 1e-12 or pb.max() < pa.min() - 1e-12:
                separated = True
    if not separated:
        return 0.0
    return min(point_segment_distance(p, poly[i], poly[(i + 1) % len(poly)])
               for vertices, poly in ((a, b), (b, a)) for p in vertices for i in range(len(poly)))


class ClearanceAudit:
    def __init__(self, required, envelope_verified, geometry_verified):
        self.required = float(required)
        self.envelope_verified, self.geometry_verified = envelope_verified, geometry_verified
        self.minimum = None
        self.lower_bound = None
        self.samples = 0
        self.max_gap_s = 0.0
        self.coverage_valid = True
        self.endpoint_gaps = {}

    def add(self, clearance, motion_bound, gap_s=0.0):
        self.samples += 1
        self.minimum = clearance if self.minimum is None else min(self.minimum, clearance)
        lower = clearance - motion_bound
        self.lower_bound = lower if self.lower_bound is None else min(self.lower_bound, lower)
        self.max_gap_s = max(self.max_gap_s, gap_s)

    def result(self):
        checked = bool(self.samples and self.envelope_verified and self.geometry_verified and self.coverage_valid)
        sampled = self.minimum is not None and self.minimum >= self.required
        continuous = self.lower_bound is not None and self.lower_bound >= self.required
        return {'samples': self.samples, 'minimum_clearance_m': self.minimum,
                'continuous_lower_bound_m': self.lower_bound, 'max_sample_gap_s': self.max_gap_s,
                'required_clearance_m': self.required, 'sampled_threshold_pass': bool(sampled),
                'envelope_verified': self.envelope_verified, 'geometry_verified': self.geometry_verified,
                'coverage_valid': self.coverage_valid,
                'endpoint_gaps': self.endpoint_gaps,
                'acceptance_evaluated': checked, 'acceptance_pass': bool(checked and continuous)}

    def add_endpoint_gap(self, clearance, gap_s, motion_rate_mps, max_gap_s, edge):
        """首末未采样时隙同样扣除(v+Rω)Δt；超限或时序异常不给验收。"""
        if edge not in ('start', 'end'):
            raise ValueError('端点名称必须为start/end')
        if not all(math.isfinite(value) for value in (clearance, gap_s, motion_rate_mps, max_gap_s)):
            self.coverage_valid = False
            raise ValueError('端点界必须为有限数')
        if gap_s < 0 or gap_s > max_gap_s or motion_rate_mps < 0:
            self.coverage_valid = False
        motion_bound = max(0.0, gap_s) * max(0.0, motion_rate_mps)
        lower = clearance - motion_bound
        self.lower_bound = lower if self.lower_bound is None else min(self.lower_bound, lower)
        self.max_gap_s = max(self.max_gap_s, max(0.0, gap_s))
        self.endpoint_gaps[edge] = {'gap_s': gap_s, 'motion_bound_m': motion_bound,
                                    'endpoint_clearance_m': clearance, 'lower_bound_m': lower}
