"""不依赖 ROS 的定位健康度计算；距离为米，时间为秒。"""
import math
import numpy as np


def weighted_region(xy, weights, mass=0.95):
    """围绕加权均值按半径收集至少 mass 权重，求该集合的真实最大直径。

    此处是粒子后验的有限样本可信区域，不是独立真值精度证明。边界同半径
    粒子全部纳入，避免相同权重时因数组顺序得到过小直径。
    """
    points = np.asarray(xy, dtype=float)
    weight = np.asarray(weights, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) != len(weight)
            or not len(points) or not np.isfinite(points).all()
            or not np.isfinite(weight).all() or np.any(weight < 0)
            or not np.any(weight > 0) or not 0 < mass <= 1):
        raise ValueError('粒子坐标、非负权重和可信质量必须有效')
    # 先缩放到[0,1]，避免有限但很大的输入权重求和溢出。
    weight = weight / weight.max()
    weight = weight / weight.sum()
    center = np.sum(points * weight[:, None], axis=0)
    radius = np.linalg.norm(points - center, axis=1)
    order = np.argsort(radius, kind='stable')
    boundary = radius[order[min(np.searchsorted(np.cumsum(weight[order]), mass), len(order) - 1)]]
    selected = radius <= boundary + 1e-12
    region = points[selected]
    maximum = 0.0
    # 分块计算精确最大两点距离，避免 N×N×2 一次占用太多内存。
    for start in range(0, len(region), 256):
        dist2 = np.sum((region[start:start + 256, None, :] - region[None, :, :]) ** 2, axis=2)
        maximum = max(maximum, float(np.max(dist2)))
    return {'diameter_m': math.sqrt(maximum), 'covered_weight': float(weight[selected].sum()),
            'particle_count': len(points), 'region_count': len(region), 'center_xy': center.tolist()}


class StableGate:
    """连续满足才开启；无效样本和时钟倒退立即清除持续时间。"""
    def __init__(self, duration_s):
        self.duration = float(duration_s)
        self.since = None
        self.last = None

    def update(self, valid, now):
        if self.last is not None and now < self.last:
            self.since = None
        self.last = now
        if not valid:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.duration


def map_health(data, width, height, resolution):
    grid = np.asarray(data, dtype=int)
    reasons = []
    if width <= 0 or height <= 0 or grid.size != width * height:
        reasons.append('地图尺寸与数据长度不一致')
    if not math.isfinite(resolution) or abs(resolution - 0.05) > 1e-6:
        reasons.append('分辨率必须为0.05m')
    if np.any((grid < -1) | (grid > 100)):
        reasons.append('占据概率超出[-1,100]')
    free = int(np.sum((grid >= 0) & (grid <= 25)))
    known = int(np.sum(grid >= 0))
    if free < 10:
        reasons.append('已知自由栅格少于10个')
    return {'healthy': not reasons, 'reasons': reasons, 'width': width, 'height': height,
            'resolution_m': resolution, 'free_cells': free,
            'known_fraction': float(known / max(1, grid.size))}


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
