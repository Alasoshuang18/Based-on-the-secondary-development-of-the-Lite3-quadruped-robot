"""适合ARM的NumPy点云流程：距离、体素、机身、重力、地面、栅格连通。"""
from collections import deque
import numpy as np


def gravity_rotation(up):
    """Rodrigues旋转：将base坐标中的向上单位向量变成level的+Z。"""
    a = np.asarray(up, dtype=float)
    norm = np.linalg.norm(a)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('重力向量无效')
    a = a / norm
    b = np.array([0., 0., 1.])
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c > 1 - 1e-10:
        return np.eye(3)
    if c < -1 + 1e-10:
        return np.diag([1., -1., -1.])
    k = np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
    return np.eye(3) + k + k @ k / (1 + c)


def connected_boxes(points, cell_size=.18, min_points=6, grid_points=None):
    """在level的XY格连通，仍按原base点返回框；避免N×N距离矩阵。"""
    p = np.asarray(points).reshape(-1, 3)
    if len(p) == 0:
        return []
    if cell_size <= 0:
        raise ValueError('栅格边长必须为正')
    grid = p if grid_points is None else np.asarray(grid_points).reshape(-1, 3)
    if len(grid) != len(p):
        raise ValueError('聚类坐标与输出坐标必须逐点对应')
    keys = np.floor(grid[:, :2] / cell_size).astype(np.int32)
    cells = {}
    for i, key in enumerate(keys):
        cells.setdefault((int(key[0]), int(key[1])), []).append(i)
    unseen = set(cells)
    boxes = []
    while unseen:
        seed = unseen.pop()
        queue, indices = deque([seed]), []
        while queue:
            x, y = queue.popleft()
            indices.extend(cells[(x, y)])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    key = (x + dx, y + dy)
                    if key in unseen:
                        unseen.remove(key)
                        queue.append(key)
        if len(indices) >= min_points:
            group = p[indices]
            lo, hi = group.min(axis=0), group.max(axis=0)
            boxes.append({'min': lo.tolist(), 'max': hi.tolist(), 'points': len(indices)})
    return boxes


def process_cloud(points, up=(0., 0., 1.), voxel=.08, min_range=.35, max_range=8.,
                  body_min=(-.45, -.25, -.45), body_max=(.45, .25, .3),
                  ground_z=-.40, ground_tolerance=.09, ground_max_slope_deg=20.,
                  obstacle_min_height=.12, obstacle_max_height=1.8,
                  cell_size=.18, min_cluster_points=6, max_points=120000):
    """返回base坐标下filtered、ground、obstacles和base轴对齐包围框。

    用level坐标拟合近地低点平面；只允许法向接近重力，墙面不会当成地面。
    ground_z是base原点到地面的负高度，站立高度变化时需调整或使用姿态/高度估计。
    """
    if voxel <= 0 or max_points < 1 or max_range <= min_range:
        raise ValueError('体素、点数上限或距离范围无效')
    p = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    p = p[np.all(np.isfinite(p), axis=1)]
    # 输入太大时按确定性步长抽样，控制ARM机内存和每帧处理时间。
    if len(p) > max_points:
        p = p[::int(np.ceil(len(p) / max_points))]
    r2 = np.einsum('ij,ij->i', p, p)
    p = p[(r2 >= min_range ** 2) & (r2 <= max_range ** 2)]
    inside = np.all((p >= np.asarray(body_min)) & (p <= np.asarray(body_max)), axis=1)
    p = p[~inside]
    if not len(p):
        empty = np.empty((0, 3), dtype=np.float32)
        return empty, empty.copy(), empty.copy(), []
    # 每体素选一个实际测量点。无需Open3D/PCL，不产生不必要的质心数组。
    keys = np.floor(p / voxel).astype(np.int32)
    _, index = np.unique(keys, axis=0, return_index=True)
    p = p[index]
    level = p @ gravity_rotation(up).T
    seed = np.abs(level[:, 2] - ground_z) < 2 * ground_tolerance
    plane = np.array([0., 0., ground_z])  # z = a*x + b*y + c
    # 初筛限制离地高度，再做两次鲁棒最小二乘，剔除矮障碍。
    if np.count_nonzero(seed) >= 12:
        a = np.c_[level[:, :2], np.ones(len(level))]
        chosen = seed.copy()
        for _ in range(2):
            if np.count_nonzero(chosen) < 12:
                break
            fitted = np.linalg.lstsq(a[chosen], level[chosen, 2], rcond=None)[0]
            if np.linalg.norm(fitted[:2]) > np.tan(np.deg2rad(ground_max_slope_deg)):
                break
            if abs(fitted[2] - ground_z) > 2 * ground_tolerance:
                break
            plane = fitted
            residual = level[:, 2] - a @ plane
            chosen = seed & (np.abs(residual) <= ground_tolerance)
    height = level[:, 2] - (level[:, :2] @ plane[:2] + plane[2])
    ground = p[np.abs(height) <= ground_tolerance]
    is_obstacle = (height >= obstacle_min_height) & (height <= obstacle_max_height)
    obstacles = p[is_obstacle]
    # 聚类按重力对齐的XY，框按base实测点计算，两者通过同一掩码保持对应。
    boxes = connected_boxes(obstacles, cell_size, min_cluster_points, level[is_obstacle])
    return p, ground, obstacles, boxes
