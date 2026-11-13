"""不依赖ROS的点云统计；空数据使用None，避免伪造有效距离。"""
import numpy as np


def cloud_statistics(points):
    p = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    finite = np.all(np.isfinite(p), axis=1)
    good = p[finite]
    distances = np.linalg.norm(good, axis=1)
    return {'points': int(len(p)), 'finite_points': int(len(good)),
            'invalid_points': int(np.count_nonzero(~finite)),
            'range_min_m': float(distances.min()) if len(good) else None,
            'range_max_m': float(distances.max()) if len(good) else None,
            'bounds_min_m': good.min(axis=0).tolist() if len(good) else None,
            'bounds_max_m': good.max(axis=0).tolist() if len(good) else None}
