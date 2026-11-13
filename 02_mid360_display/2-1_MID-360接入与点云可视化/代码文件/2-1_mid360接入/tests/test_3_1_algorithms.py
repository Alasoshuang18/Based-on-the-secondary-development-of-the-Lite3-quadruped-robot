import numpy as np
from lite3_lab_3_1.algorithms import cloud_statistics

def test_invalid_and_empty_cloud_are_not_healthy():
    s=cloud_statistics(np.array([[np.nan,0,0],[1,0,0],[0,2,0]],dtype=float))
    assert s['finite_points']==2 and s['invalid_points']==1
    assert abs(s['range_min_m']-1)<1e-6
    assert cloud_statistics(np.empty((0,3)))['range_min_m'] is None
