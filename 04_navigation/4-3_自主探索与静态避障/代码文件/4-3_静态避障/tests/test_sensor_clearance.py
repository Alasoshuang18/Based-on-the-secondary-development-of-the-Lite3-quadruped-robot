import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_3.clearance import sensor_clearance, footprint_covers_envelope, costmap_edge_clearance


def test_nearest_actual_point_uses_full_leg_envelope_and_error_margin():
    value = sensor_clearance([[.9, 0, .1], [2, 2, .3]], 1.2, .8, .03)
    assert value == pytest.approx(.27)
    assert sensor_clearance([[.5, 0, .1]], 1.2, .8, .03) == 0.


def test_empty_or_over_budget_cloud_cannot_be_reported_as_clear():
    with pytest.raises(ValueError):
        sensor_clearance([], 1.2, .8, .03)
    with pytest.raises(ValueError, match='point_budget'):
        sensor_clearance(np.zeros((11, 3)), 1.2, .8, .03, max_points=10)


def test_nav2_published_footprint_must_contain_measured_full_body_envelope():
    assert footprint_covers_envelope([[-.7,-.5],[.7,-.5],[.7,.5],[-.7,.5]],1.2,.8)
    assert not footprint_covers_envelope([[-.5,-.2],[.5,-.2],[.5,.2],[-.5,.2]],1.2,.8)
    assert not footprint_covers_envelope([], 1.2, .8)


def test_inflation_edge_distance_is_independent_of_entity_pointcloud_distance():
    grid=np.zeros((10,10),dtype=int); grid[5,8]=1
    outline=[[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]]
    result=costmap_edge_clearance(grid.ravel(),10,10,.1,[-.5,-.5],0.,outline)
    assert result['conservative_clearance_m'] == pytest.approx(.25-.1/np.sqrt(2))
    assert result['inflation_edge_observed']
    # 未知格不得当自由空间忽略。
    grid[5,6]=-1
    result=costmap_edge_clearance(grid.ravel(),10,10,.1,[-.5,-.5],0.,outline)
    assert result['conservative_clearance_m'] == 0.
    assert result['unknown_cells'] == 1


def test_costmap_origin_rotation_is_used_and_missing_edges_remain_unevaluated():
    grid=np.zeros((10,10),dtype=int); grid[5,8]=99
    outline=np.asarray([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
    a=costmap_edge_clearance(grid.ravel(),10,10,.1,[-.5,-.5],0.,outline)
    rotation=np.asarray([[0.,-1.],[1.,0.]])
    b=costmap_edge_clearance(grid.ravel(),10,10,.1,[1.5,1.5],np.pi/2,outline@rotation.T+[1.,2.])
    assert a['conservative_clearance_m'] == pytest.approx(b['conservative_clearance_m'])
    empty=costmap_edge_clearance(np.zeros(100),10,10,.1,[-.5,-.5],0.,outline)
    assert not empty['inflation_edge_observed']
    assert empty['conservative_clearance_m'] is None
    with pytest.raises(ValueError):
        costmap_edge_clearance(grid.ravel(),10,10,.1,[-.5,-.5],0.,outline+2.)
