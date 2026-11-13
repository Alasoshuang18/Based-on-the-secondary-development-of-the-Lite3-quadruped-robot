import numpy as np
from lite3_lab_3_3.algorithms import process_cloud, gravity_rotation, connected_boxes

def test_floor_is_not_obstacle_and_body_is_removed():
    x,y=np.meshgrid(np.linspace(1,3,20),np.linspace(-1,1,15))
    floor=np.c_[x.ravel(),y.ravel(),np.full(x.size,-0.45)]
    cube=np.array([[2+.03*i,.2+.03*j,z] for i in range(5) for j in range(5) for z in [-.25,0,.2]])
    p=np.vstack([floor,cube,[[0,0,0],[np.nan,0,0]]])
    filtered,ground,obstacles,boxes=process_cloud(p, voxel=.025, min_cluster_points=5)
    assert len(ground)>200 and len(obstacles)>30
    assert np.min(obstacles[:,2])>-.35
    assert len(boxes)==1 and not np.any(np.linalg.norm(filtered,axis=1)<.1)

def test_tilt_alignment_and_grid_components():
    a=np.deg2rad(15);up=np.array([0,np.sin(a),np.cos(a)])
    r=gravity_rotation(up)
    assert np.allclose(r@up,[0,0,1],atol=1e-7)
    assert np.allclose(r@r.T,np.eye(3),atol=1e-7)
    p=np.array([[1,0,0],[1.05,.05,.2],[3,1,.1],[3.05,1.05,.3]])
    assert len(connected_boxes(p,.15,2))==2


def test_tilted_floor_is_classified_in_gravity_frame():
    x,y=np.meshgrid(np.linspace(1,3,12),np.linspace(-1,1,12))
    level=np.c_[x.ravel(),y.ravel(),np.full(x.size,-.45)]
    angle=np.deg2rad(18)
    up=np.array([0,np.sin(angle),np.cos(angle)])
    rotation=gravity_rotation(up)
    base=level@rotation
    _,floor,obstacles,_=process_cloud(base,up=up,voxel=.02)
    assert len(floor)==len(level) and len(obstacles)==0


def test_invalid_cloud_is_empty_and_invalid_gravity_rejected():
    import pytest
    filtered,ground,obstacles,boxes=process_cloud([[np.nan,0,0],[0,0,0]])
    assert filtered.shape==(0,3) and len(ground)==0 and len(obstacles)==0 and boxes==[]
    with pytest.raises(ValueError):gravity_rotation([0,0,0])


def test_cluster_uses_gravity_aligned_xy_but_box_stays_in_base():
    up=np.array([0,2**-.5,2**-.5])
    rotation=gravity_rotation(up)
    base=np.array([[1,0,-.2],[1,0,.8]])@rotation
    _,_,obstacles,boxes=process_cloud(base,up=up,voxel=.02,min_cluster_points=2)
    assert len(boxes)==1
    assert np.allclose(boxes[0]['min'],base.min(axis=0),atol=1e-6)
