import copy
import csv
import json
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from lite3_lab_5_2.navigation_profile import with_tracking_headroom
from lite3_lab_5_2.motion_evidence import MotionEvidence


def test_lower_command_profile_preserves_geometry_collision_checks_and_input():
    original={'controller_server':{'ros__parameters':{'FollowPath':{
        'desired_linear_vel':.2,'use_collision_detection':True,'max_allowed_time_to_collision_up_to_carrot':4.,
        'rotate_to_heading_angular_vel':.2}}},
        'velocity_smoother':{'ros__parameters':{'max_velocity':[.2,0.,.2],'max_accel':[.05,0.,.1]}},
        'local_costmap':{'footprint':[[.3895,.24],[-.3895,-.24]],'footprint_padding':.10}}
    before=copy.deepcopy(original)
    result=with_tracking_headroom(original)
    assert original==before
    assert result['local_costmap']==before['local_costmap']
    follow=result['controller_server']['ros__parameters']['FollowPath']
    smoother=result['velocity_smoother']['ros__parameters']
    assert follow['desired_linear_vel']==follow['rotate_to_heading_angular_vel']==.1
    assert smoother['max_velocity']==[.1,0.,.1]
    assert smoother['scale_velocities']
    assert follow['use_collision_detection']
    assert follow['max_allowed_time_to_collision_up_to_carrot']==4.
    assert smoother['max_accel']==[.05,0.,.1]
    assert follow['desired_linear_vel']/follow['regulated_linear_scaling_min_radius']<=smoother['max_velocity'][2]


def test_motion_trace_keeps_a_single_frame_yaw_exceedance_and_actual_command(tmp_path):
    (tmp_path/'csv').mkdir()
    trace=MotionEvidence(tmp_path)
    message=NS(header=NS(stamp=NS(sec=10,nanosec=0)),child_frame_id='base_link',
        pose=NS(pose=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),
        twist=NS(twist=NS(linear=NS(x=.09,y=0.,z=0.),angular=NS(x=0.,y=0.,z=.22))))
    trace.observe(message,10.05,20.,dict(vx=.09,vy=0.,wz=.126),19.98)
    assert trace.last['yaw_rate_radps']==.22
    assert trace.last['bridge_wz']==.126
    message.twist.twist.angular.z=.03
    trace.observe(message,10.06,20.01,{},20.)
    assert trace.summary()['peak_estimated_yaw_rate_radps']==.22
    message.twist.twist.angular.z=float('nan')
    trace.observe(message,10.07,20.02,{},20.)
    json.dumps(trace.summary(),allow_nan=False)
    trace.close()
    rows=list(csv.DictReader((tmp_path/'csv/motion.csv').open()))
    assert float(rows[0]['wz'])==.22 and rows[2]['wz']=='nan'


def test_prepare_nav_uses_current_stage_and_records_requested_and_audit_limits(tmp_path,monkeypatch):
    from lite3_lab_5_2 import session
    import yaml
    run=tmp_path/'map_session';run.mkdir()
    nav=run/'official_upper_test';nav.mkdir()
    lab=tmp_path/'lab';(lab/'official_motion_bridge').mkdir(parents=True)
    pointer=lab/'official_motion_bridge/current_run.txt'
    monkeypatch.setattr(session,'LAB42',lab)
    monkeypatch.setattr(session,'current',lambda stage:(run,{'stage':stage}))
    for name,value in [('ROBOT_LENGTH','.779'),('ROBOT_WIDTH','.480'),('FOOTPRINT_PADDING','.10')]:
        monkeypatch.setenv(name,value)
    def generate(command,check):
        assert check and command[-4:]==['--max-forward-mps','0.10','--max-yaw-radps','0.10']
        (nav/'params.yaml').write_text(yaml.safe_dump({'controller_server':{'ros__parameters':{'FollowPath':{}}},
            'velocity_smoother':{'ros__parameters':{}}}))
        (nav/'setup.json').write_text('{}')
        pointer.write_text(str(nav))
    monkeypatch.setattr(session.subprocess,'run',generate)
    session.prepare_nav(2)
    setup=json.loads((nav/'setup.json').read_text())
    assert setup['max_forward_mps']==.1 and setup['official_bridge_cap_mps']==.2
    assert setup['audit_speed_limit_mps']==.2
    assert yaml.safe_load((nav/'params.yaml').read_text())['velocity_smoother']['ros__parameters']['scale_velocities']
