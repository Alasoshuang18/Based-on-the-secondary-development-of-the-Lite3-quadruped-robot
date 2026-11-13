import math
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_4_1.core import StreamMonitor, loop_metrics, summarize_trials, validate_pcd


def healthy():
    m=StreamMonitor()
    for i in range(100):
        t=10+i*.01
        assert m.observe('imu',t,t,wall=t)
        m.imu((0,0,9.81))
    for name in ('cloud','odom','map'):
        m.observe(name,10.99,10.99,wall=10.99)
    return m


def test_four_stream_readiness_and_source_and_receive_timeouts():
    m=healthy()
    assert m.health(10.99,wall=10.99)['ready']
    assert not m.health(13,wall=13)['ready']
    assert not m.health(10.99,wall=13)['ready']
    assert not m.health(5,wall=11)['ready']


def test_just_received_old_messages_are_still_stale():
    m=healthy()
    assert not m.observe('odom',11,20,wall=20)
    assert 'odom' in m.health(20,wall=20)['stale_streams']


@pytest.mark.parametrize('stamp',[10.99,10.9,0,float('nan'),float('inf'),50])
def test_duplicate_backwards_invalid_future_stamps_do_not_refresh(stamp):
    m=healthy()
    assert not m.observe('odom',stamp,11,wall=11)
    assert 'odom' in m.health(11,wall=11)['stale_streams']
    assert m.records['odom']['stamp']==10.99


@pytest.mark.parametrize('norm',[1.0,96.0])
def test_wrong_imu_units_cannot_initialize(norm):
    m=healthy()
    for _ in range(100):m.imu((0,0,norm))
    assert not m.health(10.99,wall=10.99)['initialized']


def test_nonfinite_imu_clears_initialization():
    m=healthy()
    assert not m.imu((0,0,float('nan')))
    assert not m.health(10.99,wall=10.99)['initialized']


def test_valid_message_recovers_from_bad_stamp_without_resetting_clock():
    m=healthy()
    m.observe('odom',50,11,wall=11)
    assert m.observe('odom',11.01,11.02,wall=11.02)
    assert m.health(11.02,wall=11.02)['ready']


def test_closed_route_and_yaw_wrapping():
    r=loop_metrics([(1,0,0,math.pi-.01),(1.1,1,0,0),(1.2,0,0,-math.pi+.01)])
    assert r['path_length_m']==2
    assert r['endpoint_separation_m']==0
    assert r['endpoint_yaw_difference_rad']==pytest.approx(.02)
    with pytest.raises(ValueError):loop_metrics([(1,0,0,0),(1,0,0,0)])


def test_summary_retains_failures_and_requires_exactly_three():
    rows=[dict(success=True,path_length_m=2),dict(success=False,path_length_m=4),dict(success=True,path_length_m=3)]
    r=summarize_trials(rows)
    assert not r['all_pass'] and r['success_count']==2
    assert r['median']['path_length_m']==3 and len(r['raw'])==3
    assert not summarize_trials([dict(success=True)])['all_pass']


def pcd(tmp_path, body, declared=2, data='ascii'):
    p=tmp_path/'test.pcd'
    p.write_text('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS '+str(declared)+'\nDATA '+data+'\n'+body)
    return p


def test_pcd_actually_reloads_all_xyz(tmp_path):
    r=validate_pcd(pcd(tmp_path,'0 1 2\n3 4 5\n'))
    assert r['reload_valid'] and r['point_count']==2


@pytest.mark.parametrize('body,declared,data',[
    ('0 1 2\n',2,'ascii'),('0 nan 2\n3 4 5\n',2,'ascii'),
    ('0 1\n3 4 5\n',2,'ascii'),('',0,'ascii'),('0 1 2\n',1,'binary')])
def test_bad_pcd_cannot_report_success(tmp_path,body,declared,data):
    with pytest.raises(ValueError):validate_pcd(pcd(tmp_path,body,declared,data))
