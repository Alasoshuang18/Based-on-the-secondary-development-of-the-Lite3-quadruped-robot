"""Real ROS construction/launch expansion on an isolated domain; no drivers."""
import importlib
import json
import os
from pathlib import Path
import sys
import pytest

rclpy=pytest.importorskip('rclpy')


@pytest.mark.parametrize('stage',[2,3,4])
def test_observer_constructs_without_motion_endpoints(tmp_path,stage,monkeypatch):
    import subprocess
    def forbidden(*args,**kwargs):
        raise AssertionError('Smoke tests must never start a process')
    monkeypatch.setattr(subprocess,'Popen',forbidden)
    names={2:'5-2_waypoint_patrol',3:'5-3_static_obstacle_avoidance',4:'5-4_dynamic_avoidance_recovery'}
    root=Path(__file__).resolve().parents[2]
    for name in names.values():
        sys.path.insert(0,str(root/name))
    cls=getattr(importlib.import_module(f'lite3_lab_5_{stage}.exploration'),
                {2:'ExplorationExperiment',3:'StaticExploration',4:'DynamicExploration'}[stage])
    params=root/names[stage]/'config/exploration.yaml'
    (tmp_path/'session.json').write_text(json.dumps({'stage':stage}))
    rclpy.init(args=['--ros-args','--params-file',str(params),'-p','result_dir:='+str(tmp_path/'exploration_test')],domain_id=96)
    node=None
    try:
        node=cls()
        assert not node.started
        assert node.p('robot_length_m')==.779
        assert node.p('max_linear_speed_mps')==.20
        assert 'amcl' not in node.lifecycle
        assert all(pub.topic_name not in ('/course/cmd_vel','/lab42/upper/vendor/cmd_vel') for pub in node.publishers)
        assert node.health()  # remains gated in an empty ROS domain
        node.measured_speed,node.measured_yaw=.10,.21
        assert any(reason.startswith('measured motion exceeds audit bounds') for reason in node.health())
        node.measured_yaw=.10
        assert not any(reason.startswith('measured motion exceeds audit bounds') for reason in node.health())
        assert node.process is None
        assert Path((tmp_path/'exploration_current.txt').read_text().strip())==tmp_path/'exploration_test'
        from rclpy.qos import ReliabilityPolicy
        odom=next(sub for sub in node.subscriptions if sub.topic_name=='/course/odom')
        assert odom.qos_profile.depth==1
        assert odom.qos_profile.reliability==ReliabilityPolicy.BEST_EFFORT
    finally:
        if node is not None:
            node.close_experiment()
            node.destroy_node()
        rclpy.try_shutdown()


def test_restore_launch_contains_posegraph_and_mapping_only(tmp_path,monkeypatch):
    from launch import LaunchContext
    from launch.actions import OpaqueFunction
    from lite3_lab_5_2 import stage_launch
    from lite3_lab_5_2.checkpoint import digest
    monkeypatch.setattr(stage_launch,'WS',tmp_path/'ws')
    monkeypatch.setattr(stage_launch,'configuration',lambda:{'length':.779})
    for key,value in dict(ROBOT_LENGTH='.779',ROBOT_WIDTH='.480',FOOTPRINT_PADDING='.10',
                          LIDAR_CONFIG='/unused/config.json',LIDAR_XYZ='[.210,0,.161308]',
                          LIDAR_RPY='[0,.338594,0]',BODY_HEIGHT='.325504965').items():
        monkeypatch.setenv(key,value)
    checkpoint=tmp_path/'parent'
    checkpoint.mkdir()
    files={}
    for name in ('map.yaml','map.pgm','map.pcd','slam.posegraph','slam.data'):
        (checkpoint/name).write_bytes(b'fixture')
        files[name]=digest(checkpoint/name)
    path=checkpoint/'checkpoint.json'
    path.write_text(json.dumps(dict(schema=1,complete=True,stage=2,
        configuration={'length':.779},files=files,saved_pose_map=[1.,2.,.3])))
    launch=stage_launch.generate_stage_launch(3)
    context=LaunchContext()
    context.launch_configurations.update(checkpoint=str(path),start_pose='[1.0,2.0,0.3]',
        pose_confirmed='true',result_dir=str(tmp_path/'new'),start_hardware='false')
    build=next(action for action in launch.entities if isinstance(action,OpaqueFunction))
    actions=build.execute(context)
    # Expanding the group sets global ROS parameters without executing the include.
    group=actions[0]
    entities=list(group.get_sub_entities())
    setters=[action for action in entities if type(action).__name__=='SetParameter']
    assert len(setters)==3
    for action in setters:
        action.execute(context)
    params=dict(context.launch_configurations['global_params'])
    assert params['map_file_name']==str(checkpoint/'slam')
    assert list(params['map_start_pose'])==[1.,2.,.3]
    assert params['map_start_at_dock'] is False
    session=json.loads((tmp_path/'new/session.json').read_text())
    assert session['map_to_odom_owner']=='slam_toolbox'


def test_restore_without_confirmed_pose_is_rejected(tmp_path,monkeypatch):
    from launch import LaunchContext
    from launch.actions import OpaqueFunction
    from lite3_lab_5_2 import stage_launch
    monkeypatch.setattr(stage_launch,'configuration',lambda:{})
    monkeypatch.setattr(stage_launch,'load_checkpoint',lambda *args: {})
    launch=stage_launch.generate_stage_launch(3)
    context=LaunchContext()
    context.launch_configurations.update(checkpoint='/parent',start_pose='[0.,0.,0.]',pose_confirmed='false')
    build=next(action for action in launch.entities if isinstance(action,OpaqueFunction))
    with pytest.raises(ValueError,match='Verify'):
        build.execute(context)
