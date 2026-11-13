"""Stage session utilities: checkpoints and a separate X11 RViz entry."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import yaml
from .checkpoint import digest, load_checkpoint, current_audit
from .stage_launch import WS, LAB42


def current(stage):
    run = Path((WS/'results'/f'5_{stage}_current_run.txt').read_text().strip()).resolve(strict=True)
    data = json.loads((run/'session.json').read_text())
    if data['stage'] != stage:
        raise ValueError('Wrong stage session')
    return run, data


def prepare_nav(stage):
    from ament_index_python.packages import get_package_prefix
    from .navigation_profile import with_tracking_headroom
    run,_=current(stage)
    executable=Path(get_package_prefix('lab42_official_motion'))/'lib/lab42_official_motion/prepare_nav.py'
    subprocess.run([str(executable),'--length',os.environ['ROBOT_LENGTH'],
        '--width',os.environ['ROBOT_WIDTH'],'--padding',os.environ['FOOTPRINT_PADDING'],
        '--max-forward-mps','0.10','--max-yaw-radps','0.10'],check=True)
    nav=Path((LAB42/'official_motion_bridge/current_run.txt').read_text().strip()).resolve(strict=True)
    if nav.parent!=run:
        raise RuntimeError('Generated navigation configuration belongs to another A session')
    path=nav/'params.yaml'
    config=with_tracking_headroom(yaml.safe_load(path.read_text()))
    temporary=path.with_suffix('.tmp')
    temporary.write_text(yaml.safe_dump(config,sort_keys=False))
    temporary.replace(path)
    path=nav/'setup.json'
    setup=json.loads(path.read_text())
    setup.update(max_forward_mps=.10,max_yaw_radps=.10,
        official_bridge_cap_mps=.20,official_bridge_yaw_cap_radps=.20,
        audit_speed_limit_mps=.20,audit_yaw_limit_radps=.20,
        speed_profile='tracking_headroom_010',scale_velocities=True)
    path.write_text(json.dumps(setup,indent=2))
    print('本阶段Nav2请求上限0.10 m/s、0.10 rad/s；官方桥和实测审核上限仍为0.20。')
    print('NAV_RUN='+str(nav))


def rviz(stage=2):
    if not os.environ.get('DISPLAY'):
        raise RuntimeError('No DISPLAY: reconnect with MobaXterm X11 forwarding; never set DISPLAY=:0')
    os.environ.setdefault('QT_X11_NO_MITSHM','1')
    # GLX travels through the authenticated SSH X11 channel. Software rendering
    # is opt-in because support depends on the local MobaXterm X server.
    profile = LAB42/'nav2_autonomy_tools/lab4_2_navigation_light.rviz'
    if stage==2:
        from .manual_view import configure_manual_view
        directory=Path(tempfile.mkdtemp(prefix='5_2_manual_rviz_',dir=WS/'results'))
        generated=directory/'manual_navigation.rviz'
        generated.write_text(yaml.safe_dump(configure_manual_view(yaml.safe_load(profile.read_text())),
                                           sort_keys=False,allow_unicode=True))
        profile=generated
    subprocess.run(['/usr/bin/python3','-B',str(LAB42/'fastlio_latency_fix/prepare_navigation_view.py'),str(profile)], check=True)
    os.execvp('nice', ['nice','-n','10','rviz2','-d',str(profile)])


def save(stage):
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from nav_msgs.msg import Odometry
    from rcl_interfaces.srv import GetParameters
    from rclpy.time import Time
    from slam_toolbox.srv import SerializePoseGraph
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener
    run, session = current(stage)
    from .standstill import PoseStandstill
    standstill=PoseStandstill()
    rclpy.init()
    node = rclpy.create_node('exploration_checkpoint')
    state = dict(upper=None, upper_at=-math.inf, active=True, actions_at=-math.inf,
                 speed=math.inf, yaw=math.inf, odom_at=-math.inf, odom_stamp=-math.inf)
    def upper(msg):
        state.update(upper=json.loads(msg.data), upper_at=time.monotonic())
    def actions(msg):
        state.update(active=any(item.status in (1,2,3) for item in msg.status_list), actions_at=time.monotonic())
    def odom(msg):
        standstill.observe_message(msg)
        state.update(speed=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y),
                     yaw=abs(msg.twist.twist.angular.z), odom_at=time.monotonic(),
                     odom_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    node.create_subscription(String,'/lab42/upper/status',upper,1)
    node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',actions,10)
    from rclpy.qos import qos_profile_sensor_data
    node.create_subscription(Odometry,'/course/odom',odom,qos_profile_sensor_data)
    buffer = Buffer()
    listener = TransformListener(buffer,node)
    def call(service_type, name, request, seconds=10):
        client = node.create_client(service_type,name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Service unavailable: '+name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
            if not future.done() or future.result() is None:
                raise RuntimeError('Service timeout: '+name)
            return future.result()
        finally:
            node.destroy_client(client)
    def stopped():
        now = time.monotonic()
        ros = node.get_clock().now().nanoseconds*1e-9
        upper = state['upper'] or {}
        return (not state['active'] and now-state['upper_at'] <= .5 and upper.get('mode')=='DISABLED'
                and all(abs(upper.get(k,math.inf)) < 1e-9 for k in ('vx','vy','wz'))
                and now-state['odom_at'] <= 1. and -.05 <= ros-state['odom_stamp'] <= 1.
                and standstill.report(ros)['stationary'])
    snapshot = None
    try:
        # The explorer writes this after cancellation and process exit. Avoid
        # relying on the action status topic, which may publish only on change.
        evidence_dir = current_audit(run)
        print('EXPLORATION_EVIDENCE='+str(evidence_dir), flush=True)
        audit = json.loads((evidence_dir/'safety_status.json').read_text())
        if not audit.get('stop_confirmed') or not audit.get('explorer_exited'):
            raise RuntimeError('Stop exploration and obtain confirmed action/bridge/odometry stop first')
        if any(name in node.get_node_names() for name in
               ('explore','explore_node','lite3_lab_5_2_explorer','lite3_lab_5_3_explorer','lite3_lab_5_4_explorer')):
            raise RuntimeError('Explorer still running')
        state['active'] = bool(audit['active_actions'])
        response = call(CancelGoal,'/navigate_to_pose/_action/cancel_goal',CancelGoal.Request())
        if response.return_code != 0:
            raise RuntimeError('Nav2 cancellation unavailable; checkpoint refused')
        response = call(Trigger,'/lab42/upper/stop',Trigger.Request())
        if not response.success:
            raise RuntimeError('Official stop refused; checkpoint refused')
        stable, until = None, time.monotonic()+6
        while time.monotonic() < until:
            rclpy.spin_once(node,timeout_sec=.05)
            stable = (stable or time.monotonic()) if stopped() else None
            if stable and time.monotonic()-stable >= .5:
                break
        if not stable or time.monotonic()-stable < .5:
            raise RuntimeError('Bridge must be DISABLED with fresh stationary odometry')
        result = call(GetParameters,'/lab_4_2/get_parameters',GetParameters.Request(names=['result_dir']))
        if Path(result.values[0].string_value).resolve() != run:
            raise RuntimeError('Mapping process belongs to another session')
        result = call(Trigger,'/course/save_map',Trigger.Request(),30)
        if not result.success:
            raise RuntimeError('Map save failed: '+result.message)
        snapshot = Path(tempfile.mkdtemp(prefix='checkpoint_',dir=run))
        source = run/'maps/robot_dog_auto_map.yaml'
        meta = yaml.safe_load(source.read_text())
        if abs(float(meta['resolution'])-.05)>1e-6:
            raise ValueError('Map resolution differs from 5-1')
        image = source.parent/meta['image']
        shutil.copy2(image,snapshot/'map.pgm')
        shutil.copy2(source.with_suffix('.pcd'),snapshot/'map.pcd')
        meta['image']='map.pgm'
        (snapshot/'map.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
        result = call(SerializePoseGraph,'/slam_toolbox/serialize_map',
                      SerializePoseGraph.Request(filename=str(snapshot/'slam')),30)
        if result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            raise RuntimeError('SLAM posegraph serialization failed')
        rclpy.spin_once(node,timeout_sec=.05)
        if not stopped():
            raise RuntimeError('Robot/state changed during checkpoint; do not use partial checkpoint')
        tf = buffer.lookup_transform('map','base_footprint',Time())
        stamp = tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
        if not -.55 <= node.get_clock().now().nanoseconds*1e-9-stamp <= 1.:
            raise RuntimeError('Saved pose TF is stale')
        q = tf.transform.rotation
        pose = [tf.transform.translation.x,tf.transform.translation.y,
                math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        files = {name:digest(snapshot/name) for name in
                 ('map.yaml','map.pgm','map.pcd','slam.posegraph','slam.data')}
        manifest = dict(schema=1,complete=True,stage=stage,configuration=session['configuration'],
                        parent_checkpoint=session['parent_checkpoint'],saved_pose_map=pose,
                        files=files,stationary_snapshot=True,
                        note='Sequential stationary map and posegraph saves; not an atomic sensor-time snapshot')
        path = snapshot/'checkpoint.json'
        path.write_text(json.dumps(manifest,indent=2))
        load_checkpoint(path)
        (run/'latest_checkpoint.txt').write_text(str(path)+'\n')
        print('CHECKPOINT='+str(path),flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('command',choices=['save','rviz','current','audit','verify','prepare-nav'])
    parser.add_argument('--stage',type=int,choices=[2,3,4],default=2)
    parser.add_argument('--checkpoint')
    args=parser.parse_args()
    if args.command=='prepare-nav':
        prepare_nav(args.stage)
    elif args.command=='save':
        save(args.stage)
    elif args.command=='rviz':
        rviz(args.stage)
    elif args.command=='current':
        print(current(args.stage)[0])
    elif args.command=='audit':
        print(current_audit(current(args.stage)[0]))
    else:
        print(json.dumps(load_checkpoint(args.checkpoint),indent=2))


if __name__=='__main__':
    main()
