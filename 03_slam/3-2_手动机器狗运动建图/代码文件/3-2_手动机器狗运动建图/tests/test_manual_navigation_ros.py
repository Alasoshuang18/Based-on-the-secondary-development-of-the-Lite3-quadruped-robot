"""Real ROS manual client with a fake Nav2 server; isolated from robot domain."""
import json
import time
import threading
from pathlib import Path
import pytest
rclpy=pytest.importorskip('rclpy')


@pytest.fixture
def manual_fixture(tmp_path):
    from rclpy.node import Node
    from rclpy.action import ActionServer
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid,Odometry
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from lite3_lab_5_2.manual_navigation import ManualNavigation
    (tmp_path/'session.json').write_text(json.dumps({'stage':2}))
    rclpy.init(domain_id=98,args=['--ros-args','-p','result_dir:='+str(tmp_path/'exploration_manual'),
                                 '-p','enable_real_motion:=true','-p','site_configuration_verified:=true'])
    client=ManualNavigation()
    client.health=lambda:[]
    client.estimate_xyyaw=lambda *a:(1.,1.,0.)
    fixture=Node('manual_navigation_fixture')
    goals=[]
    mode=['ENABLED']
    async def execute(handle):
        goals.append(handle.request)
        handle.succeed()
        return NavigateToPose.Result()
    server=ActionServer(fixture,NavigateToPose,'/navigate_to_pose',execute_callback=execute)
    def stop(request,response):
        mode[0]='DISABLED'
        response.success=True
        return response
    stop_service=fixture.create_service(Trigger,'/lab42/upper/stop',stop)
    upper=fixture.create_publisher(String,'/lab42/upper/status',1)
    odoms=fixture.create_publisher(Odometry,'/course/odom',1)
    clicks=fixture.create_publisher(PoseStamped,'/lab5/manual_goal',1)
    grid=OccupancyGrid()
    grid.header.frame_id='map'
    grid.info.width=80;grid.info.height=80;grid.info.resolution=.05
    grid.info.origin.orientation.w=1.
    grid.data=[0]*6400
    def publish():
        now=fixture.get_clock().now().to_msg()
        grid.header.stamp=now
        client.receive_costmap(grid)
        message=Odometry()
        message.header.stamp=now;message.header.frame_id='odom';message.child_frame_id='base_link'
        message.pose.pose.orientation.w=1.
        odoms.publish(message)
        upper.publish(String(data=json.dumps(dict(mode=mode[0],ready=True,execute=True,vx=0.,vy=0.,wz=0.,max_forward_mps=.2,max_yaw_radps=.2))))
    timer=fixture.create_timer(.02,publish)
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(client);executor.add_node(fixture)
    def pump(predicate,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.02)
            if predicate():return True
        return False
    def click():
        goal=PoseStamped()
        goal.header.frame_id='map';goal.header.stamp=fixture.get_clock().now().to_msg()
        goal.pose.position.x=2.;goal.pose.position.y=2.;goal.pose.orientation.w=1.
        clicks.publish(goal)
    ctx=dict(client=client,fixture=fixture,goals=goals,grid=grid,executor=executor,pump=pump,click=click,tmp_path=tmp_path)
    try:
        pump(lambda:client.odom_stamp is not None and client.upper,2)
        yield ctx
    finally:
        if not client.finished:
            client.cancel_navigation('fixture_cleanup')
            pump(lambda:client.finished,3)
        executor.shutdown()
        executor.remove_node(client);executor.remove_node(fixture)
        client.close_experiment();client.destroy_node()
        server.destroy();fixture.destroy_node()
        rclpy.try_shutdown()


def test_manual_client_never_explores_and_only_submits_a_fresh_safe_click(manual_fixture):
    c=manual_fixture
    node=c['client']
    c['pump'](lambda:False,.4)
    assert c['goals']==[] and node.process is None
    node.health=lambda:['odom missing or stale']
    c['click']();c['pump'](lambda:False,.25)
    assert c['goals']==[] and not node.started
    node.health=lambda:[]
    c['grid'].data[40*80+40]=100
    c['pump'](lambda:False,.4)
    c['click']();c['pump'](lambda:False,.25)
    assert c['goals']==[] and not node.started
    c['grid'].data[40*80+40]=0
    c['pump'](lambda:False,.4)
    c['click']()
    assert c['pump'](lambda:node.finished,8)
    assert len(c['goals'])==1
    assert c['goals'][0].pose.pose.position.x==2.
    summary=json.loads((node.result_dir/'summary.json').read_text())
    assert summary['goal_selection']=='rviz_manual'
    assert summary['navigation_success']


def test_odom_state_callback_keeps_advancing_when_main_callback_waits(manual_fixture):
    c=manual_fixture
    node=c['client']
    entered=threading.Event();release=threading.Event()
    blocker=[]
    def block():
        blocker[0].cancel()
        entered.set()
        release.wait(timeout=2)
    blocker.append(node.create_timer(.03,block))
    try:
        assert c['pump'](entered.is_set,2)
        before=node.odom_stamp
        c['pump'](lambda:False,.3)
        assert node.odom_stamp>before+.05
        assert not release.is_set()
        assert node.process is None and c['goals']==[]
    finally:
        release.set()
        c['pump'](lambda:False,.1)
