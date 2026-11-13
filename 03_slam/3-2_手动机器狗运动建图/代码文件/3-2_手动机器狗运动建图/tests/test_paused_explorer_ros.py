"""Integration fixture: actual C++ explorer, synthetic map, fake Nav2; no robot."""
import os
import signal
import subprocess
import time
from pathlib import Path
import pytest

rclpy=pytest.importorskip('rclpy')


@pytest.mark.parametrize('abort_first',[False,True])
def test_paused_explorer_sends_no_goal_until_resume(tmp_path,abort_first):
    from ament_index_python.packages import get_package_prefix
    from explore_lite_msgs.msg import ExploreStatus
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from std_msgs.msg import Bool
    from tf2_ros import StaticTransformBroadcaster
    executable=Path(get_package_prefix('explore_lite'))/'lib/explore_lite/explore'
    assert (executable.parent/'paused_start.json').is_file(), 'Build and verify the paused-start explorer first'
    rclpy.init(domain_id=97)
    node=rclpy.create_node('paused_explorer_fixture')
    goals=[]
    async def execute(handle):
        goals.append(handle.request)
        if abort_first and len(goals)==1:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()
    server=ActionServer(node,NavigateToPose,'/navigate_to_pose',execute_callback=execute)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    maps=node.create_publisher(OccupancyGrid,'/fixture_map',qos)
    resume=node.create_publisher(Bool,'/explore/resume',10)
    statuses=[]
    status=node.create_subscription(ExploreStatus,'/explore/status',lambda msg:statuses.append(msg.status),qos)
    transform=TransformStamped()
    transform.header.frame_id='map'
    transform.child_frame_id='base_footprint'
    transform.header.stamp=node.get_clock().now().to_msg()
    transform.transform.translation.x=1.5
    transform.transform.translation.y=1.5
    transform.transform.rotation.w=1.
    broadcaster=StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transform)
    grid=OccupancyGrid()
    grid.header.frame_id='map'
    grid.info.width=60
    grid.info.height=60
    grid.info.resolution=.05
    grid.info.origin.orientation.w=1.
    # Two distinct frontier windows separated by known occupied border cells.
    grid.data=[0 if 10<=x<50 and 10<=y<50 else
               -1 if (x<10 and 12<=y<36) or (x>=50 and 18<=y<42) else 100
               for y in range(60) for x in range(60)]
    def pump_until(predicate,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            grid.header.stamp=node.get_clock().now().to_msg()
            maps.publish(grid)
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():
                return True
        return False
    log=(tmp_path/'explorer.log').open('w')
    process=subprocess.Popen([str(executable),'--ros-args','-p','start_paused:=true',
        '-p','robot_base_frame:=base_footprint','-p','costmap_topic:=/fixture_map',
        '-p','return_to_init:=false','-p','planner_frequency:=0.15','-p','min_frontier_size:=0.75'],
        env=dict(os.environ,ROS_DOMAIN_ID='97',ROS_LOCALHOST_ONLY='1'),stdout=log,
        stderr=subprocess.STDOUT,start_new_session=True)
    try:
        assert pump_until(lambda:'exploration_paused' in statuses,10), (tmp_path/'explorer.log').read_text()
        pump_until(lambda:False,.6)
        assert goals==[]
        resume.publish(Bool(data=True))
        wanted=2 if abort_first else 1
        assert pump_until(lambda:len(goals)>=wanted,10), (tmp_path/'explorer.log').read_text()
        assert goals[0].pose.header.frame_id=='map'
        if abort_first:
            import math
            positions=[(goal.pose.pose.position.x,goal.pose.pose.position.y) for goal in goals[:2]]
            assert math.dist(*positions)>.25, 'An aborted frontier must be blacklisted rather than repeated'
        resume.publish(Bool(data=False))
        pump_until(lambda:statuses.count('exploration_paused')>=2,3)
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait(timeout=3)
        log.close()
        server.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
