"""One explicitly selected RViz destination while the existing SLAM keeps mapping."""
import math
import signal
import time
import threading
import rclpy
from geometry_msgs.msg import PointStamped,PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lite3_lab_5_1.geometry import yaw_from_quaternion
from lite3_lab_5_1.safety import fresh_source
from .exploration import ExplorationExperiment
from .manual_policy import destination_check


class ManualNavigation(ExplorationExperiment):
    def __init__(self):
        self._state_group=MutuallyExclusiveCallbackGroup()
        self._event_lock=threading.RLock()
        super().__init__('lite3_lab_5_2',{'manual_goal_timeout_s':600.,'max_aborted_goals':0})
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('max_aborted_goals',Parameter.Type.INTEGER,0)])
        self.costmap=None
        self.costmap_at=-math.inf
        self.selection=None
        self.precheck=None
        self.manual_handle=None
        self.result_future=None
        self.send_future=None
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.selected=self.create_publisher(PoseStamped,'/lab5/selected_goal',durable)
        self.create_subscription(PoseStamped,'/lab5/manual_goal',self.selected_pose,1)
        self.create_subscription(PointStamped,'/clicked_point',self.selected_point,1)
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.receive_costmap,durable)
        self.get_logger().info('手选目标模式：B放行后，在C用2D Goal Pose拖出目标与朝向；也支持Publish Point。无需调用start。')

    def create_subscription(self,msg_type,topic,callback,qos_profile,**kwargs):
        if topic in ('/course/odom','/lab42/upper/status'):
            kwargs['callback_group']=self._state_group
        return super().create_subscription(msg_type,topic,callback,qos_profile,**kwargs)

    def event(self,kind,**data):
        with self._event_lock:
            return super().event(kind,**data)

    def warmup_explorer(self):
        pass

    def health(self):
        return [reason for reason in super().health() if reason!='explorer warming up in paused mode']

    def receive_costmap(self,message):
        self.costmap,self.costmap_at=message,time.monotonic()

    def start_service(self,request,response):
        response.success=False
        response.message='手选模式不调用start；确认官方桥ready后，在RViz用2D Goal Pose或Publish Point提交新目标。'
        return response

    def selected_point(self,message):
        goal=PoseStamped()
        goal.header=message.header
        goal.pose.position=message.point
        try:
            yaw=self.estimate_xyyaw()[2]
        except Exception as exc:
            self.get_logger().warning('尚无定位TF，未提交目标：'+str(exc))
            return
        goal.pose.orientation.z=math.sin(yaw/2)
        goal.pose.orientation.w=math.cos(yaw/2)
        self.selected_pose(goal)

    def selected_pose(self,goal):
        if self.started or self.finished or self.stop_reason is not None:
            self.get_logger().warning('本F只接收一个目标；等待本点结束后重新启动F以记录下一点。')
            return
        try:
            if goal.header.frame_id!='map':
                raise ValueError('RViz Fixed Frame必须为map')
            position,quaternion=goal.pose.position,goal.pose.orientation
            if not all(math.isfinite(v) for v in (position.x,position.y,quaternion.x,quaternion.y,quaternion.z,quaternion.w)):
                raise ValueError('目标坐标或朝向不是有限数')
            if abs(sum(v*v for v in (quaternion.x,quaternion.y,quaternion.z,quaternion.w))-1.)>.01:
                raise ValueError('目标朝向四元数未归一化')
            yaw=yaw_from_quaternion(quaternion)
            goal.pose.position.z=0.
            goal.pose.orientation.x=goal.pose.orientation.y=0.
            goal.pose.orientation.z,goal.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
            self.selected.publish(goal)
            reasons=self.health()
            if reasons or self.healthy_since is None or time.monotonic()-self.healthy_since<.3:
                raise ValueError('未提交，也不排队；等健康恢复后重新点击：'+', '.join(reasons))
            grid=self.costmap
            stamp=None if grid is None else grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9
            if grid is None or grid.header.frame_id!='map' or not fresh_source(stamp,self.costmap_at,self.sim_now(),time.monotonic(),1.5,1.5):
                raise ValueError('全局代价地图缺失或过期，请等地图更新后重新选择')
            origin=grid.info.origin
            self.precheck=destination_check(grid.data,grid.info.width,grid.info.height,grid.info.resolution,
                [origin.position.x,origin.position.y,yaw_from_quaternion(origin.orientation)],
                [position.x,position.y,yaw],self.p('robot_length_m'),self.p('robot_width_m'),self.p('footprint_padding_m'))
            self.event('manual_destination_check',x=position.x,y=position.y,yaw=yaw,check=self.precheck)
            if not self.precheck['passed']:
                raise ValueError('目标或目标足迹不安全：'+str(self.precheck))
            goal.header.stamp=self.get_clock().now().to_msg()
            self.selection=dict(x=position.x,y=position.y,yaw=yaw,center_cost=self.precheck['center_cost'])
            self.exploration_started_ros,self.exploration_started_wall=self.sim_now(),time.monotonic()
            self.started=True
            self.task_states['manual_submission_pending']=1
            self.get_logger().info('提交手选目标：'+str(self.selection)+'；路径仍由Nav2进行碰撞检查。')
            request=NavigateToPose.Goal()
            request.pose=goal
            self.send_future=self.nav_client.send_goal_async(request)
            self.send_future.add_done_callback(self.goal_accepted_manual)
        except Exception as exc:
            self.get_logger().warning(str(exc))
            self.event('manual_goal_not_submitted',reason=str(exc))
            if self.started:
                self.task_states.pop('manual_submission_pending',None)
                self.cancel_navigation('manual_goal_submission_failed')

    def goal_accepted_manual(self,future):
        try:
            self.manual_handle=future.result()
            self.task_states.pop('manual_submission_pending',None)
            if not self.manual_handle or not self.manual_handle.accepted:
                self.cancel_navigation('manual_goal_rejected')
                return
            key=bytes(self.manual_handle.goal_id.uuid).hex()
            self.task_states.setdefault(key,1)
            self.result_future=self.manual_handle.get_result_async()
            self.result_future.add_done_callback(self.goal_result_manual)
            if self.stop_reason is not None:
                self.manual_handle.cancel_goal_async()
        except Exception as exc:
            self.task_states.pop('manual_submission_pending',None)
            self.cancel_navigation('manual_goal_acceptance_error: '+str(exc))

    def goal_result_manual(self,future):
        try:
            status=future.result().status
            key=bytes(self.manual_handle.goal_id.uuid).hex()
            self.task_states[key]=status
            if status==4:
                self.reached_goals.add(key)
            elif status==6:
                self.aborted_goals.add(key)
            self.event('manual_goal_result',status=status,goal=self.selection)
            self.cancel_navigation({4:'manual_goal_succeeded',5:'manual_goal_canceled',6:'manual_goal_aborted'}.get(status,'manual_goal_unknown_result'))
        except Exception as exc:
            self.cancel_navigation('manual_goal_result_error: '+str(exc))

    def tick(self):
        if self.finished or self.stop_reason is not None:
            self.maintain_stop()
            return
        self.poll_lifecycle()
        reasons=self.health()
        now=time.monotonic()
        self.healthy_since=(self.healthy_since or now) if not reasons else None
        if not self.started:
            if now-self.last_health_log>=2.:
                self.last_health_log=now
                self.event('manual_navigation_health',waiting_for=reasons)
                self.get_logger().info('Waiting: '+(', '.join(reasons) if reasons else '在RViz选择目标，不调用start'))
            return
        if reasons:
            self.event('runtime_health_lost',waiting_for=reasons,
                odom_source_age_s=None if self.odom_stamp is None else self.sim_now()-self.odom_stamp,
                odom_receive_age_s=None if self.odom_wall is None else now-self.odom_wall,
                odom_age_at_receive_s=getattr(self,'odom_age_at_receive',None),
                measured_motion=self.motion.last,
                measured_speed_limit_mps=self.p('max_linear_speed_mps'),
                measured_yaw_limit_radps=self.p('max_yaw_rate_rps'),
                upper_status=self.upper)
            self.cancel_navigation('runtime_health_lost: '+','.join(reasons))
        elif now-self.exploration_started_wall>self.p('manual_goal_timeout_s'):
            self.cancel_navigation('manual_goal_timeout')

    def summary_extra(self):
        return dict(goal_selection='rviz_manual',selected_goal=self.selection,destination_check=self.precheck,
                    online_mapping=True,automatic_frontier_exploration=False)

    def close_experiment(self):
        # Also hand back an enabled bridge when exiting before the first click.
        if rclpy.ok() and not self.finished:
            self.cancel_navigation('interrupted')
            deadline=time.monotonic()+11.
            while not self.finished and rclpy.ok() and time.monotonic()<deadline:
                rclpy.spin_once(self,timeout_sec=.05)
                self.maintain_stop()
        super().close_experiment()


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    stopping=[False]
    previous={n:signal.signal(n,lambda *_:stopping.__setitem__(0,True)) for n in (signal.SIGINT,signal.SIGTERM)}
    node=None
    executor=None
    try:
        node=ManualNavigation()
        executor=MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        while rclpy.ok() and not stopping[0] and not node.finished:
            executor.spin_once(timeout_sec=.05)
    finally:
        if executor:
            executor.shutdown()
            if node:
                executor.remove_node(node)
        if node:
            node.close_experiment()
            node.destroy_node()
        for n,handler in previous.items():
            signal.signal(n,handler)
        rclpy.try_shutdown()
