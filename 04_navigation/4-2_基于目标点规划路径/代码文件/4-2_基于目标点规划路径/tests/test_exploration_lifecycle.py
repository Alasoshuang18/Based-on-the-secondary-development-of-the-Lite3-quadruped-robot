"""Run real lifecycle methods with controlled clocks; never starts robot nodes."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS


def method(name, namespace):
    path=Path(__file__).resolve().parents[1]/'lite3_lab_5_2/exploration.py'
    tree=ast.parse(path.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    module=ast.Module(body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name],type_ignores=[])
    exec(compile(ast.fix_missing_locations(module),str(path),'exec'),namespace)
    return namespace[name]


def test_historical_goal_success_cannot_count_as_this_exploration():
    tasks=method('tasks',{})
    node=NS(started=True,preexisting_tasks={'01'},task_states={},reached_goals=set(),
            exploration_started_ros=100.,stop_reason=None,event=lambda *a,**k:None,
            cancel_navigation=lambda reason:None,get_logger=lambda:NS(info=lambda *args:None))
    def item(key,stamp,status):
        return NS(goal_info=NS(goal_id=NS(uuid=bytes([key])),stamp=NS(sec=stamp,nanosec=0)),status=status)
    tasks(node,NS(status_list=[item(1,101,4),item(2,99,4),item(3,101,4)]))
    assert node.reached_goals=={'03'}


def test_late_executing_status_cannot_regress_an_action_result():
    tasks=method('tasks',{})
    events=[]
    node=NS(started=True,preexisting_tasks=set(),task_states={'01':4},reached_goals={'01'},
        exploration_started_ros=100.,event=lambda *a,**k:events.append((a,k)))
    goal=NS(goal_id=NS(uuid=b'\x01'),stamp=NS(sec=101,nanosec=0))
    tasks(node,NS(status_list=[NS(goal_info=goal,status=2)]))
    assert node.task_states['01']==4
    assert events[0][0][0]=='older_nonterminal_status_ignored'


def test_stop_acceptance_without_terminal_action_cannot_confirm_stopped(tmp_path):
    now=[100.]
    namespace=dict(time=NS(monotonic=lambda:now[0]),json=__import__('json'),
                   math=__import__('math'),fresh_source=lambda *args:True)
    stop=method('maintain_stop',namespace)
    node=NS(stop_reason='user_cancel',stop_at=99.,stop_future=None,cancel_future=None,
            stop_ack=True,cancel_ack=True,last_stop_request=100.,process=None,
            upper_at=100.,upper={'mode':'DISABLED','vx':0.,'vy':0.,'wz':0.},
            odom_stamp=1.,odom_wall=100.,sim_now=lambda:1.,measured_speed=0.,measured_yaw=0.,
            standstill=NS(report=lambda now:dict(stationary=True)),stationary_since=99.,task_states={'goal':2},result_dir=tmp_path,
            finished=False,reached_goals={'older'},finish=lambda *a,**k:None)
    stop(node)
    assert not __import__('json').loads((tmp_path/'safety_status.json').read_text())['stop_confirmed']
    node.task_states['goal']=5
    stop(node)
    assert __import__('json').loads((tmp_path/'safety_status.json').read_text())['stop_confirmed']


def test_cancel_reply_requires_terminal_for_previously_unobserved_goal(tmp_path):
    clock=NS(monotonic=lambda:100.)
    namespace=dict(time=clock,json=__import__('json'),math=__import__('math'),fresh_source=lambda *args:True)
    stop=method('maintain_stop',namespace)
    response=NS(return_code=0,goals_canceling=[NS(goal_id=NS(uuid=b'\x02'))])
    node=NS(stop_reason='user_cancel',stop_at=99.,stop_future=None,
            cancel_future=NS(done=lambda:True,result=lambda:response),
            stop_ack=True,cancel_ack=False,last_stop_request=100.,process=None,
            upper_at=100.,upper={'mode':'DISABLED','vx':0.,'vy':0.,'wz':0.},
            odom_stamp=1.,odom_wall=100.,sim_now=lambda:1.,measured_speed=0.,measured_yaw=0.,
            standstill=NS(report=lambda now:dict(stationary=True)),stationary_since=99.,task_states={},result_dir=tmp_path,finished=False,
            reached_goals=set(),finish=lambda *a,**k:None)
    stop(node)
    assert node.task_states=={'02':3}
    assert not __import__('json').loads((tmp_path/'safety_status.json').read_text())['stop_confirmed']


def test_start_while_warming_up_does_not_queue_future_motion():
    start=method('start_service',{'time':NS(monotonic=lambda:100.)})
    node=NS(finished=False,stop_reason=None,started=False,start_requested=False,
            healthy_since=None,health=lambda:['explorer warming up in paused mode'])
    response=start(node,None,NS())
    assert not response.success
    assert not node.start_requested


def test_start_requires_continuous_health_then_accepts_explicit_request():
    start=method('start_service',{'time':NS(monotonic=lambda:100.)})
    node=NS(finished=False,stop_reason=None,started=False,start_requested=False,
            healthy_since=99.9,health=lambda:[])
    assert not start(node,None,NS()).success
    assert not node.start_requested
    node.healthy_since=99.5
    assert start(node,None,NS()).success
    assert node.start_requested


def test_aborted_frontier_is_retained_and_retry_is_bounded():
    tasks=method('tasks',{})
    cancelled=[]
    node=NS(started=True,preexisting_tasks=set(),task_states={},reached_goals=set(),aborted_goals=set(),
            exploration_started_ros=100.,stop_reason=None,event=lambda *a,**k:None,
            cancel_navigation=cancelled.append,p=lambda key:3,
            get_logger=lambda:NS(info=lambda *args:None,warning=lambda *args:None))
    def fail(key):
        goal=NS(goal_id=NS(uuid=bytes([key])),stamp=NS(sec=101,nanosec=0))
        tasks(node,NS(status_list=[NS(goal_info=goal,status=6)]))
    fail(1); fail(1)
    assert len(node.aborted_goals)==1 and not cancelled
    fail(2)
    assert len(node.aborted_goals)==2 and not cancelled
    fail(3)
    assert cancelled==['nav2_aborted_goal_limit_reached']


def test_dynamic_strict_policy_stops_on_first_aborted_goal():
    tasks=method('tasks',{})
    cancelled=[]
    node=NS(started=True,preexisting_tasks=set(),task_states={},reached_goals=set(),aborted_goals=set(),
            exploration_started_ros=100.,stop_reason=None,event=lambda *a,**k:None,
            cancel_navigation=cancelled.append,p=lambda key:0,
            get_logger=lambda:NS(info=lambda *args:None,warning=lambda *args:None))
    goal=NS(goal_id=NS(uuid=b'\x01'),stamp=NS(sec=101,nanosec=0))
    tasks(node,NS(status_list=[NS(goal_info=goal,status=6)]))
    assert cancelled==['nav2_aborted_goal_limit_reached']


def test_cancel_after_finished_does_not_reuse_old_stop_acknowledgement():
    cancel=method('cancel_service',{'math':__import__('math')})
    calls=[]
    node=NS(stop_ack=True,cancel_ack=True,last_stop_request=100.,cancel_navigation=calls.append)
    result=cancel(node,None,NS())
    assert result.success and calls==['user_cancel']
    assert not node.stop_ack and not node.cancel_ack
    assert node.last_stop_request==-float('inf')


def test_ended_session_reasserts_stop_when_bridge_is_reenabled(tmp_path):
    namespace=dict(time=NS(monotonic=lambda:100.),json=__import__('json'),
                   math=__import__('math'),fresh_source=lambda *args:True,Trigger=NS(Request=lambda:object()))
    stop=method('maintain_stop',namespace)
    requests=[]
    def request(value):
        requests.append(value)
        return NS(done=lambda:False)
    node=NS(stop_reason='nav2_aborted_goal_limit_reached',stop_at=90.,stop_future=None,cancel_future=None,
            stop_ack=True,cancel_ack=True,last_stop_request=98.,process=None,
            upper_at=100.,upper={'mode':'ENABLED','vx':0.,'vy':0.,'wz':0.},
            official_stop=NS(service_is_ready=lambda:True,call_async=request),
            odom_stamp=1.,odom_wall=100.,sim_now=lambda:1.,measured_speed=0.,measured_yaw=0.,
            standstill=NS(report=lambda now:dict(stationary=True)),stationary_since=99.,task_states={},result_dir=tmp_path,finished=True,
            reached_goals=set(),finish=lambda *a,**k:None)
    stop(node)
    assert len(requests)==1 and not node.stop_ack
    assert not __import__('json').loads((tmp_path/'safety_status.json').read_text())['stop_confirmed']


def test_one_arrival_does_not_hide_another_canceled_goal_in_summary(tmp_path):
    import json
    finish=method('finish',{'json':json})
    node=NS(stop_reason='budget_finished',finished=False,navigation_success=False,
            package_name='lite3_lab_5_2',p=lambda key:'real',reached_goals={'arrived'},
            aborted_goals=set(),task_states={'arrived':4,'canceled':5},frontiers_exhausted=False,
            summary_extra=lambda:{},motion=NS(summary=lambda:{}),result_dir=tmp_path,event=lambda *a,**k:None,
            get_logger=lambda:NS(info=lambda *a:None))
    finish(node,'budget_finished',successful=True)
    summary=json.loads((tmp_path/'summary.json').read_text())
    assert summary['reached_goals']==1
    assert not summary['all_observed_goals_succeeded']
