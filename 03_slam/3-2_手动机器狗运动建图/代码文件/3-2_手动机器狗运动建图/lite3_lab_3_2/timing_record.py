"""Bounded per-second timing evidence; original message stamps stay untouched."""
import time
import os
import json


class TimingRecord:
    def __init__(self,node,directory):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        self.node=node
        self.state={}
        self.previous_tick=time.monotonic()
        self.file=(directory/'timing.jsonl').open('a')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions=[node.create_subscription(typ,topic,lambda msg,t=topic:self.observe(t,msg),qos)
            for topic,typ in (('/Odometry',Odometry),('/course/odom',Odometry),('/scan',LaserScan))]
        self.timer=node.create_timer(1.,self.snapshot)

    def observe(self,topic,msg):
        now=time.monotonic()
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.node.get_clock().now().nanoseconds*1e-9-stamp
        state=self.state.setdefault(topic,dict(at=None,stamp=None,age_max=None,gap_max=0.,regressions=0,count=0))
        if state['at'] is not None:
            state['gap_max']=max(state['gap_max'],now-state['at'])
            if stamp<state['stamp']:
                state['regressions']+=1
        state.update(at=now,stamp=stamp,age_at_receive=age,
            age_max=age if state['age_max'] is None else max(state['age_max'],age),count=state['count']+1)

    def snapshot(self):
        now=time.monotonic()
        ros=self.node.get_clock().now().nanoseconds*1e-9
        streams={topic:dict(age_at_receive_s=s['age_at_receive'],
            age_at_receive_max_s=s['age_max'],source_age_s=ros-s['stamp'],
            receive_age_s=now-s['at'],receive_gap_max_s=s['gap_max'],
            samples=s['count'],stamp_regressions=s['regressions']) for topic,s in self.state.items()}
        self.file.write(json.dumps(dict(ros_s=ros,timer_delay_s=max(0.,now-self.previous_tick-1.),
            load_1m=os.getloadavg()[0],streams=streams))+'\n')
        self.file.flush()
        self.previous_tick=now
        for state in self.state.values():
            state.update(age_max=None,gap_max=0.,count=0)

    def close(self):
        self.timer.cancel()
        self.file.close()
