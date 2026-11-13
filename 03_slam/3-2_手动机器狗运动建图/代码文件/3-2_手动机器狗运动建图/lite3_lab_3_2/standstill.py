"""Stationary evidence from a continuous odom pose window, without retiming."""
from collections import deque
import math


class PoseStandstill:
    window_s=2.
    speed_limit=.005
    yaw_rate_limit=.005
    position_span_limit=.020
    yaw_span_limit=.020

    def __init__(self):
        self.samples=deque(maxlen=256)

    def observe_message(self,message):
        if message.header.frame_id!='odom' or message.child_frame_id!='base_link':
            self.samples.clear()
            return
        p,q=message.pose.pose.position,message.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.update(message.header.stamp.sec+message.header.stamp.nanosec*1e-9,p.x,p.y,yaw)

    def update(self,stamp,x,y,yaw):
        if not all(math.isfinite(v) for v in (stamp,x,y,yaw)):
            self.samples.clear()
            return
        if self.samples:
            gap=stamp-self.samples[-1][0]
            if gap<=0 or gap>.30:
                self.samples.clear()
            else:
                previous=self.samples[-1][3]
                yaw=previous+(yaw-previous+math.pi)%(2*math.pi)-math.pi
        self.samples.append((stamp,x,y,yaw))
        while len(self.samples)>2 and self.samples[1][0]<=stamp-self.window_s:
            self.samples.popleft()

    def report(self,now_ros):
        rows=list(self.samples)
        result=dict(stationary=False,ready=False,method='odom_pose_regression_2s',
            samples=len(rows),speed_limit_mps=self.speed_limit,yaw_limit_radps=self.yaw_rate_limit,
            position_span_limit_m=self.position_span_limit,yaw_span_limit_rad=self.yaw_span_limit)
        if len(rows)<5 or not -.05<=now_ros-rows[-1][0]<=1.:
            return result
        elapsed=rows[-1][0]-rows[0][0]
        if elapsed<self.window_s-.05:
            return result
        times=[row[0]-rows[0][0] for row in rows]
        mean=sum(times)/len(times)
        centered=[t-mean for t in times]
        denominator=sum(t*t for t in centered)
        slopes=[sum(t*row[axis] for t,row in zip(centered,rows))/denominator for axis in (1,2,3)]
        speed,yaw_rate=math.hypot(*slopes[:2]),abs(slopes[2])
        diameter=max(math.hypot(a[1]-b[1],a[2]-b[2]) for a in rows for b in rows)
        yaw_span=max(row[3] for row in rows)-min(row[3] for row in rows)
        result.update(ready=True,window_elapsed_s=elapsed,fitted_speed_mps=speed,
            fitted_yaw_rate_radps=yaw_rate,position_span_m=diameter,yaw_span_rad=yaw_span,
            stationary=(speed<=self.speed_limit and yaw_rate<=self.yaw_rate_limit and
                        diameter<=self.position_span_limit and yaw_span<=self.yaw_span_limit))
        return result
