"""Unfiltered estimated motion and the contemporaneous official bridge output."""
import csv
import math


class MotionEvidence:
    fields=['stamp_ros_s','received_ros_s','received_monotonic_s','child_frame_id',
            'vx','vy','vz','wx','wy','wz','speed_xy_mps','yaw_rate_radps',
            'odom_x','odom_y','odom_yaw',
            'bridge_vx','bridge_vy','bridge_wz','bridge_status_age_s']

    def __init__(self,directory):
        self.file=(directory/'csv/motion.csv').open('w',newline='')
        self.writer=csv.DictWriter(self.file,fieldnames=self.fields)
        self.writer.writeheader()
        self.last=None
        self.peak_speed=self.peak_yaw=0.

    def observe(self,message,ros,wall,upper,upper_at):
        v,w=message.twist.twist.linear,message.twist.twist.angular
        values=dict(vx=v.x,vy=v.y,vz=v.z,wx=w.x,wy=w.y,wz=w.z)
        speed,yaw=math.hypot(v.x,v.y),abs(w.z)
        p,q=message.pose.pose.position,message.pose.pose.orientation
        pose_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        row=dict(stamp_ros_s=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            received_ros_s=ros,received_monotonic_s=wall,child_frame_id=message.child_frame_id,
            **values,speed_xy_mps=speed,yaw_rate_radps=yaw,
            odom_x=p.x,odom_y=p.y,odom_yaw=pose_yaw,
            bridge_vx=upper.get('vx'),bridge_vy=upper.get('vy'),bridge_wz=upper.get('wz'),
            bridge_status_age_s=wall-upper_at)
        # JSON evidence stays valid even if an estimate is nonfinite. The CSV
        # retains the original value; the existing motion guard still rejects it.
        self.last={k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in row.items()}
        if math.isfinite(speed):self.peak_speed=max(self.peak_speed,speed)
        if math.isfinite(yaw):self.peak_yaw=max(self.peak_yaw,yaw)
        self.writer.writerow(row)
        self.file.flush()

    def summary(self):
        return dict(file='csv/motion.csv',peak_estimated_speed_xy_mps=self.peak_speed,
                    peak_estimated_yaw_rate_radps=self.peak_yaw,last_sample=self.last,
                    original_twist_unfiltered=True)

    def close(self):
        self.file.close()
