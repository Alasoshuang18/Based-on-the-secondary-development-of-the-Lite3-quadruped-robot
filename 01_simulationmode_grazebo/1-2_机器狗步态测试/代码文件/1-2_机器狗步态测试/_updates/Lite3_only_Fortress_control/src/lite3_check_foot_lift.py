#!/usr/bin/env python3
"""Read-only flat-ground foot-height diagnostics; no control publishing."""
import math
import time
from collections import deque

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from lite3_gait_core import JOINTS, LEGS
from lite3_gait_core import measured_feet


def stamp(header):
    return header.stamp.sec+header.stamp.nanosec/1e9


def main():
    rclpy.init()
    node=rclpy.create_node('lite3_foot_lift_check',
                           parameter_overrides=[Parameter('use_sim_time', value=True)])
    states=deque(maxlen=200)
    poses=deque(maxlen=200)
    seen=set()
    stats={leg: {'min': float('inf'), 'max': -float('inf')} for leg in LEGS}
    last_report=time.monotonic()
    samples=0

    def joints(message):
        if len(message.name)!=len(message.position):
            return
        data=dict(zip(message.name,message.position))
        if all(j in data and math.isfinite(data[j]) for j in JOINTS):
            states.append((stamp(message.header),[data[j] for j in JOINTS]))

    def pose(message):
        for tf in message.transforms:
            seen.add(tf.child_frame_id)
            if tf.child_frame_id != 'lite3_control':
                continue
            t,q=tf.transform.translation,tf.transform.rotation
            value=(t.x,t.y,t.z,q.x,q.y,q.z,q.w)
            when=stamp(tf.header)
            if when>0 and all(math.isfinite(x) for x in value):
                poses.append((when,value[:3],value[3:]))

    subscriptions=[
        node.create_subscription(JointState,'/lite3_sim/joint_state_broadcaster/joint_states',joints,qos_profile_sensor_data),
        node.create_subscription(TFMessage,'/lite3_sim/model_pose',pose,qos_profile_sensor_data)]
    print('只读测量：运行至按 Ctrl+C；先做四腿抬脚测试，之后可在键盘终端按 W。',flush=True)
    print('下方高度是足部球形碰撞体底面距 z=0 平地的几何距离。',flush=True)
    print('此工具不检测接触力，不修改模型或发送关节命令。',flush=True)
    last_pair=None
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=0.05)
            if states and poses:
                pt,position,quaternion=poses[-1]
                jt,q=min(states,key=lambda s: abs(s[0]-pt))
                key=(pt,jt)
                # No guessing from unsynchronized samples or zero timestamps.
                if pt>0 and jt>0 and abs(pt-jt)<=0.025 and key!=last_pair:
                    last_pair=key
                    feet=measured_feet(position,quaternion,q)
                    samples+=1
                    for leg,foot in zip(LEGS,feet):
                        bottom=foot[2]-0.022
                        stats[leg]['min']=min(stats[leg]['min'],bottom)
                        stats[leg]['max']=max(stats[leg]['max'],bottom)
                    if time.monotonic()-last_report>=1:
                        last_report=time.monotonic()
                        height=' '.join(f'{leg}={1000*(f[2]-0.022):+.1f}mm' for leg,f in zip(LEGS,feet))
                        print(f'sim={pt:.2f} samples={samples} body=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}) '+height,flush=True)
            if time.monotonic()-last_report>5:
                print(f'等待同步数据；关节样本={len(states)} 同步位姿={len(poses)} '
                      f'model_seen={"lite3_control" in seen}；检查 C 终端的 PosePublisher 和桥接。',flush=True)
                last_report=time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        print(f'有效同步样本：{samples}')
        if samples:
            for leg in LEGS:
                print(f'{leg}: 最低 {stats[leg]["min"]*1000:.1f} mm，最高 {stats[leg]["max"]*1000:.1f} mm')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
