"""Read-only, bounded point-cloud and model-pose verification on the board."""
import json
import math
from pathlib import Path
import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

rclpy.init()
node = Node('navigation_readonly_probe')
result = {'clouds': 0, 'poses': 0}
stamps = []


def cloud(msg):
    result['clouds'] += 1
    stamps.append(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
    fields = {field.name: field.offset for field in msg.fields}
    valid = 0
    quadrants = set()
    for row in range(msg.height):
        for col in range(msg.width):
            offset = row*msg.row_step+col*msg.point_step
            xyz = [struct.unpack_from(('>' if msg.is_bigendian else '<')+'f',msg.data,offset+fields[k])[0]
                   for k in ('x','y','z')]
            if all(math.isfinite(v) for v in xyz):
                valid += 1
                quadrants.add(('front' if xyz[0]>=0 else 'back')+('left' if xyz[1]>=0 else 'right'))
    result.update(frame=msg.header.frame_id,width=msg.width,height=msg.height,
                  finite_points=valid,quadrants=sorted(quadrants),fields=list(fields))


def pose(msg):
    for transform in msg.transforms:
        if transform.child_frame_id == 'lite3_control':
            result['poses'] += 1
            p = transform.transform.translation
            result['body_xyz'] = [p.x,p.y,p.z]


node.create_subscription(PointCloud2,'/lite3_sim/mid360/points',cloud,qos_profile_sensor_data)
node.create_subscription(TFMessage,'/lite3_sim/model_pose',pose,qos_profile_sensor_data)
end = time.monotonic()+18
while time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.05)
if len(stamps)>1 and stamps[-1]>stamps[0]:
    result['sim_cloud_hz'] = (len(stamps)-1)/(stamps[-1]-stamps[0])
path = Path(__file__).resolve().parents[1]/'.runtime/pointcloud_probe.json'
path.parent.mkdir(exist_ok=True)
path.write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result),flush=True)
node.destroy_node()
rclpy.shutdown()
if result['clouds']<2 or result.get('finite_points',0)==0:
    raise SystemExit('Point cloud verification failed')
