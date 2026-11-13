#!/usr/bin/env python3
"""Read-only snapshot of the actual local costmap, footprint and scan."""
import argparse
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from geometry_msgs.msg import PolygonStamped
from nav2_msgs.srv import GetCostmap
from sensor_msgs.msg import LaserScan, PointCloud2
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def line_cells(x0,y0,x1,y1):
    dx,dy=abs(x1-x0),abs(y1-y0)
    sx,sy=1 if x0<x1 else -1,1 if y0<y1 else -1
    error=dx-dy
    while True:
        yield x0,y0
        if x0==x1 and y0==y1:break
        twice=2*error
        if twice>-dy:error-=dy;x0+=sx
        if twice<dx:error+=dx;y0+=sy


def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    rclpy.init()
    node=rclpy.create_node('lab42_readonly_collision_snapshot')
    received={}
    subscriptions=[]
    for topic,kind,qos in [('/scan',LaserScan,qos_profile_sensor_data),
            ('/local_costmap/published_footprint',PolygonStamped,5),
            ('/course/points_raw',PointCloud2,qos_profile_sensor_data),
            ('/lab42/upper/status',String,5)]:
        subscriptions.append(node.create_subscription(kind,topic,
            lambda message,key=topic:received.__setitem__(key,message),qos))
    buffer=Buffer(cache_time=Duration(seconds=5))
    listener=TransformListener(buffer,node)
    service=node.create_client(GetCostmap,'/local_costmap/get_costmap')
    report={'motion_commands_sent':False,'captured_ros_s':node.get_clock().now().nanoseconds*1e-9}
    try:
        start=time.monotonic()
        end=start+10
        while time.monotonic()<end and (not service.service_is_ready() or
                '/scan' not in received or '/local_costmap/published_footprint' not in received or
                time.monotonic()<start+5):
            rclpy.spin_once(node,timeout_sec=.05)
        if not service.service_is_ready():raise RuntimeError('局部代价地图服务未就绪；先启动A和D，不需要E或enable')
        future=service.call_async(GetCostmap.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=3)
        if not future.done() or future.result() is None:raise RuntimeError('读取局部代价地图超时')
        grid=future.result().map
        meta=grid.metadata
        origin=meta.origin.position
        resolution=meta.resolution
        pose=buffer.lookup_transform(grid.header.frame_id,'base_footprint',Time())
        base=pose.transform.translation
        r=rotation(pose.transform.rotation)
        def body_xy(x,y):
            dx,dy=x-base.x,y-base.y
            return [r[0][0]*dx+r[1][0]*dy,r[0][1]*dx+r[1][1]*dy]
        def cell(x,y):return math.floor((x-origin.x)/resolution),math.floor((y-origin.y)/resolution)
        def cell_record(mx,my):
            if not (0<=mx<meta.size_x and 0<=my<meta.size_y):return {'outside':True,'cell':[mx,my]}
            x,y=origin.x+(mx+.5)*resolution,origin.y+(my+.5)*resolution
            return {'cell':[mx,my],'cost':int(grid.data[my*meta.size_x+mx]),'body_xy':body_xy(x,y)}
        footprint=received.get('/local_costmap/published_footprint')
        if footprint is None or footprint.header.frame_id!=grid.header.frame_id:
            raise RuntimeError('足迹未收到或与局部代价地图坐标系不一致')
        vertices=[(p.x,p.y) for p in footprint.polygon.points]
        boundary=set()
        for a,b in zip(vertices,vertices[1:]+vertices[:1]):boundary.update(line_cells(*cell(*a),*cell(*b)))
        perimeter=[cell_record(*xy) for xy in sorted(boundary)]
        lethal=[v for v in perimeter if v.get('cost')==254]
        report.update(grid_frame=grid.header.frame_id,resolution=resolution,
            robot_xy=[base.x,base.y],footprint_body=[body_xy(*xy) for xy in vertices],
            footprint_lethal_cells=lethal,
            footprint_unknown_cells=[v for v in perimeter if v.get('cost')==255],
            footprint_outside_cells=[v for v in perimeter if v.get('outside')],
            perimeter=perimeter,
            grid={'width':meta.size_x,'height':meta.size_y,'origin':[origin.x,origin.y],
                  'data':list(grid.data)},footprint_world=vertices)
        report['timing']={
            'costmap_stamp_s':grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9,
            'published_footprint_stamp_s':footprint.header.stamp.sec+footprint.header.stamp.nanosec*1e-9,
            'pose_tf_stamp_s':pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9}
        # The published footprint and latest pose can be from different updates.
        # Evaluate the configured outline at that pose too, instead of treating
        # a later empty perimeter as proof that a previous collision was false.
        pointer=Path.home()/'catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization/official_motion_bridge/current_run.txt'
        if pointer.is_file():
            setup_file=Path(pointer.read_text().strip())/'setup.json'
            if setup_file.is_file():
                setup=json.loads(setup_file.read_text())
                hx=setup['length']/2+setup['padding'];hy=setup['width']/2+setup['padding']
                outline=[(hx,hy),(hx,-hy),(-hx,-hy),(-hx,hy)]
                current=[(base.x+r[0][0]*x+r[0][1]*y,base.y+r[1][0]*x+r[1][1]*y) for x,y in outline]
                edges=set()
                for a,b in zip(current,current[1:]+current[:1]):edges.update(line_cells(*cell(*a),*cell(*b)))
                report['current_pose_footprint_lethal_cells']=[cell_record(*xy) for xy in sorted(edges)
                    if 0<=xy[0]<meta.size_x and 0<=xy[1]<meta.size_y and grid.data[xy[1]*meta.size_x+xy[0]]==254]
        near_lethal=[]
        for i,value in enumerate(grid.data):
            if value!=254:continue
            item=cell_record(i%meta.size_x,i//meta.size_x)
            x,y=item['body_xy']
            if abs(x)<.85 and abs(y)<.75:near_lethal.append(item)
        report['nearest_lethal_cells_body']=sorted(near_lethal,key=lambda v:math.hypot(*v['body_xy']))[:20]
        scan=received.get('/scan')
        if scan is not None:
            report['scan_frame']=scan.header.frame_id
            points=[]
            for i,d in enumerate(scan.ranges):
                if math.isfinite(d) and scan.range_min<=d<=min(scan.range_max,2.):
                    angle=scan.angle_min+i*scan.angle_increment
                    points.append([d*math.cos(angle),d*math.sin(angle)])
            report['scan_points']=points
            report['scan_min_range']=min((math.hypot(*v) for v in points),default=None)
        cloud=received.get('/course/points_raw')
        if cloud is not None:
            try:
                transform=buffer.lookup_transform('base_footprint',cloud.header.frame_id,
                    Time.from_msg(cloud.header.stamp))
                matrix=rotation(transform.transform.rotation)
                t=transform.transform.translation
                report['lidar_to_base_footprint']={'translation':[t.x,t.y,t.z],
                    'rotation':matrix,'stamp_s':transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9}
                fields={f.name:f for f in cloud.fields}
                assert all(fields[name].datatype==7 for name in ('x','y','z'))
                fmt='>f' if cloud.is_bigendian else '<f'
                near=[]
                wider=[]
                for i in range(cloud.width*cloud.height):
                    offset=(i//cloud.width)*cloud.row_step+(i%cloud.width)*cloud.point_step
                    p=[struct.unpack_from(fmt,cloud.data,offset+fields[name].offset)[0] for name in ('x','y','z')]
                    if not all(math.isfinite(v) for v in p):continue
                    q=[sum(matrix[j][k]*p[k] for k in range(3))+[t.x,t.y,t.z][j] for j in range(3)]
                    if abs(q[0])<1.2 and abs(q[1])<1.2 and -.3<q[2]<1.7:near.append(q)
                    if abs(q[0])<4 and abs(q[1])<4 and -.6<q[2]<1.7:wider.append(q)
                report['near_cloud_base_footprint']=near
                report['wider_cloud_base_footprint']=wider
            except Exception as error:report['cloud_error']=str(error)
        if '/lab42/upper/status' in received:report['upper_status']=json.loads(received['/lab42/upper/status'].data)
    except Exception as error:
        report['error']=str(error)
    finally:
        report['captured_ros_s']=node.get_clock().now().nanoseconds*1e-9
        output=args.output or Path.home()/('lab42_collision_'+str(time.time_ns())+'.json')
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        summary={k:v for k,v in report.items() if k not in ('grid','perimeter','scan_points','near_cloud_base_footprint','wider_cloud_base_footprint')}
        print(json.dumps(summary,ensure_ascii=False,indent=2))
        print('完整快照：',output)
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if 'error' in report else 0


if __name__=='__main__':raise SystemExit(main())
