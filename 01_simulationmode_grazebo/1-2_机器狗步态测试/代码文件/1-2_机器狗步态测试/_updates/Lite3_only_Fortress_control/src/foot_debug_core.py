"""Foot sphere bottom position from synchronized measured pose and joints."""
import math
from collections import OrderedDict
from lite3_gait_core import forward


class PosePairs:
    """Match TFMessage names to timestamped PoseArray by identical geometry.

    Both bridge conversions retain Pose_V element order and double values.
    Never assign a receive-time/clock approximation to an unstamped pose.
    """
    def __init__(self):
        self.named=OrderedDict()
        self.stamped=OrderedDict()

    def _match(self, key):
        if key not in self.named or key not in self.stamped:
            return None
        names=self.named.pop(key)
        stamp=self.stamped.pop(key)
        if 'lite3_control' not in names or stamp<=0:
            return None
        pose=key[names.index('lite3_control')]
        return stamp,pose[:3],pose[3:]

    def names(self,names,poses):
        if len(names)!=len(poses) or len(set(names))<1:
            return None
        key=tuple(tuple(p) for p in poses)
        self.named[key]=list(names)
        if len(self.named)>200:
            self.named.popitem(last=False)
        return self._match(key)

    def time(self,stamp,poses):
        if not math.isfinite(stamp) or stamp<=0:
            return None
        key=tuple(tuple(p) for p in poses)
        self.stamped[key]=stamp
        if len(self.stamped)>200:
            self.stamped.popitem(last=False)
        return self._match(key)


def world_feet(body_position, quaternion, joint_positions):
    x,y,z,w=quaternion
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm<0.5:
        raise ValueError('Invalid body orientation')
    x,y,z,w=[q/norm for q in quaternion]
    matrix=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
    result=[]
    for leg in range(4):
        point=forward(leg,joint_positions[3*leg:3*leg+3])
        result.append(tuple(body_position[k]+sum(matrix[k][i]*point[i] for i in range(3)) for k in range(3)))
    return result
