"""Measured-world, one-foot-at-a-time calibration and slow crawl primitives.

This module provides geometry and continuous foot targets. The ROS node must
gate phase changes on fresh feedback, support clearance and executed targets;
no contact force, dynamic balance or Gazebo validation is implied here.
"""
import math
import xml.etree.ElementTree as ET

from lite3_gait_core import (JOINTS, LEGS, body_from_world, forward, inverse,
                            measured_feet, smooth, support_target,
                            world_from_body)


def triangle_margin(point, vertices):
    """Signed distance to the nearest support-triangle edge, in metres."""
    if len(vertices) != 3 or any(len(v) < 2 for v in vertices):
        raise ValueError('Expected three supporting toe positions')
    if not all(math.isfinite(v) for p in (point, *vertices) for v in p[:2]):
        raise ValueError('Non-finite support geometry')
    cx = sum(v[0] for v in vertices)/3.0
    cy = sum(v[1] for v in vertices)/3.0
    points = sorted(vertices, key=lambda v: math.atan2(v[1]-cy, v[0]-cx))
    distances = []
    for a, b in zip(points, points[1:]+points[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length < 1e-8:
            raise ValueError('Degenerate support triangle')
        distances.append((dx*(point[1]-a[1])-dy*(point[0]-a[0]))/length)
    return min(distances)


def support_metrics(measured_pose, feet, com_body, leg, radius=0.022, ground_z=0.0):
    """Actual support triangle, COM projection and each geometric foot bottom."""
    if leg not in range(4) or len(feet) != 4:
        raise ValueError('Invalid support metric leg/feet')
    position, quaternion = measured_pose
    actual_com = world_from_body(com_body, position, quaternion)
    bottoms = tuple(f[2]-radius-ground_z for f in feet)
    others = {LEGS[i]: bottoms[i] for i in range(4) if i != leg}
    return {'leg': LEGS[leg], 'bottoms': bottoms, 'com': actual_com,
            'support_margin': triangle_margin(actual_com, [f for i, f in enumerate(feet) if i != leg]),
            'max_support_clearance': max(others.values()),
            'max_support_floor_error': max(abs(x) for x in others.values()),
            'target_leg_height': bottoms[leg], 'support_bottoms': others}


def _vector(element, key, default='0 0 0'):
    values = tuple(float(x) for x in (default if element is None else element.get(key, default)).split())
    if len(values) != 3 or not all(math.isfinite(x) for x in values):
        raise ValueError('Invalid URDF '+key)
    return values


def _matrix_product(a, b):
    return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _matrix_vector(a, v):
    return tuple(sum(a[i][j]*v[j] for j in range(3)) for i in range(3))


def _axis_rotation(axis, angle):
    length = math.sqrt(sum(x*x for x in axis))
    if length < 1e-10:
        raise ValueError('Invalid URDF zero joint axis')
    x, y, z = (v/length for v in axis)
    c, s, d = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return ((c+x*x*d, x*y*d-z*s, x*z*d+y*s),
            (y*x*d+z*s, c+y*y*d, y*z*d-x*s),
            (z*x*d-y*s, z*y*d+x*s, c+z*z*d))


class URDFCenterOfMass:
    """Whole articulated-model COM in TORSO, from URDF masses and joint axes.

    Link inertial origins, joint origin rpy, arbitrary revolute axes, fixed
    links and prismatic joints are respected. The torso origin is not used as
    a substitute for the articulated model's center of mass.
    """
    def __init__(self, path, root_link='TORSO'):
        root = ET.parse(path).getroot()
        self.root_link = root_link
        self.links = {}
        for link in root.findall('link'):
            name = link.get('name')
            if not name or name in self.links:
                raise ValueError('Invalid/duplicate URDF link')
            inertial = link.find('inertial')
            mass_tag = None if inertial is None else inertial.find('mass')
            mass = 0.0 if mass_tag is None else float(mass_tag.get('value'))
            if not math.isfinite(mass) or mass < 0:
                raise ValueError('Invalid URDF mass: '+name)
            origin = None if inertial is None else inertial.find('origin')
            self.links[name] = (mass, _vector(origin, 'xyz'))
        if root_link not in self.links:
            raise ValueError('Missing COM root link: '+root_link)
        self.children = {}
        child_names = set()
        for joint in root.findall('joint'):
            parent, child = joint.find('parent'), joint.find('child')
            if parent is None or child is None:
                raise ValueError('Invalid URDF joint parent/child')
            parent, child = parent.get('link'), child.get('link')
            if child in child_names or parent not in self.links or child not in self.links:
                raise ValueError('Invalid URDF joint tree')
            child_names.add(child)
            kind = joint.get('type')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError('Unsupported URDF joint type: '+str(kind))
            origin = joint.find('origin')
            rpy = _vector(origin, 'rpy')
            rotation = _matrix_product(_axis_rotation((0, 0, 1), rpy[2]),
                        _matrix_product(_axis_rotation((0, 1, 0), rpy[1]),
                                        _axis_rotation((1, 0, 0), rpy[0])))
            record = (joint.get('name'), kind, child, _vector(origin, 'xyz'), rotation,
                      _vector(joint.find('axis'), 'xyz', '1 0 0'))
            self.children.setdefault(parent, []).append(record)
        reached, queue = set(), [root_link]
        while queue:
            name = queue.pop()
            if name in reached:
                raise ValueError('URDF joint cycle')
            reached.add(name)
            queue.extend(j[2] for j in self.children.get(name, []))
        self.total_mass = sum(self.links[name][0] for name in reached)
        if self.total_mass <= 0:
            raise ValueError('No positive URDF mass in COM subtree')

    def body_com(self, joints):
        if isinstance(joints, dict):
            values = joints
        else:
            if len(joints) != len(JOINTS):
                raise ValueError('Expected twelve Lite3 joint positions for COM')
            values = dict(zip(JOINTS, joints))
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError('Non-finite joint position for COM')
        identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        weighted = [0.0, 0.0, 0.0]
        queue = [(self.root_link, (0.0, 0.0, 0.0), identity)]
        while queue:
            name, position, rotation = queue.pop()
            mass, local = self.links[name]
            inertial = _matrix_vector(rotation, local)
            for k in range(3):
                weighted[k] += mass*(position[k]+inertial[k])
            for joint_name, kind, child, origin, joint_rotation, axis in self.children.get(name, []):
                value = 0.0 if kind == 'fixed' else values.get(joint_name)
                if value is None:
                    raise ValueError('Missing measured joint for COM: '+joint_name)
                local_position = origin
                if kind == 'prismatic':
                    length = math.sqrt(sum(x*x for x in axis))
                    if length < 1e-10:
                        raise ValueError('Invalid URDF zero joint axis')
                    displacement = _matrix_vector(joint_rotation, tuple(value*x/length for x in axis))
                    local_position = tuple(origin[k]+displacement[k] for k in range(3))
                local_rotation = joint_rotation
                if kind in ('revolute', 'continuous'):
                    local_rotation = _matrix_product(joint_rotation, _axis_rotation(axis, value))
                translation = _matrix_vector(rotation, local_position)
                queue.append((child, tuple(position[k]+translation[k] for k in range(3)),
                              _matrix_product(rotation, local_rotation)))
        return tuple(v/self.total_mass for v in weighted)


class MeasuredSupportStep:
    """Shift onto measured support, lift, hold/transfer, and lower one foot.

    Support legs always use their captured world toe anchors and a planned
    body transform. The swing leg uses the current measured body transform,
    so body tilt cannot erase a requested world clearance. Before ``lift``,
    call ``begin_lift`` after the shift endpoint has physically settled.

    ``stride`` is a WORLD-frame XY displacement. For WASD, rotate body-frame
    velocity/placement into world coordinates in the ROS layer.
    """
    def __init__(self, position, quaternion, joints, leg, stride=(0.0, 0.0),
                 height=0.03, com_body=(0.0, 0.0, 0.0), margin=0.018):
        if leg not in range(4) or len(position) != 3 or len(quaternion) != 4:
            raise ValueError('Invalid measured step pose or leg')
        if (len(stride) != 2 or len(com_body) != 3 or
                not all(math.isfinite(x) for x in (*position, *quaternion,
                                                   *stride, *com_body, height, margin))):
            raise ValueError('Non-finite measured step parameter')
        if not 0.005 <= height <= 0.05 or not 0.005 <= margin <= 0.03:
            raise ValueError('Measured lift height/support margin outside range')
        if math.hypot(*stride) > 0.05:
            raise ValueError('Measured step stride exceeds 50 mm')
        self.leg = leg
        self.position = tuple(position)
        self.quaternion = tuple(quaternion)
        self.target_quaternion = self.quaternion
        self.anchors = [tuple(f) for f in measured_feet(position, quaternion, joints)]
        self.height = height
        self.stride = tuple(stride)
        self.com_body = tuple(com_body)
        self.margin = margin
        initial_com = world_from_body(com_body, position, quaternion)
        wanted_com = support_target([f for i, f in enumerate(self.anchors) if i != leg],
                                    initial_com[:2], margin)
        self.target_body = (position[0]+wanted_com[0]-initial_com[0],
                            position[1]+wanted_com[1]-initial_com[1], position[2])
        self.lift_start = None
        self.command_start = None

    def command_body(self, u):
        """Smooth body shift; measured initial height/rotation are retained."""
        fraction = smooth(u)
        return tuple(a+(b-a)*fraction for a, b in zip(self.position, self.target_body))

    @property
    def target_foot(self):
        if self.lift_start is None:
            return self.anchors[self.leg]
        return (self.lift_start[0]+self.stride[0], self.lift_start[1]+self.stride[1], self.lift_start[2])

    def support_margin(self, measured_pose, feet, com_body=None):
        com_body = self.com_body if com_body is None else com_body
        return support_metrics(measured_pose, feet, com_body, self.leg)['support_margin']

    def begin_lift(self, measured_pose, joints, last_command):
        """Latch real lift baseline and continuous endpoint after shift gate.

        Unlike V4.1, the commanded anchor is only the zero-phase endpoint;
        at full lift the target is the ACTUAL start toe plus ``height``.
        Thus an old endpoint 20 mm below the measured toe cannot permanently
        turn a 30 mm clearance request into a 10 mm clearance request.
        """
        if self.lift_start is not None:
            raise ValueError('Measured lift already initialized')
        position, quaternion = measured_pose
        self.lift_start = tuple(measured_feet(position, quaternion, joints)[self.leg])
        self.command_start = tuple(measured_feet(position, quaternion, last_command)[self.leg])

    def swing_world(self, stage, u):
        if self.lift_start is None:
            raise ValueError('Call begin_lift with fresh feedback before swinging')
        fraction = smooth(u)
        raised = (self.lift_start[0], self.lift_start[1], self.lift_start[2]+self.height)
        if stage == 'lift':
            return tuple(a+(b-a)*fraction for a, b in zip(self.command_start, raised))
        if stage == 'hold':
            return raised
        if stage == 'transfer':
            return (raised[0]+self.stride[0]*fraction,
                    raised[1]+self.stride[1]*fraction, raised[2])
        if stage in ('lower', 'settle'):
            fraction = 1.0 if stage == 'settle' else fraction
            return (raised[0]+self.stride[0], raised[1]+self.stride[1],
                    self.lift_start[2]+self.height*(1.0-fraction))
        raise ValueError('Unknown measured swing stage: '+stage)

    def targets(self, stage, u, measured_pose):
        """Twelve Lite3 joint positions; enforce slew rate in the ROS layer."""
        if not math.isfinite(u) or not 0.0 <= u <= 1.0:
            raise ValueError('Phase fraction must be finite and between zero and one')
        if stage == 'shift':
            body = self.command_body(u)
            return [q for i, foot in enumerate(self.anchors)
                    for q in inverse(i, body_from_world(foot, body, self.quaternion))]
        if stage not in ('lift', 'hold', 'transfer', 'lower', 'settle'):
            raise ValueError('Unknown measured step stage: '+stage)
        target = [q for i, foot in enumerate(self.anchors)
                  for q in inverse(i, body_from_world(foot, self.target_body, self.quaternion))]
        position, quaternion = measured_pose
        target[3*self.leg:3*self.leg+3] = inverse(
            self.leg, body_from_world(self.swing_world(stage, u), position, quaternion))
        return target

    def diagnostics(self, measured_pose, joints, radius=0.022, ground_z=0.0):
        """Separate requested-foot clearance from wrong/support-leg lifting.

        Values are geometric foot-bottom distances, not contact forces. The
        caller must not pass a test because an unrelated foot is elevated.
        """
        position, quaternion = measured_pose
        feet = measured_feet(position, quaternion, joints)
        metrics = support_metrics(measured_pose, feet, self.com_body, self.leg, radius, ground_z)
        relative = (float('nan') if self.lift_start is None else
                    feet[self.leg][2]-self.lift_start[2])
        metrics['actual_lift'] = relative
        metrics['body_xy_error'] = math.hypot(position[0]-self.target_body[0],
                                             position[1]-self.target_body[1])
        return metrics

