"""Pure-Python Lite3 kinematics and slow three-support crawl planner.

No ROS imports; virtual foot anchors model no-slip flat-ground contact.
This is an experimental kinematic controller, not dynamic balance control.
"""
import math

LEGS = ('FL', 'FR', 'HL', 'HR')
JOINTS = [f'{leg}_{axis}_joint' for leg in LEGS for axis in ('HipX', 'HipY', 'Knee')]
HIP = [(0.1745, 0.062), (0.1745, -0.062), (-0.1745, 0.062), (-0.1745, -0.062)]
LIMITS = [(-0.523, 0.523), (-2.67, 0.314), (0.524, 2.792)]
STAND = [0.0, -0.65, 1.3] * 4
L1, L2, SIDE = 0.20, 0.21012, 0.09735


def clock_step_kind(delta):
    """A bounded positive sampling gap is not necessarily a clock reset."""
    if not math.isfinite(delta) or delta < 0 or delta > 1.0:
        return 'fault'
    if delta > 0.2:
        return 'hold'
    return 'normal'


class TrackingGuard:
    """Use one controller-state sample, not a future target vs older feedback.

    Only fresh, increasing simulation timestamps count toward persistence.
    """
    def __init__(self):
        self.stamp = None
        self.since = None
        self.maximum = 0.0
        self.joint = ''
        self.failure = ''

    def update(self, names, errors, stamp):
        if (len(names) != len(errors) or len(set(names)) != len(names) or
                not math.isfinite(stamp) or stamp < 0 or
                not all(math.isfinite(x) for x in errors)):
            return False
        data = dict(zip(names, errors))
        if not all(j in data for j in JOINTS):
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False
        if self.stamp is not None and stamp-self.stamp > 0.2:
            self.since = None  # Missing samples cannot prove continuous error.
        self.stamp = stamp
        self.joint = max(JOINTS, key=lambda j: abs(data[j]))
        self.maximum = abs(data[self.joint])
        self.failure = ''
        if self.maximum <= 0.35:
            self.since = None
        elif self.maximum > 0.8:
            self.failure = f'Controller tracking error: {self.joint}={self.maximum:.3f} rad (>0.8 immediate)'
        else:
            if self.since is None:
                self.since = stamp
            if stamp-self.since >= 0.3:
                self.failure = (f'Controller tracking error: {self.joint}={self.maximum:.3f} rad '
                                f'(>0.35 for {stamp-self.since:.3f} sim s)')
        return True


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def smooth(t):
    t = clamp(t, 0.0, 1.0)
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c*x - s*y, s*x + c*y


def rotation_matrix(quaternion):
    """Rotation of a measured Gazebo model pose, quaternion order x,y,z,w."""
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError('Invalid measured body quaternion')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 0.5:
        raise ValueError('Invalid measured body quaternion')
    x, y, z, w = (v/norm for v in quaternion)
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def world_from_body(point, position, quaternion):
    matrix = rotation_matrix(quaternion)
    return tuple(position[k]+sum(matrix[k][j]*point[j] for j in range(3)) for k in range(3))


def body_from_world(point, position, quaternion):
    """Inverse of the measured model transform, including roll and pitch."""
    matrix = rotation_matrix(quaternion)
    delta = [point[k]-position[k] for k in range(3)]
    return tuple(sum(matrix[k][j]*delta[k] for k in range(3)) for j in range(3))


def measured_feet(position, quaternion, joints):
    if len(joints) != 12:
        raise ValueError('Expected twelve measured joint positions')
    return [world_from_body(forward(i, joints[3*i:3*i+3]), position, quaternion)
            for i in range(4)]


def yaw_from_quaternion(quaternion):
    matrix = rotation_matrix(quaternion)
    return math.atan2(matrix[1][0], matrix[0][0])


class MeasuredSwing:
    """World-space swing target from a measured toe start and measured body.

    The actual toe is the lift-test baseline. A separate commanded toe, formed
    from the last published endpoint and the same body transform, preserves
    command continuity even when measured joints lag that endpoint.
    """
    def __init__(self, leg, actual_toe, planned_toe, initial_heading, commanded_toe=None):
        if leg not in range(4) or len(actual_toe) != 3 or len(planned_toe) != 3:
            raise ValueError('Invalid measured swing start')
        if not all(math.isfinite(v) for v in (*actual_toe, *planned_toe, initial_heading)):
            raise ValueError('Non-finite measured swing start')
        self.leg = leg
        self.actual_toe = tuple(actual_toe)
        self.commanded_toe = tuple(actual_toe if commanded_toe is None else commanded_toe)
        if len(self.commanded_toe) != 3 or not all(math.isfinite(v) for v in self.commanded_toe):
            raise ValueError('Invalid commanded swing start')
        self.planned_toe = tuple(planned_toe)
        self.heading = initial_heading

    def world_target(self, planned_toe):
        dx = planned_toe[0]-self.planned_toe[0]
        dy = planned_toe[1]-self.planned_toe[1]
        wx, wy = rotate(dx, dy, self.heading)
        return (self.commanded_toe[0]+wx,
                self.commanded_toe[1]+wy,
                self.commanded_toe[2]+planned_toe[2])

    def joint_target(self, planned_toe, body_position, body_quaternion):
        target = self.world_target(planned_toe)
        return inverse(self.leg, body_from_world(target, body_position, body_quaternion))


class JointTargetRamp:
    """Coordinated endpoint ramp; never exceed the configured joint rate.

    A pending endpoint must be reached before the planner advances again.
    Large corrections and an overdue transition still fail explicitly.
    """
    def __init__(self, start, rate=0.8, max_correction=0.10, timeout=0.35):
        if (len(start) != len(JOINTS) or not all(math.isfinite(v) for v in start)
                or not all(math.isfinite(v) and v > 0 for v in (rate, max_correction, timeout))):
            raise ValueError('Invalid joint target ramp settings')
        self.current = list(start)
        self.rate, self.max_correction, self.timeout = rate, max_correction, timeout
        self.goal = None
        self.elapsed = 0.0
        self.lag = 0.0
        self.largest_joint = ''

    @property
    def pending(self):
        return self.goal is not None

    def push(self, target, dt):
        if self.pending:
            raise ValueError('Reach pending joint endpoint before advancing planner')
        self._step_time(dt)
        if len(target) != len(JOINTS) or not all(math.isfinite(v) for v in target):
            raise ValueError('Non-finite or incomplete joint target')
        index = max(range(len(JOINTS)), key=lambda i: abs(target[i]-self.current[i]))
        difference = abs(target[index]-self.current[index])
        self.largest_joint = JOINTS[index]
        if difference > self.rate*dt+self.max_correction:
            raise ValueError(f'Joint target correction too large: {JOINTS[index]} '
                             f'delta={difference:.6f} rad, rate_step={self.rate*dt:.6f} rad, '
                             f'max_correction={self.max_correction:.3f} rad')
        self.goal = list(target)
        self.elapsed = 0.0
        return self.advance(dt)

    def _step_time(self, dt):
        if not math.isfinite(dt) or not 0 < dt <= 0.2:
            raise ValueError('Invalid joint target ramp time step')

    def advance(self, dt):
        self._step_time(dt)
        if self.goal is None:
            self.lag = 0.0
            return list(self.current)
        difference = [goal-current for goal,current in zip(self.goal,self.current)]
        maximum = max(abs(value) for value in difference)
        scale = min(1.0,self.rate*dt/max(maximum,1e-15))
        candidate = [current+scale*delta for current,delta in zip(self.current,difference)]
        self.lag = max(abs(goal-value) for goal,value in zip(self.goal,candidate))
        self.elapsed += dt
        if self.lag > 1e-9 and self.elapsed > self.timeout:
            raise ValueError(f'Joint target ramp overdue: {self.largest_joint}; '
                             f'lag={self.lag:.6f} rad after {self.elapsed:.3f} sim s')
        self.current = candidate
        if self.lag <= 1e-9:
            self.current = self.goal
            self.goal = None
            self.lag = 0.0
            self.elapsed = 0.0
        return list(self.current)


def forward(leg, q):
    a, b, k = q
    x = L1*math.sin(b) + L2*math.sin(b+k)
    z = -L1*math.cos(b) - L2*math.cos(b+k)
    d = SIDE if leg in (0, 2) else -SIDE
    return (HIP[leg][0] + x,
            HIP[leg][1] + d*math.cos(a) + z*math.sin(a),
            -d*math.sin(a) + z*math.cos(a))


def inverse(leg, point):
    x, y, z = point
    x, y = x - HIP[leg][0], y - HIP[leg][1]
    d = SIDE if leg in (0, 2) else -SIDE
    lateral = y*y + z*z - d*d
    if lateral <= 0:
        raise ValueError('Foot inside abduction workspace')
    sagittal_z = -math.sqrt(lateral)
    a = math.atan2(sagittal_z, d) - math.atan2(z, y)
    a = (a + math.pi) % (2*math.pi) - math.pi
    cosine = (x*x + sagittal_z*sagittal_z - L1*L1 - L2*L2)/(2*L1*L2)
    if not -1.0 <= cosine <= 1.0:
        raise ValueError('Unreachable foot target')
    k = math.acos(cosine)
    b = math.atan2(x, -sagittal_z) - math.atan2(L2*math.sin(k), L1+L2*math.cos(k))
    q = [a, b, k]
    for value, (lower, upper) in zip(q, LIMITS):
        if not math.isfinite(value) or not lower + 0.015 <= value <= upper - 0.015:
            raise ValueError('IK target outside joint limit margin')
    return q


def body_coordinates(point, body, yaw, roll, pitch):
    # Inverse Rz(yaw) Ry(pitch) Rx(roll).
    x, y = rotate(point[0]-body[0], point[1]-body[1], -yaw)
    z = point[2] - body[2]
    c, s = math.cos(pitch), math.sin(pitch)
    x, z = c*x-s*z, s*x+c*z
    c, s = math.cos(roll), math.sin(roll)
    y, z = c*y+s*z, -s*y+c*z
    return x, y, z


def support_target(points, preferred, margin):
    """Nearest point in a support triangle inset by margin (metres).

    This constrains the planned torso origin, not the measured whole-body COM.
    Unlike a centroid target it does not require a large sway on every step.
    """
    cx = sum(p[0] for p in points)/3
    cy = sum(p[1] for p in points)/3
    vertices = sorted(points, key=lambda p: math.atan2(p[1]-cy, p[0]-cx))
    lines = []
    for a, b in zip(vertices, vertices[1:]+vertices[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length < 1e-8:
            raise ValueError('Degenerate support triangle')
        nx, ny = -dy/length, dx/length
        lines.append((nx, ny, nx*a[0]+ny*a[1]+margin))
    inset = []
    for a, b in zip(lines, lines[1:]+lines[:1]):
        det = a[0]*b[1]-a[1]*b[0]
        if abs(det) < 1e-8:
            raise ValueError('Degenerate support triangle')
        inset.append(((a[2]*b[1]-a[1]*b[2])/det,
                      (a[0]*b[2]-a[2]*b[0])/det))
    if any(nx*x+ny*y < c-1e-8 for x,y in inset for nx,ny,c in lines):
        raise ValueError('Support triangle too small for margin')
    if all(nx*preferred[0]+ny*preferred[1] >= c for nx,ny,c in lines):
        return list(preferred)
    candidates = []
    for a,b in zip(inset, inset[1:]+inset[:1]):
        dx,dy = b[0]-a[0], b[1]-a[1]
        t = clamp(((preferred[0]-a[0])*dx+(preferred[1]-a[1])*dy)/
                  max(dx*dx+dy*dy, 1e-16), 0, 1)
        candidates.append([a[0]+t*dx, a[1]+t*dy])
    return min(candidates, key=lambda p: math.hypot(p[0]-preferred[0],p[1]-preferred[1]))


def swing_profile(t):
    """Lift vertically, transfer at clearance, then lower vertically.

    Returns horizontal progress and normalized height, with zero endpoint
    velocity/acceleration at all three segment boundaries.
    """
    t = clamp(t, 0, 1)
    if t < 0.30:
        return 0.0, smooth(t/0.30)
    if t < 0.70:
        return smooth((t-0.30)/0.40), 1.0
    return 1.0, 1.0-smooth((t-0.70)/0.30)


class Crawl:
    """Shift onto the support triangle, swing one foot, then settle.

    Commands represent body-frame velocity requests. Foot placement is open
    loop; measured orientation may supply bounded roll/pitch corrections.
    """
    ORDER = (3, 1, 2, 0)

    def __init__(self, shift_time=0.9, swing_time=1.8, settle_time=0.2,
                 step_height=0.025, max_stride=0.04, support_margin=0.018):
        if not (0.5 <= shift_time <= 3 and 0.4 <= swing_time <= 3 and
                0.1 <= settle_time <= 1 and 0.005 <= step_height <= 0.03 and
                0.01 <= max_stride <= 0.05 and 0.01 <= support_margin <= 0.03):
            raise ValueError('Gait parameters outside conservative ranges')
        self.shift_time, self.swing_time = shift_time, swing_time
        self.settle_time, self.step_height = settle_time, step_height
        self.max_stride = max_stride
        self.support_margin = support_margin
        self.cycle_time = 4*(shift_time+swing_time+settle_time)
        self.height = -forward(0, STAND[:3])[2]
        self.feet = [list(forward(i, STAND[3*i:3*i+3])) for i in range(4)]
        self.nominal = [list(foot) for foot in self.feet]
        self.nominal_center = [sum(f[k] for f in self.nominal)/4 for k in (0, 1)]
        for foot in self.feet:
            foot[2] = 0.0
        self.body = [0.0, 0.0, self.height]
        self.yaw = 0.0
        self.state, self.elapsed, self.index = 'idle', 0.0, 0
        self.stop_requested = False
        self.roll = self.pitch = 0.0
        self.target_body = list(self.body)
        self.target_yaw = 0.0

    def _center(self, omit=None):
        points = [f for i, f in enumerate(self.feet) if i != omit]
        return [sum(f[axis] for f in points)/len(points) for axis in (0, 1)]

    def _shift(self, velocity, turn):
        self.leg = self.ORDER[self.index]
        self.start_body, self.start_yaw = list(self.body), self.yaw
        all_center = self._center()
        cx, cy = rotate(*self.nominal_center, self.yaw)
        preferred = [all_center[0]-cx, all_center[1]-cy]
        center = support_target([f for i,f in enumerate(self.feet) if i != self.leg],
                                preferred, self.support_margin)
        self.target_body = [center[0], center[1], self.height]
        self.target_yaw = self.yaw + turn*(self.cycle_time/4)
        self.start_foot = list(self.feet[self.leg])
        all_center = self._center()
        # Re-home each foot to the nominal footprint, rather than accumulating
        # a per-foot twist indefinitely during command reversals.
        rx = self.nominal[self.leg][0]-self.nominal_center[0]
        ry = self.nominal[self.leg][1]-self.nominal_center[1]
        tx, ty = rotate(rx, ry, self.yaw+turn*self.cycle_time/2)
        vx, vy = rotate(velocity*self.cycle_time, 0.0, self.yaw)
        dx = all_center[0]+tx+vx-self.start_foot[0]
        dy = all_center[1]+ty+vy-self.start_foot[1]
        length = math.hypot(dx, dy)
        scale = min(1.0, self.max_stride/max(length, 1e-12))
        self.target_foot = [self.start_foot[0]+dx*scale,
                            self.start_foot[1]+dy*scale, 0.0]
        self.state, self.elapsed = 'shift', 0.0

    def _recenter(self):
        center = self._center()
        cx, cy = rotate(*self.nominal_center, self.yaw)
        self.start_body = list(self.body)
        self.target_body = [center[0]-cx, center[1]-cy, self.height]
        self.state, self.elapsed = 'recenter', 0.0

    def _swing_profile(self, t):
        return swing_profile(t)

    def update(self, dt, velocity, turn, roll_correction=0.0, pitch_correction=0.0):
        if not 0.0 <= dt <= 0.05:
            raise ValueError('Invalid planner time step')
        velocity, turn = clamp(velocity, -0.004, 0.004), clamp(turn, -0.012, 0.012)
        # The first version supports the four single-axis WASD commands only.
        if abs(turn) > 0.002:
            velocity = 0.0
        moving = abs(velocity) > 0.0005 or abs(turn) > 0.002
        if not moving and self.state not in ('idle', 'recenter'):
            self.stop_requested = True
        if self.state == 'idle' and moving:
            self.stop_requested = False
            self._shift(velocity, turn)
        self.elapsed += dt
        if self.state == 'shift':
            u = smooth(self.elapsed/self.shift_time)
            self.body = [a+(b-a)*u for a, b in zip(self.start_body, self.target_body)]
            self.yaw = self.start_yaw + (self.target_yaw-self.start_yaw)*u
            if self.elapsed >= self.shift_time:
                if self.stop_requested:
                    self._recenter()  # Never lift a new leg after a stop request.
                else:
                    self.state, self.elapsed = 'swing', 0.0
        elif self.state == 'swing':
            t = clamp(self.elapsed/self.swing_time, 0.0, 1.0)
            u, lift = self._swing_profile(t)
            self.feet[self.leg] = [self.start_foot[i]+(self.target_foot[i]-self.start_foot[i])*u
                                    for i in range(3)]
            self.feet[self.leg][2] = self.step_height * lift
            if self.elapsed >= self.swing_time:
                self.feet[self.leg] = list(self.target_foot)
                self.state, self.elapsed = 'settle', 0.0
        elif self.state == 'settle' and self.elapsed >= self.settle_time:
            if self.stop_requested or not moving:
                self._recenter()
            else:
                self.index = (self.index+1) % 4
                self._shift(velocity, turn)
        elif self.state == 'recenter':
            u = smooth(self.elapsed/self.shift_time)
            self.body = [a+(b-a)*u for a, b in zip(self.start_body, self.target_body)]
            if self.elapsed >= self.shift_time:
                self.state, self.elapsed = 'idle', 0.0
        # Limit balance correction slew to avoid a noisy sensor causing jumps.
        correction_step = 0.08*dt
        self.roll += clamp(roll_correction-self.roll, -correction_step, correction_step)
        self.pitch += clamp(pitch_correction-self.pitch, -correction_step, correction_step)
        result = []
        for leg, foot in enumerate(self.feet):
            result.extend(inverse(leg, body_coordinates(foot, self.body, self.yaw,
                                                       self.roll, self.pitch)))
        return result


def bezier_swing(t):
    """Cubic Bezier horizontal interpolation and two vertical half curves.

    Endpoint controls are duplicated: B(u)=3u^2-2u^3. Returns normalized
    position (horizontal, vertical) and derivatives with respect to phase.
    Divide derivatives by swing duration to obtain physical velocities.
    """
    t = clamp(t, 0.0, 1.0)
    def b(u):
        return u*u*(3-2*u)
    def db(u):
        return 6*u*(1-u)
    if t < 0.5:
        z, dz = b(2*t), 2*db(2*t)
    else:
        z, dz = 1-b(2*t-1), -2*db(2*t-1)
    return b(t), z, db(t), dz


class PhaseWalk(Crawl):
    """Article-inspired phase/duty-factor walk, adapted to position control.

    Touchdown phase offsets relative to FL are FL=0, FR=.5, HL=.75, HR=.25.
    Swing order is HR, FR, HL, FL. Duty > .75 leaves all-feet-down intervals
    for support transfer. No measured COM velocity, MPC or WBC is claimed.
    """
    OFFSETS = (0.0, 0.5, 0.75, 0.25)

    def __init__(self, cycle_period=12.0, duty_factor=0.82, settle_time=0.15,
                 step_height=0.025, max_stride=0.04, support_margin=0.018):
        values = (cycle_period, duty_factor, settle_time, step_height,
                  max_stride, support_margin)
        if not all(math.isfinite(x) for x in values):
            raise ValueError('Non-finite phase gait parameter')
        if not 0.75 < duty_factor < 1.0:
            raise ValueError('Position-controlled walk needs duty_factor > 0.75')
        swing = (1-duty_factor)*cycle_period
        shift = cycle_period/4-swing-settle_time
        super().__init__(shift, swing, settle_time, step_height, max_stride, support_margin)
        self.duty_factor = duty_factor

    def _swing_profile(self, t):
        x,z,_,_ = bezier_swing(t)
        return x,z

    def _shift(self, velocity, turn):
        super()._shift(velocity, turn)
        # Predicted nominal hip at touchdown plus half-stance feedforward.
        # Command velocity is used because actual body velocity is unavailable.
        # Never replace measured velocity with zero in a feedback term.
        center = self._center()
        rx = self.nominal[self.leg][0]-self.nominal_center[0]
        ry = self.nominal[self.leg][1]-self.nominal_center[1]
        lead_time = (0.5+self.duty_factor/2)*self.cycle_time
        tx,ty = rotate(rx, ry, self.yaw+turn*lead_time)
        vx,vy = rotate(velocity*lead_time, 0.0, self.yaw)
        dx,dy = center[0]+tx+vx-self.start_foot[0], center[1]+ty+vy-self.start_foot[1]
        scale = min(1.0,self.max_stride/max(math.hypot(dx,dy),1e-12))
        self.target_foot = [self.start_foot[0]+dx*scale,self.start_foot[1]+dy*scale,0.0]

    @property
    def contact_plan(self):
        return [not (self.state == 'swing' and i == self.leg) for i in range(4)]

    @property
    def swing_phase(self):
        return clamp(self.elapsed/self.swing_time,0,1) if self.state == 'swing' else 0.0
