"""ROS-independent source-time health checks, trial statistics and ASCII PCD reload."""
from collections import deque
import math
from pathlib import Path
import statistics
import time


class StreamMonitor:
    STREAMS = ('cloud', 'imu', 'odom', 'map')

    def __init__(self, stale_timeout_s=1.0, imu_samples=100,
                 imu_norm_min_mps2=8.5, imu_norm_max_mps2=11.0,
                 imu_norm_std_max_mps2=0.8, latency_window=1000):
        values = (stale_timeout_s, imu_norm_min_mps2, imu_norm_max_mps2, imu_norm_std_max_mps2)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Health thresholds must be finite and positive')
        if imu_norm_min_mps2 >= imu_norm_max_mps2 or imu_samples < 2 or latency_window < 1:
            raise ValueError('Invalid health window or IMU range')
        self.timeout = stale_timeout_s
        self.imu_required = int(imu_samples)
        self.imu_min, self.imu_max, self.imu_std_max = values[1:]
        self.norms = deque(maxlen=self.imu_required)
        # Only an explicit trial begun from a healthy stationary window permits
        # dynamics. This is recording state, never a robot motion command.
        self.motion_active = False
        self.motion_imu_verified = False
        self.motion_initial_health = None
        self.records = {name: dict(count=0, invalid_stamps=0, stamp=None, wall=None,
                                  valid_latest=False, latencies=deque(maxlen=latency_window))
                        for name in self.STREAMS}

    def observe(self, stream, stamp, now, wall=None):
        """Reject repeated/backward/future stamps; receive time never replaces source time."""
        r = self.records[stream]
        wall = time.monotonic() if wall is None else wall
        valid = (all(math.isfinite(v) for v in (stamp, now, wall)) and stamp > 0
                 and now-stamp >= -0.05 and (r['stamp'] is None or stamp > r['stamp']))
        if not valid:
            r['invalid_stamps'] += 1
            r['valid_latest'] = False
            return False
        r.update(count=r['count']+1, stamp=stamp, wall=wall, valid_latest=True)
        r['latencies'].append(now-stamp)
        return now-stamp <= self.timeout

    def invalidate_imu(self):
        self.norms.clear()
        self.motion_imu_verified = False

    def begin_motion(self, now, wall=None):
        if self.motion_active:
            return False
        initial = self.health(now, wall=wall, require_stationary=True)
        if not initial['ready']:
            return False
        self.motion_initial_health = initial
        self.motion_imu_verified = True
        self.motion_active = True
        return True

    def end_motion(self):
        self.motion_active = False
        self.motion_imu_verified = False
        self.motion_initial_health = None

    def imu(self, acceleration):
        if len(acceleration) != 3 or not all(math.isfinite(v) for v in acceleration):
            self.invalidate_imu()
            return False
        norm = math.sqrt(sum(v*v for v in acceleration))
        if not math.isfinite(norm):
            self.invalidate_imu()
            return False
        self.norms.append(norm)
        return True

    def health(self, now, wall=None, *, require_stationary=False):
        wall = time.monotonic() if wall is None else wall
        stale, streams = [], {}
        for name, r in self.records.items():
            source_age = None if r['stamp'] is None else now-r['stamp']
            receive_age = None if r['wall'] is None else wall-r['wall']
            if (not r['valid_latest'] or source_age is None or receive_age is None
                    or not all(math.isfinite(v) for v in (source_age, receive_age))
                    or not -0.05 <= source_age <= self.timeout
                    or not 0 <= receive_age <= self.timeout):
                stale.append(name)
            latency = r['latencies']
            streams[name] = dict(count=r['count'], invalid_stamps=r['invalid_stamps'],
                                 latency_mean_s=statistics.mean(latency) if latency else None,
                                 latency_max_s=max(latency) if latency else None,
                                 source_age_s=source_age, receive_age_s=receive_age)
        mean = statistics.mean(self.norms) if self.norms else None
        std = statistics.pstdev(self.norms) if len(self.norms) > 1 else None
        full_window = len(self.norms) >= self.imu_required
        stationary = bool(full_window and self.imu_min <= mean <= self.imu_max
                          and std < self.imu_std_max)
        recording = self.motion_active and not require_stationary
        initialized = bool(full_window and
                           (self.motion_imu_verified if recording else stationary))
        reasons = [name+':invalid_or_stale' for name in stale]
        if not full_window:
            reasons.append('imu:insufficient_valid_samples')
        if recording:
            if not self.motion_imu_verified:
                reasons.append('imu:stationary_recheck_required')
        elif full_window:
            if not self.imu_min <= mean <= self.imu_max:
                reasons.append('imu:stationary_mean_out_of_range')
            if not std < self.imu_std_max:
                reasons.append('imu:not_stationary')
        return dict(ready=bool(initialized and not stale), stale_streams=stale,
                    initialized=initialized, imu_norm_mean_mps2=mean,
                    imu_norm_std_mps2=std, imu_sample_count=len(self.norms), streams=streams,
                    imu_stationary_ready=stationary,
                    health_phase='recording' if recording else 'stationary_check',
                    health_reasons=reasons)


def loop_metrics(poses):
    """Rows: source stamp, x, y, yaw. Global endpoint error is not a return confirmation."""
    if len(poses) < 2 or any(len(p) != 4 or not all(math.isfinite(v) for v in p) for p in poses):
        raise ValueError('Need at least two finite timestamped poses')
    if any(b[0] <= a[0] for a, b in zip(poses, poses[1:])):
        raise ValueError('Odometry stamps must increase')
    a, b = poses[0], poses[-1]
    return dict(path_length_m=sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a, b in zip(poses, poses[1:])),
                endpoint_separation_m=math.hypot(b[1]-a[1], b[2]-a[2]),
                endpoint_yaw_difference_rad=abs(math.atan2(math.sin(b[3]-a[3]), math.cos(b[3]-a[3]))),
                odom_max_gap_s=max(b[0]-a[0] for a, b in zip(poses, poses[1:])))


def summarize_trials(trials):
    medians = {}
    for key in ('path_length_m', 'position_drift_m', 'yaw_drift_rad', 'duration_s'):
        values = [float(t[key]) for t in trials if isinstance(t.get(key), (int, float))
                  and not isinstance(t[key], bool) and math.isfinite(t[key])]
        medians[key] = statistics.median(values) if values else None
    return dict(trial_count=len(trials), success_count=sum(bool(t.get('success')) for t in trials),
                all_pass=len(trials) == 3 and all(t.get('success', False) for t in trials),
                median=medians, raw=trials)


def validate_pcd(path):
    """Read every ASCII PCD XYZ; reject empty, truncated, nonfinite, or wrong-count data."""
    path = Path(path).expanduser()
    with path.open('r', encoding='ascii') as handle:
        header = {}
        for line in handle:
            parts = line.strip().split()
            if not parts or parts[0].startswith('#'):
                continue
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == 'DATA':
                break
        if header.get('DATA') != ['ascii']:
            raise ValueError('PCD must contain DATA ascii')
        fields = header.get('FIELDS', [])
        counts = list(map(int, header.get('COUNT', ['1']*len(fields))))
        declared = int(header.get('POINTS', ['0'])[0])
        if declared <= 0 or len(counts) != len(fields) or any(n < 1 for n in counts):
            raise ValueError('Invalid PCD POINTS/COUNT')
        if not all(axis in fields and counts[fields.index(axis)] == 1 for axis in ('x', 'y', 'z')):
            raise ValueError('PCD needs scalar x, y, z')
        columns = sum(counts)
        xyz_indices = [sum(counts[:fields.index(axis)]) for axis in ('x', 'y', 'z')]
        actual = 0
        for line in handle:
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            parts = line.split()
            if len(parts) != columns or not all(math.isfinite(float(parts[i])) for i in xyz_indices):
                raise ValueError('PCD has invalid XYZ or column count')
            actual += 1
        if actual != declared:
            raise ValueError('PCD point count differs from POINTS header')
    return dict(pcd=str(path.resolve()), point_count=actual, format='ascii', reload_valid=True)
