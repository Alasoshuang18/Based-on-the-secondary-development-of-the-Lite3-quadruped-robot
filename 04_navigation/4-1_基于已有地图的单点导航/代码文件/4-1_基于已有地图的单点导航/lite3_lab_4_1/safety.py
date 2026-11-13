"""取消守卫：接受取消不等于动作终态；失败必须升级到锁存停车。"""
import math

def real_motion_allowed(mode, enabled, site_verified):
    """真机需现场配置核对和显式运动许可；bag始终只测量。"""
    return mode == 'sim' or (mode == 'real' and enabled is True and site_verified is True)


def fresh_source(stamp, received, now_ros, now_wall, max_header_s, max_wall_s, previous=None):
    """真实header和单调接收租约同时有效；重复/未来/逆序帧不可续租。"""
    return bool(all(v is not None and math.isfinite(v) for v in (stamp, received, now_ros, now_wall))
                and 0 <= now_ros-stamp <= max_header_s and 0 <= now_wall-received <= max_wall_s
                and (previous is None or stamp > previous))

def backend_available(status, mode, now_wall, received_wall, timeout=1.5):
    """候选输出超时不代表步态未就绪；检查实际认证与状态租约。"""
    if (mode not in ('sim','real') or not isinstance(status,dict) or status.get('backend') != mode
            or received_wall is None or not 0. <= now_wall-received_wall <= timeout):
        return False
    if mode == 'real': return status.get('backend_ready') is True
    return status.get('gait_ready') is True and status.get('backend_ready',True) is True


class CancellationGuard:
    def __init__(self, timeout_s=10.0):
        self.timeout_s = float(timeout_s)
        self.requested_wall = None
        self.cancel_accepted = False
        self.terminal_status = None
        self.emergency_required = False
        self.emergency_acknowledged = False
        self.issues = []

    @property
    def terminal_confirmed(self):
        return self.terminal_status in (4, 5, 6)

    def request(self, now_wall):
        if self.requested_wall is None:
            self.requested_wall = now_wall
        # Nav2与零候选可能竞相到达同一出口；所有取消直接请求锁存，不能靠候选竞争停车。
        self.emergency_required = True

    def fail(self, reason):
        self.emergency_required = True
        if reason not in self.issues:
            self.issues.append(reason)

    def cancel_response(self, return_code, target_listed):
        self.cancel_accepted = return_code == 0 and bool(target_listed)
        if not self.cancel_accepted:
            self.fail('cancel_rejected_or_target_not_listed')
        return self.cancel_accepted

    def terminal(self, status):
        if status not in (4, 5, 6):
            return False
        self.terminal_status = status
        return True

    def watchdog(self, now_wall):
        if (self.requested_wall is not None and not self.terminal_confirmed
                and now_wall - self.requested_wall >= self.timeout_s):
            self.fail('cancel_terminal_timeout')

    def needs_stop(self, finished):
        return bool(finished or self.requested_wall is not None or self.emergency_required)

    def needs_emergency(self, finished):
        return bool(finished or self.emergency_required)

    def snapshot(self):
        return {'cancel_requested_wall_s': self.requested_wall,
                'cancel_accepted': self.cancel_accepted, 'terminal_status': self.terminal_status,
                'terminal_confirmed': self.terminal_confirmed,
                'emergency_required': self.emergency_required,
                'emergency_stop_acknowledged': self.emergency_acknowledged,
                'issues': list(self.issues)}
