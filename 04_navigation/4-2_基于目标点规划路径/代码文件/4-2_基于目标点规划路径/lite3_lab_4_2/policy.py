"""巡检状态机：到达后停留，不把跳过的失败点记成成功。"""
import math


def validate_waypoints(entries):
    if not isinstance(entries, list) or not entries:
        raise ValueError('waypoints必须为非空列表')
    result = []
    for index, point in enumerate(entries):
        if not isinstance(point, dict):
            raise ValueError('每个waypoint必须为键值配置')
        item = {'name': str(point.get('name', 'point_%d' % (index + 1)))}
        for key, default in [('x', None), ('y', None), ('yaw', 0.0), ('dwell_s', 0.0)]:
            value = float(point.get(key, default))
            if not math.isfinite(value) or (key == 'dwell_s' and value < 0):
                raise ValueError('目标%d的%s无效' % (index, key))
            item[key] = value
        result.append(item)
    return result


class PatrolSequence:
    def __init__(self, waypoints, trials=3, failure_policy='abort'):
        if trials < 1 or failure_policy not in ('abort', 'skip'):
            raise ValueError('trials必须正整数，失败策略仅abort/skip')
        self.waypoints, self.trials, self.policy = waypoints, trials, failure_policy
        self.trial, self.index = 1, 0
        self.done, self.success = False, True
        self.dwell_until = None
        self.records = []

    def current(self):
        return self.waypoints[self.index]

    def arrive(self, now):
        self.records.append({'trial': self.trial, 'waypoint': self.index + 1,
                             'name': self.current()['name'], 'status': 'succeeded', 'sim_s': now,
                             'dwell_completed': False})
        self.dwell_until = now + self.current()['dwell_s']

    def _advance(self):
        self.index += 1
        if self.index >= len(self.waypoints):
            self.index = 0
            self.trial += 1
            if self.trial > self.trials:
                self.done = True
        self.dwell_until = None

    def advance_if_due(self, now):
        if self.dwell_until is None or now < self.dwell_until:
            return False
        self.records[-1]['dwell_completed'] = True
        self.records[-1]['departure_sim_s'] = now
        self._advance()
        return True

    def fail(self, status, now):
        self.success = False
        self.records.append({'trial': self.trial, 'waypoint': self.index + 1,
                             'name': self.current()['name'], 'status': status, 'sim_s': now})
        if self.policy == 'abort':
            self.done = True
        else:
            self._advance()
