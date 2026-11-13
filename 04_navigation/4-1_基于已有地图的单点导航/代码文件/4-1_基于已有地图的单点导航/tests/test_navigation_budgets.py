"""检查默认任务满足运动时间的必要条件，不把可行预算当作导航成功。"""
import ast
import math
from pathlib import Path

import pytest
import yaml


CHAPTER = Path(__file__).resolve().parents[2]
COMMON = CHAPTER.parent / 'lite3_course_common' / 'lite3_course_common' / 'bringup.py'
LESSONS = sorted(CHAPTER.glob('5-*/config/params.yaml'))


def motion_limits():
    # 读取公共Nav2的实际覆盖值，速度变化时不沿用过时的测试常量；无需ROS导入。
    settings = {}
    for node in ast.walk(ast.parse(COMMON.read_text(encoding='utf-8'))):
        if isinstance(node, ast.keyword) and node.arg in (
                'max_vel_x', 'max_speed_xy', 'max_velocity', 'xy_goal_tolerance'):
            settings[node.arg] = ast.literal_eval(node.value)
    speed = min(settings['max_vel_x'], settings['max_speed_xy'], settings['max_velocity'][0])
    tolerance = settings['xy_goal_tolerance']
    assert math.isfinite(speed) and speed > 0
    assert math.isfinite(tolerance) and tolerance >= 0
    return speed, tolerance


def assert_goal_budget(parameters, destinations, *, waypoint_tolerance=False):
    speed, tolerance = motion_limits()
    previous = (parameters['initial_x'], parameters['initial_y'])
    previous_tolerance = 0.0
    for destination in destinations:
        distance = math.hypot(destination[0] - previous[0], destination[1] - previous[1])
        minimum_sim_s = max(0.0, distance - previous_tolerance - tolerance) / speed
        assert parameters['goal_timeout_sim_s'] >= minimum_sim_s, (
            f'{previous}->{destination}: straight-line lower bound {minimum_sim_s:.2f}s '
            f'exceeds goal budget {parameters["goal_timeout_sim_s"]}s; detours need more time')
        previous = destination
        previous_tolerance = tolerance if waypoint_tolerance else 0.0
    assert parameters['goal_timeout_wall_s'] >= parameters['goal_timeout_sim_s']


@pytest.mark.parametrize('configuration', LESSONS, ids=lambda path: path.parents[1].name)
def test_default_goal_budgets_cover_straight_line_lower_bound(configuration):
    parameters = yaml.safe_load(configuration.read_text(encoding='utf-8'))['/**']['ros__parameters']
    waypoints_file = configuration.with_name('waypoints.yaml')
    if waypoints_file.exists():
        points = yaml.safe_load(waypoints_file.read_text(encoding='utf-8'))['waypoints']
        destinations = [(point['x'], point['y']) for point in points] * parameters['trials']
        assert_goal_budget(parameters, destinations, waypoint_tolerance=True)
    else:
        assert_goal_budget(parameters, [(parameters['goal_x'], parameters['goal_y'])])


def test_runtime_fallback_goal_budget_also_covers_distance_lower_bound():
    source = Path(__file__).resolve().parents[1] / 'lite3_lab_5_1' / 'runtime.py'
    defaults = next(ast.literal_eval(node.value) for node in ast.walk(ast.parse(source.read_text(encoding='utf-8')))
                    if isinstance(node, ast.Assign) and isinstance(node.value, ast.Dict)
                    and any(isinstance(target, ast.Name) and target.id == 'defaults' for target in node.targets))
    assert_goal_budget(defaults, [(defaults['goal_x'], defaults['goal_y'])])
