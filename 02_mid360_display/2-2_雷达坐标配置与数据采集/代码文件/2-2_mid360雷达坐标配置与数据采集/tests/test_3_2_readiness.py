"""慢RTF录包前置条件：原始采样、接收租约与停钟保护相互独立。"""
import math
import pytest
from lite3_lab_3_2 import algorithms


def gate(stamp=10., received=100.):
    ready = algorithms.RecordingReadiness()
    ready.observe('cloud', stamp, received)
    ready.observe('imu', stamp, received)
    ready.observe_clock(10., 100.)  # 必须实际收到clock推进，不能只看到非零clock。
    ready.status('sim', 10., 100.)
    return ready


def test_slow_rtf_raw_frames_stay_ready_through_two_wall_second_wait():
    ready = gate()
    for wall in [100.5, 101., 101.5, 102., 102.5, 103.]:
        ready.observe_clock(10.+(wall-100.)*.08, wall)
        status = ready.status('sim', 10.+(wall-100.)*.08, wall)
        assert status['ready']
    assert status['streams']['cloud']['wall_age_s'] == 3.
    assert status['streams']['cloud']['ros_age_s'] == pytest.approx(.24)


def test_stopped_clock_cannot_start_a_recording():
    ready = gate()
    assert not ready.status('sim', 10., 100.1)['ready']
    ready.observe_clock(10.04, 100.5)
    assert ready.status('sim', 10.04, 100.5)['ready']
    status = ready.status('sim', 10.04, 101.01)
    assert not status['ready'] and status['reason'] == 'clock_stalled'


def test_recording_tick_does_not_renew_a_previously_received_clock_progress():
    ready = gate()
    ready.observe_clock(10.04, 100.5)
    assert ready.status('sim', 10.04, 100.5)['ready']
    ready.observe_clock(10.05, 100.51)  # 随后暂停；最近真实推进为100.51墙秒。
    status = ready.status('sim', 10.05, 101.1)
    assert not status['ready'] and status['reason'] == 'clock_stalled'


@pytest.mark.parametrize('stamp,received,ros_now,wall_now,reason', [
    (8., 100., 10.1, 100.5, 'ros_stale'),
    (11., 100., 10.1, 100.5, 'future_stamp'),
    (0., 100., 10.1, 100.5, 'invalid_stamp'),
    (math.nan, 100., 10.1, 100.5, 'invalid_stamp'),
    (10., 89., 10.1, 100.5, 'wall_hard_limit'),
    (10., 101., 10.1, 100.5, 'wall_rewound'),
])
def test_sim_rejects_old_future_invalid_and_wall_expired_input(stamp, received, ros_now, wall_now, reason):
    ready = gate(stamp, received)
    ready.observe_clock(ros_now, wall_now)
    status = ready.status('sim', ros_now, wall_now)
    assert not status['ready'] and status['streams']['cloud']['reason'] == reason


def test_real_keeps_one_wall_second_lease_without_requiring_sim_clock():
    ready = gate()
    assert ready.status('real', 10., 100.9)['ready']
    assert not ready.status('real', 10., 101.01)['ready']


def test_missing_input_bag_mode_and_rewound_sim_clock_cannot_start_recording():
    ready = algorithms.RecordingReadiness()
    assert not ready.status('sim', 1., 1.)['ready']
    ready = gate()
    assert not ready.status('bag', 10.1, 100.5)['ready']
    ready.observe_clock(9., 101.)
    assert not ready.status('sim', 9., 101.)['ready']
    ready.observe_clock(11., 101.5)
    assert not ready.status('sim', 11., 101.5)['ready']  # 时钟倒退锁存，需重启核查。
