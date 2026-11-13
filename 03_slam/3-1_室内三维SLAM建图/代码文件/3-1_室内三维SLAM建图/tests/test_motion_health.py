"""Stationary admission and runtime data health are separate recording checks."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_4_1.core import StreamMonitor


def feed(monitor, values, start=10.0):
    for index, value in enumerate(values):
        now = start + index * .005
        assert monitor.observe('imu', now, now, wall=now)
        assert monitor.imu((0, 0, value))
    for stream in ('cloud', 'odom', 'map'):
        assert monitor.observe(stream, now, now, wall=now)
    return now


def admitted():
    monitor = StreamMonitor()
    now = feed(monitor, [9.81] * 100)
    assert monitor.begin_motion(now, wall=now)
    assert monitor.motion_initial_health['health_phase'] == 'stationary_check'
    assert monitor.motion_initial_health['ready']
    return monitor


@pytest.mark.parametrize('values', [[9.81] * 99, [1.] * 100, [96.] * 100, [5.8, 18.2] * 50])
def test_motion_never_skips_stationary_admission(values):
    monitor = StreamMonitor()
    now = feed(monitor, values)
    assert not monitor.begin_motion(now, wall=now)
    assert not monitor.motion_active
    assert not monitor.motion_imu_verified
    assert monitor.motion_initial_health is None


def test_all_four_streams_are_required_to_begin():
    monitor = StreamMonitor()
    now = feed(monitor, [9.81] * 100)
    monitor.records['map']['valid_latest'] = False
    assert not monitor.begin_motion(now, wall=now)


def test_observed_gait_mean_and_variance_do_not_imply_stream_failure():
    monitor = admitted()
    now = feed(monitor, [5.8, 18.2] * 50, start=11.)
    health = monitor.health(now, wall=now)
    assert health['imu_norm_mean_mps2'] > monitor.imu_max
    assert health['imu_norm_std_mps2'] > monitor.imu_std_max
    assert health['ready'] and health['initialized']
    assert not health['imu_stationary_ready']
    assert health['health_phase'] == 'recording'
    assert not health['health_reasons']
    assert not monitor.begin_motion(now, wall=now)


def test_closing_and_next_admission_still_require_stationary_si_check():
    monitor = admitted()
    now = feed(monitor, [5.8, 18.2] * 50, start=11.)
    closing = monitor.health(now, wall=now, require_stationary=True)
    assert not closing['ready']
    assert 'imu:not_stationary' in closing['health_reasons']
    assert 'imu:stationary_mean_out_of_range' in closing['health_reasons']
    monitor.end_motion()
    assert not monitor.health(now, wall=now)['ready']
    assert not monitor.begin_motion(now, wall=now)
    now = feed(monitor, [9.81] * 100, start=12.)
    assert monitor.health(now, wall=now)['ready']
    assert monitor.begin_motion(now, wall=now)


@pytest.mark.parametrize('stream', StreamMonitor.STREAMS)
def test_runtime_does_not_relax_source_or_receive_freshness(stream):
    monitor = admitted()
    now = feed(monitor, [5.8, 18.2] * 50, start=11.)
    monitor.records[stream]['stamp'] = now - 1.01
    health = monitor.health(now, wall=now)
    assert not health['ready']
    assert stream in health['stale_streams']
    assert stream + ':invalid_or_stale' in health['health_reasons']
    monitor.records[stream]['stamp'] = now
    monitor.records[stream]['wall'] = now - 1.01
    assert not monitor.health(now, wall=now)['ready']


@pytest.mark.parametrize('stamp', [10.495, 10., 0., float('nan'), float('inf'), 100.])
def test_runtime_keeps_duplicate_backwards_future_stamp_rejection(stamp):
    monitor = admitted()
    assert not monitor.observe('odom', stamp, 11., wall=11.)
    assert not monitor.health(11., wall=11.)['ready']


@pytest.mark.parametrize('acceleration', [(0., 0., math.nan), (0., 0., math.inf), (0., 0.)])
def test_invalid_acceleration_revokes_trial_initialization(acceleration):
    monitor = admitted()
    assert not monitor.imu(acceleration)
    now = feed(monitor, [9.81] * 100, start=11.)
    health = monitor.health(now, wall=now)
    assert not health['ready']
    assert health['imu_stationary_ready']
    assert 'imu:stationary_recheck_required' in health['health_reasons']
    assert not monitor.begin_motion(now, wall=now)
    monitor.end_motion()
    assert monitor.begin_motion(now, wall=now)


def test_invalid_angular_velocity_uses_same_revocation_path():
    monitor = admitted()
    monitor.invalidate_imu()
    assert not monitor.motion_imu_verified
    assert not monitor.norms
    health = monitor.health(10.495, wall=10.495)
    assert not health['ready']
    assert 'imu:insufficient_valid_samples' in health['health_reasons']


@pytest.mark.parametrize('scale', [1., 96.])
def test_wrong_units_still_prevent_closing_a_trial(scale):
    monitor = admitted()
    now = feed(monitor, [scale] * 100, start=11.)
    health = monitor.health(now, wall=now, require_stationary=True)
    assert not health['ready']
    assert 'imu:stationary_mean_out_of_range' in health['health_reasons']
