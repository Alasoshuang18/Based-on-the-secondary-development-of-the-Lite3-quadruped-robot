import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_2.policy import PatrolSequence, validate_waypoints


def test_three_trials_keep_order_and_dwell():
    sequence = PatrolSequence(validate_waypoints([{'x': 1, 'y': 0, 'dwell_s': 2}, {'x': 0, 'y': 0}]), 3, 'abort')
    assert sequence.current()['x'] == 1
    sequence.arrive(10)
    assert not sequence.advance_if_due(11.9)
    assert sequence.advance_if_due(12)
    assert sequence.index == 1
    sequence.arrive(12)
    assert sequence.advance_if_due(12)
    assert sequence.trial == 2 and sequence.index == 0
    for now in (20, 30):
        sequence.arrive(now)
        sequence.advance_if_due(now + 2)
        sequence.arrive(now + 2)
        sequence.advance_if_due(now + 2)
    assert sequence.done and len(sequence.records) == 6


def test_abort_and_skip_are_explicit():
    points = validate_waypoints([{'x': 1, 'y': 0}, {'x': 2, 'y': 0}])
    abort = PatrolSequence(points, 3, 'abort')
    abort.fail('aborted', 4)
    assert abort.done and not abort.success
    skip = PatrolSequence(points, 3, 'skip')
    skip.fail('aborted', 4)
    assert skip.index == 1 and not skip.success
    with pytest.raises(ValueError):
        validate_waypoints([{'x': float('nan'), 'y': 0}])


def test_arrival_is_not_completed_dwell():
    sequence = PatrolSequence(validate_waypoints([{'x': 0, 'y': 0, 'dwell_s': 3}]), 3)
    sequence.arrive(10)
    assert sequence.records[-1]['dwell_completed'] is False
    assert not sequence.advance_if_due(12)
    assert sequence.advance_if_due(13)
    assert sequence.records[-1]['dwell_completed'] is True
    with pytest.raises(ValueError):
        validate_waypoints([3])
