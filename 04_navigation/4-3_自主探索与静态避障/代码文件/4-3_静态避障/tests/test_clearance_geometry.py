import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_3.clearance import rectangle, polygon_clearance, ClearanceAudit


def test_distance_uses_entity_surface_and_foot_envelope():
    dog = rectangle(0, 0, 0, 1.2, 0.8)
    obstacle = rectangle(1.0, 0, 0, 0.4, 0.4)
    assert polygon_clearance(dog, obstacle) == pytest.approx(0.2)
    assert polygon_clearance(dog, rectangle(0.7, 0, 0, 0.4, 0.4)) == 0


def test_rotated_polygon_and_containment_count_as_collision():
    assert polygon_clearance(rectangle(0, 0, 0, 3, 3), rectangle(0, 0, 0.7, 0.1, 0.1)) == 0
    assert polygon_clearance(rectangle(0, 0, 0.5, 1, 1), rectangle(4, 0, 0.4, 1, 1)) > 2


def test_global_minimum_cannot_be_hidden_by_median():
    audit = ClearanceAudit(0.1, True, True)
    for value in [0.4] * 100 + [0.02]:
        audit.add(value, 0.01)
    result = audit.result()
    assert result['minimum_clearance_m'] == 0.02
    assert result['sampled_threshold_pass'] is False
    assert result['continuous_lower_bound_m'] < 0.02
    assert not ClearanceAudit(0.1, False, True).result()['acceptance_pass']


def test_start_and_end_unsampled_motion_can_fail_clearance():
    for edge in ('start', 'end'):
        audit = ClearanceAudit(0.1, True, True)
        audit.add(0.11, 0.0)
        audit.add_endpoint_gap(0.11, 0.1, 0.5, 0.15, edge)
        result = audit.result()
        assert result['minimum_clearance_m'] == pytest.approx(0.11)
        assert result['continuous_lower_bound_m'] == pytest.approx(0.06)
        assert result['acceptance_evaluated']
        assert not result['acceptance_pass']


def test_endpoint_gap_over_limit_is_unevaluated_even_with_large_clearance():
    audit = ClearanceAudit(0.1, True, True)
    audit.add(2.0, 0.0)
    audit.add_endpoint_gap(2.0, 0.151, 0.5, 0.15, 'end')
    result = audit.result()
    assert not result['coverage_valid']
    assert not result['acceptance_evaluated']
