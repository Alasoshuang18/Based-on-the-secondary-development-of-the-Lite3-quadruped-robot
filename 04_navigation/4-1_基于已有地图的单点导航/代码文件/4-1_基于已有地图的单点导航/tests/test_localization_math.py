import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_1.geometry import weighted_region, StableGate, map_health


def test_weighted_region_excludes_less_than_five_percent_outlier():
    result = weighted_region([[0, 0], [0.1, 0], [100, 0]], [0.48, 0.48, 0.04])
    assert result['diameter_m'] == pytest.approx(0.1)
    assert result['covered_weight'] >= 0.95


def test_region_rejects_bad_weights_and_reports_real_spread():
    with pytest.raises(ValueError):
        weighted_region([[0, 0]], [0])
    with pytest.raises(ValueError):
        weighted_region([[0, 0]], [-1])
    assert weighted_region([[0, 0], [0.31, 0]], [1, 1])['diameter_m'] > 0.3


def test_stability_resets_on_bad_data_or_clock_rewind():
    gate = StableGate(3.0)
    assert not gate.update(True, 5.0)
    assert not gate.update(True, 7.9)
    assert gate.update(True, 8.0)
    assert not gate.update(False, 9.0)
    assert not gate.update(True, 10.0)
    assert not gate.update(True, 2.0)


def test_map_health_rejects_all_unknown_and_wrong_resolution():
    assert not map_health([-1] * 100, 10, 10, 0.05)['healthy']
    assert not map_health([0] * 100, 10, 10, 0.1)['healthy']
    assert map_health([0] * 90 + [100] * 10, 10, 10, 0.05)['healthy']


def test_weight_normalization_does_not_overflow():
    region = weighted_region([[0, 0], [0.1, 0]], [1e308, 1e308])
    assert region['covered_weight'] == pytest.approx(1.0)
    assert region['diameter_m'] == pytest.approx(0.1)
