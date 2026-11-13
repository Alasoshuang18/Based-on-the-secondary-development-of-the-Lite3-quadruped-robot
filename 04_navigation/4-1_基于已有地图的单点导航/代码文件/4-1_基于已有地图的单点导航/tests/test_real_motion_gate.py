import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_1.safety import real_motion_allowed, fresh_source


def test_site_measurement_and_explicit_motion_enable_are_both_required():
    assert not real_motion_allowed('real', False, True)
    assert not real_motion_allowed('real', True, False)
    assert real_motion_allowed('real', True, True)
    assert real_motion_allowed('sim', False, False)
    assert not real_motion_allowed('bag', True, True)


def test_real_sensor_leases_reject_future_old_and_duplicate_original_headers():
    assert fresh_source(100., 200., 100.2, 200.2, .5, .5)
    assert not fresh_source(100., 200., 100.6, 200.1, .5, .5)
    assert not fresh_source(100.3, 200., 100.2, 200.2, .5, .5)
    assert not fresh_source(100., 200., 100.2, 200.6, .5, .5)
    assert not fresh_source(100., 200., 100.2, 200.2, .5, .5, previous=100.)
