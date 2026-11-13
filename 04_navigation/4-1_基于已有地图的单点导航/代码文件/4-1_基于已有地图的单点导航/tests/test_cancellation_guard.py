import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_1.safety import CancellationGuard


def test_accepted_cancel_still_requires_terminal_and_timeout_latches_stop():
    guard = CancellationGuard(10.0)
    guard.request(100.0)
    assert guard.needs_emergency(False)
    assert guard.cancel_response(0, True)
    assert guard.needs_stop(False)
    assert not guard.terminal_confirmed
    guard.watchdog(110.1)
    assert guard.emergency_required
    assert 'cancel_terminal_timeout' in guard.issues
    assert not guard.terminal_confirmed
    assert guard.needs_stop(True)
    assert guard.terminal(5)
    assert guard.needs_stop(True)


def test_rejected_or_unlisted_cancel_escalates_without_claiming_terminal():
    for code, listed in [(1, True), (0, False)]:
        guard = CancellationGuard(10)
        guard.request(0)
        assert not guard.cancel_response(code, listed)
        assert guard.emergency_required and not guard.terminal_confirmed
        assert not guard.terminal(2)


def test_finished_experiment_keeps_stop_output_even_without_active_goal():
    guard = CancellationGuard(10)
    assert not guard.needs_stop(False)
    assert guard.needs_stop(True)
    assert guard.needs_emergency(True)
