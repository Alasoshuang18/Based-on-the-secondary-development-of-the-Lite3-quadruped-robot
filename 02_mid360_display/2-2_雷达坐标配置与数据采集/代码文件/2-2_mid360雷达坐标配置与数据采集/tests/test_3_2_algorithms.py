import numpy as np
from lite3_lab_3_2.algorithms import recording_limits, quaternion_error

def test_recording_limits_prevent_unbounded_disk_and_ram():
    assert recording_limits(90,999999999)==(30.0,67108864)
    assert recording_limits(-1,-1)==(1.0,1048576)

def test_quaternion_sign_equivalence():
    assert quaternion_error([0,0,0,1],[0,0,0,-1])<1e-9
    assert abs(quaternion_error([0,0,0,1],[0,0,1,0])-np.pi)<1e-9


def test_simulation_recording_uses_the_same_clock_as_headers():
    from lite3_lab_3_2.algorithms import recording_clock_arguments
    assert recording_clock_arguments('sim')==['--use-sim-time']
    assert recording_clock_arguments('real')==[]
