from pathlib import Path
import importlib.util
import pytest

path = Path(__file__).parents[1]/'lite3_lab_4_1/startup_guard.py'
spec = importlib.util.spec_from_file_location('guard', path)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)
WS = '/home/test/catkin_ws_lite3dog'


def process(pid, executable, domain=43):
    return {'pid': pid, 'argv': [executable], 'domain': domain}


def test_clean_start():
    guard.check_real_runtime([], 43, True, WS)


def test_parent_killed_but_lio_still_present():
    with pytest.raises(RuntimeError, match='PID=12'):
        guard.check_real_runtime([process(12, '/drivers/fastlio_mapping')], 43, True, WS)


def test_python_adapter_rejected():
    row = {'pid': 13, 'argv': ['/usr/bin/python3', WS+'/install/common/lib/common/sensor_adapter'], 'domain': 43}
    with pytest.raises(RuntimeError, match='PID=13'):
        guard.check_real_runtime([row], 43, True, WS)


def test_other_domain_lio_untouched():
    guard.check_real_runtime([process(14, '/drivers/fastlio_mapping', 44)], 43, True, WS)


def test_one_driver_reuse():
    guard.check_real_runtime([process(15, '/drivers/livox_ros_driver2_node')], 43, False, WS)


@pytest.mark.parametrize('rows', [[], [process(16, '/drivers/livox_ros_driver2_node'), process(17, '/drivers/livox_ros_driver2_node')], [process(18, '/drivers/livox_ros_driver2_node', 44)]])
def test_unverified_driver_reuse_rejected(rows):
    with pytest.raises(RuntimeError, match='start_hardware=false'):
        guard.check_real_runtime(rows, 43, False, WS)


def test_second_driver_rejected():
    with pytest.raises(RuntimeError, match='Livox'):
        guard.check_real_runtime([process(19, '/drivers/livox_ros_driver2_node')], 43, True, WS)


def test_replay_publisher_rejected():
    row={'pid':20,'argv':['/usr/bin/python3','/opt/ros/humble/bin/ros2','bag','play','test_bag'],'domain':43}
    with pytest.raises(RuntimeError,match='PID=20'):
        guard.check_real_runtime([row],43,True,WS)
