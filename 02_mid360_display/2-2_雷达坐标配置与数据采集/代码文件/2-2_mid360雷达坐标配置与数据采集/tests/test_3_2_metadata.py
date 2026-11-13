"""验证元数据内容判据；这些构造字典不代表真实录包或回放结果。"""
import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from lite3_lab_3_2 import algorithms


def metadata(points=1, imu=20, duration_ns=200000000):
    counts = {'/course/points_raw': points, '/course/imu': imu, '/clock': 2}
    return {'rosbag2_bagfile_information': {
        'duration': {'nanoseconds': duration_ns}, 'message_count': sum(counts.values()),
        'topics_with_message_count': [
            {'topic_metadata': {'name': name}, 'message_count': count}
            for name, count in counts.items()]}}


def actual_validation(data, returncode=0):
    assert hasattr(algorithms, 'recording_metadata'), '缺少录包元数据内容校验'
    return algorithms.recording_metadata(data, returncode)


def test_pointcloud_and_imu_with_positive_duration_and_clean_exit_are_required():
    result = actual_validation(metadata())
    assert result['valid_recording']
    assert result['bag_duration_ns'] == 200000000
    assert result['bag_duration_s'] == pytest.approx(.2)
    assert result['topic_message_counts'] == {'/course/points_raw': 1, '/course/imu': 20, '/clock': 2}
    assert result['message_count'] == 23
    assert result['validation_errors'] == []


def test_existing_metadata_and_successful_process_with_no_cloud_is_failure():
    # 对应已观察到的失败条件：存在metadata、进程exit0、IMU有消息、点云为0。
    result = actual_validation(metadata(points=0, imu=7, duration_ns=72000000))
    assert not result['valid_recording']
    assert result['topic_message_counts']['/course/points_raw'] == 0
    assert 'no_pointcloud_messages' in result['validation_errors']


@pytest.mark.parametrize('data,returncode,error', [
    (metadata(imu=0), 0, 'no_imu_messages'),
    (metadata(duration_ns=0), 0, 'nonpositive_duration'),
    (metadata(), 1, 'recorder_exit_not_zero'),
])
def test_other_incomplete_recordings_are_explicit_failures(data, returncode, error):
    result = actual_validation(data, returncode)
    assert not result['valid_recording']
    assert error in result['validation_errors']


@pytest.mark.parametrize('data', [None, {}, {'rosbag2_bagfile_information': []},
                                  {'rosbag2_bagfile_information': {'duration': {'nanoseconds': 'invalid'}}}])
def test_missing_or_malformed_metadata_never_passes(data):
    result = actual_validation(data)
    assert not result['valid_recording']
    assert any(error.startswith('invalid_metadata:') for error in result['validation_errors'])


def test_invalid_counts_and_inconsistent_totals_never_pass():
    data = metadata()
    data['rosbag2_bagfile_information']['topics_with_message_count'][0]['message_count'] = -1
    assert not actual_validation(data)['valid_recording']
    data = metadata()
    data['rosbag2_bagfile_information']['message_count'] += 1
    assert 'message_count_mismatch' in actual_validation(data)['validation_errors']


def test_missing_cloud_topic_is_distinguished_from_any_other_topic():
    data = metadata()
    information = data['rosbag2_bagfile_information']
    information['topics_with_message_count'][0]['topic_metadata']['name'] = '/unrelated/cloud'
    result = actual_validation(data)
    assert not result['valid_recording']
    assert 'no_pointcloud_messages' in result['validation_errors']


@pytest.mark.parametrize('points', [0, 1])
def test_actual_stop_callback_writes_content_verdict_and_still_closes_owned_resources(tmp_path, points):
    source = Path(__file__).resolve().parents[1]/'lite3_lab_3_2/experiment.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    klass = next(item for item in tree.body if isinstance(item, ast.ClassDef))
    stop = next(item for item in klass.body if isinstance(item, ast.FunctionDef) and item.name == 'stop_recording')
    namespace = dict(yaml=yaml, recording_metadata=algorithms.recording_metadata,
                     time=SimpleNamespace(monotonic=lambda: 20.),
                     write_json=lambda path, data: path.write_text(json.dumps(data), encoding='utf-8'))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[stop], type_ignores=[])), str(source), 'exec'), namespace)
    bag = tmp_path/'raw_bag'
    bag.mkdir()
    (bag/'metadata.yaml').write_text(yaml.safe_dump(metadata(points=points)), encoding='utf-8')
    closed, errors = [], []
    node = SimpleNamespace(output=tmp_path, process=SimpleNamespace(poll=lambda: 0, returncode=0),
                           record_started=3., duration=30., cache=16777216, finished=False,
                           log=SimpleNamespace(close=lambda: closed.append(True)),
                           get_logger=lambda: SimpleNamespace(error=errors.append))
    namespace['stop_recording'](node)
    result = json.loads((tmp_path/'record_result.json').read_text(encoding='utf-8'))
    assert result['valid_recording'] is (points > 0)
    assert result['metadata_exists'] and result['returncode'] == 0
    assert result['elapsed_s'] == 17. and result['requested_duration_s'] == 30.
    assert result['cache_bytes'] == 16777216
    assert node.process is None and node.finished and node.log is None and closed == [True]
    assert bool(errors) is (points == 0)
