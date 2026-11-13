"""TF查找成功后，外参证据必须可直接写成严格JSON。"""
import json
import math
import numpy as np
import pytest
from lite3_lab_3_2 import algorithms


def test_matching_tf_evidence_survives_standard_json_round_trip():
    row = {'translation_m': [.22, 0., .175], 'quaternion_xyzw': [0., 0., 0., 1.]}
    row.update(algorithms.compare_extrinsics(row['translation_m'], row['quaternion_xyzw'],
               np.array([.22, 0., .175]), np.array([0., 0., 0., -1.]),
               np.float64(.05), np.float64(5.)))
    evidence = json.loads(json.dumps({'base_link<-lidar_link': row}, allow_nan=False))
    actual = evidence['base_link<-lidar_link']
    assert actual['matches_expected'] is True
    assert actual['translation_error_m'] == pytest.approx(0.)
    assert actual['rotation_error_deg'] == pytest.approx(0.)


def test_mismatching_tf_keeps_measured_errors_and_false_boolean_in_json():
    actual = algorithms.compare_extrinsics([.03, .04, 0.], [0., 0., math.sqrt(.5), math.sqrt(.5)],
               [0., 0., 0.], [0., 0., 0., 1.], .1, 5.)
    evidence = json.loads(json.dumps(actual, allow_nan=False))
    assert evidence['matches_expected'] is False
    assert evidence['translation_error_m'] == pytest.approx(.05)
    assert evidence['rotation_error_deg'] == pytest.approx(90.)
