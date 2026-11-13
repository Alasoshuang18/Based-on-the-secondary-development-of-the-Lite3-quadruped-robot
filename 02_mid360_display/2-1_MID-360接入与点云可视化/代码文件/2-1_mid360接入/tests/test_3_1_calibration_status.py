"""TF连通诊断不能替代真实安装测量记录。"""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as Obj
import pytest


@pytest.mark.parametrize('measured,record_exists,expected', [
    (False, False, 'UNMEASURED'), (True, False, 'MISSING_RECORD'), (True, True, 'DECLARED_WITH_RECORD')])
def test_actual_status_distinguishes_connection_from_manual_calibration(tmp_path, measured, record_exists, expected):
    source=Path(__file__).resolve().parents[1]/'lite3_lab_3_1/experiment.py'
    cls=next(n for n in ast.parse(source.read_text(encoding='utf-8')).body if isinstance(n,ast.ClassDef))
    report=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='report')
    scope=dict(time=Obj(monotonic=lambda:100.),json=json,Path=Path,String=Obj,write_json=lambda *args:None)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[report],type_ignores=[])),str(source),'exec'),scope)
    record=tmp_path/'measured_mount.txt'
    if record_exists:record.write_text('现场测量记录夹具',encoding='utf-8')
    parameters=dict(stale_timeout_s=1.,calibration_measured=measured,calibration_record_path=str(record))
    sent=[]
    node=Obj(last_receive=100.,last_imu=100.,stats=dict(finite_points=1,tf_ready=True),received=1,
             frames={'lidar_link'},output=tmp_path,get_parameter=lambda name:Obj(value=parameters[name]),
             status_pub=Obj(publish=sent.append))
    scope['report'](node)
    data=json.loads(sent[0].data)
    assert data['healthy'] is True
    assert data['healthy_scope']=='pointcloud_connection_and_tf_only'
    assert data['calibration_status']==expected
    assert data['calibration_record_exists'] is record_exists
