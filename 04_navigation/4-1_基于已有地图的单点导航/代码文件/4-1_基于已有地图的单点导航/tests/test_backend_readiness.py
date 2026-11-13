import importlib.util
from pathlib import Path

path=Path(__file__).parents[1]/'lite3_lab_5_1/safety.py'
spec=importlib.util.spec_from_file_location('backend_safety',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_navigation_waits_for_actual_backend_and_fresh_status():
    ready=module.backend_available
    assert not ready({'backend':'sim','gait_ready':False},'sim',10.,9.8,1.5)
    assert ready({'backend':'sim','gait_ready':True},'sim',10.,9.8,1.5)
    assert not ready({'backend':'sim','gait_ready':True},'sim',12.,9.8,1.5)
    assert not ready({'backend':'real','gait_ready':True},'sim',10.,9.8,1.5)
    assert not ready({'backend':'sim','gait_ready':'false'},'sim',10.,9.8,1.5)
    assert not ready({'backend':'sim','gait_ready':True,'backend_ready':False},'sim',10.,9.8,1.5)
    assert not ready({'backend':'bag','gait_ready':True},'bag',10.,9.8,1.5)
    assert ready({'backend':'real','gait_ready':False,'backend_ready':True},'real',10.,9.8,1.5)
