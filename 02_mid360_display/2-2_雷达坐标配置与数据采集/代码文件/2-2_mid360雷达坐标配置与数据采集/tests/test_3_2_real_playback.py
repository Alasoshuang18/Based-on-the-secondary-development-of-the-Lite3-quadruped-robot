"""执行实际回放函数：真实bag需保留Livox逐点时间，且只由player生成clock。"""
import ast
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as Obj
import pytest


@pytest.mark.parametrize('source,has_custom', [('real', True), ('sim', False)])
def test_actual_playback_uses_real_custom_topic_and_one_generated_clock(tmp_path, monkeypatch, source, has_custom):
    launch_file=Path(__file__).resolve().parents[1]/'launch/experiment.launch.py'
    tree=ast.parse(launch_file.read_text(encoding='utf-8'))
    playback=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='playback')
    guard=ModuleType('lite3_course_common.runtime_guard')
    checks=[]
    guard.ensure_no_clock=lambda:checks.append(True)
    monkeypatch.setitem(sys.modules,'lite3_course_common.runtime_guard',guard)
    class Configuration:
        def __init__(self,name):self.name=name
        def perform(self,context):return context[self.name]
    scope=dict(Path=Path,LaunchConfiguration=Configuration,
               get_package_share_directory=lambda package:str(tmp_path),ExecuteProcess=lambda **kw:kw)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[playback],type_ignores=[])),str(launch_file),'exec'),scope)
    (tmp_path/'metadata.yaml').write_text('rosbag2_bagfile_information: {}',encoding='utf-8')
    result=scope['playback'](dict(mode='bag',play_bag='true',bag_path=str(tmp_path),playback_rate='1.0',bag_source=source))
    command=result[0]['cmd'];topics=command[command.index('--topics')+1:]
    assert checks==[True] and command.count('--clock')==1
    assert '--delay' not in command, 'Humble的--delay期间clock先推进，真正起播会回拨；不能据此初始化LIO'
    assert '/clock' not in topics
    assert ('/livox/lidar' in topics) is has_custom
    assert {'/course/points_raw','/course/imu','/tf','/tf_static'} <= set(topics)
