"""直接执行真实回调方法，移除ROS继承壳；时间/消息来自显式测试夹具。"""
import ast
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_1.safety import fresh_source


def method(name):
    path = Path(__file__).resolve().parents[1]/'lite3_lab_5_1/runtime.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    function = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)
    module = ast.parse('')
    module.body = [function]
    namespace = {'time': NS(monotonic=lambda:200.), 'math': math, 'fresh_source': fresh_source,
                 'weighted_region': lambda a,b: {'diameter_m': .1}}
    exec(compile(ast.fix_missing_locations(module), str(path), 'exec'), namespace)
    return namespace[name]


def test_real_particles_cannot_renew_convergence_with_repeated_original_header():
    node = NS(p=lambda key: {'mode':'real','map_frame':'map'}[key],
              particle_stamp=100., particle_wall=199.5, sim_now=lambda:100.1,
              event=lambda *a,**k:None, particle_report={'diameter_m':.1})
    message = NS(header=NS(stamp=NS(sec=100,nanosec=0),frame_id='map'),
                 particles=[NS(pose=NS(position=NS(x=0.,y=0.)),weight=1.)])
    method('on_particles')(node,message)
    assert node.particle_wall == 199.5


def test_real_plan_discards_old_original_time_instead_of_fabricating_response():
    calls=[]
    node = NS(finished=False, p=lambda k:{'mode':'real','map_frame':'map'}[k],
              latest_path_stamp=99., sim_now=lambda:100.1,
              event=lambda *a,**k:calls.append((a,k)), plan_observed=lambda p:calls.append(('accepted',p)))
    message = NS(header=NS(stamp=NS(sec=98,nanosec=0),frame_id='map'),poses=[])
    method('on_plan')(node,message)
    assert node.latest_path_stamp == 99.
    assert not any(call[0] == 'accepted' for call in calls)
