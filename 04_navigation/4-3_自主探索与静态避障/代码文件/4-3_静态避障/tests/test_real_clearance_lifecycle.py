import ast
import sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'5-1_single_goal_navigation'))
from lite3_lab_5_3.clearance import ClearanceAudit


def test_stale_costmap_at_action_completion_cannot_pass_clearance_acceptance():
    path=Path(__file__).resolve().parents[1]/'lite3_lab_5_3/experiment.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    module=ast.parse('')
    module.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='summary_extra']
    namespace={'time':NS(monotonic=lambda:200.1)}
    from lite3_lab_5_1.safety import fresh_source
    namespace['fresh_source']=fresh_source
    exec(compile(ast.fix_missing_locations(module),str(path),'exec'),namespace)
    audit=ClearanceAudit(.1,True,True); audit.add(1.,0.)
    inflation=ClearanceAudit(.1,True,True); inflation.add(1.,0.)
    params=dict(max_sensor_gap_s=.15,obstacle_topic='/course/obstacles',robot_length_m=1.2,
        robot_width_m=.8,sensor_error_bound_m=.15,footprint_padding_m=.15,
        costmap_inflation_radius_m=.9,costmap_cost_scaling_factor=3.,max_costmap_gap_s=1.)
    node=NS(clearance_audit=audit,last_clearance_stamp=100.,last_clearance_value=1.,started=True,
        sim_now=lambda:100.1,motion_rate_bound=lambda:.04,p=params.__getitem__,navigation_success=True,
        footprint_valid=True,measurement_failures=[],costmap_snapshot={},
        sensor_stamp=100.,sensor_wall=200.,footprint_stamp=100.,footprint_wall=200.,
        costmap_stamp=90.,costmap_wall=190.)
    node.inflation_audit=inflation; node.last_inflation_stamp=100.; node.last_inflation_clearance=1.
    result=namespace['summary_extra'](node)
    assert not result['clearance_audit']['acceptance_pass']
    assert not result['clearance_audit']['acceptance_evaluated']
