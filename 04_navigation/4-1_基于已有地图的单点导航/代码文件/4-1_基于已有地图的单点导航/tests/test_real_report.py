import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_1.report import main


def test_real_report_draws_actual_estimates_without_fabricating_truth_error(tmp_path):
    (tmp_path/'summary.json').write_text(json.dumps({'package':'lite3_lab_5_1','mode':'real',
                                'termination_reason':'succeeded'}),encoding='utf-8')
    (tmp_path/'csv').mkdir()
    (tmp_path/'csv/estimated_trajectory.csv').write_text(
        'ros_unix_s,wall_monotonic_s,estimated_map_x,estimated_map_y,estimated_map_yaw\n'
        '100,200,0,0,0\n100.1,200.1,0.01,0,0\n',encoding='utf-8')
    (tmp_path/'csv/inflation_edge_clearance.csv').write_text(
        'ros_s,footprint_to_inflation_lower_bound_m,motion_bound_m\n100,0.3,0.02\n100.5,0.2,0.02\n',encoding='utf-8')
    main([str(tmp_path)])
    assert (tmp_path/'figures/estimated_trajectory.png').is_file()
    assert not (tmp_path/'figures/trajectory_and_actual_error.png').exists()
    assert (tmp_path/'figures/inflation_edge_clearance.png').is_file()
