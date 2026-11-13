"""只读取已存在的真实CSV/JSON生成图；缺失数据不生成成功替代值。"""
import argparse
import csv
import json
from pathlib import Path


def read_rows(path):
    if not path.is_file():
        return []
    with path.open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def main(args=None):
    parser = argparse.ArgumentParser(description='由真实课程导航日志生成轨迹、误差和净空图')
    parser.add_argument('result_dir', type=Path)
    options = parser.parse_args(args)
    directory = options.result_dir.expanduser().resolve()
    summary = directory / 'summary.json'
    if not summary.is_file():
        parser.error('summary.json不存在；先结束实验并保留运行日志')
    with summary.open(encoding='utf-8') as stream:
        value = json.load(stream)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figures = directory / 'figures'
    figures.mkdir(exist_ok=True)
    generated = []
    clock_label = 'ROS system time (s)' if value.get('mode') == 'real' else 'simulation time (s)'
    rows = read_rows(directory / 'csv' / 'estimated_trajectory.csv')
    if rows:
        fig, axes = plt.subplots(figsize=(7, 5))
        axes.plot([float(r['estimated_map_x']) for r in rows], [float(r['estimated_map_y']) for r in rows],
                  label='estimated TF; absolute accuracy not evaluated')
        axes.set(xlabel='map x (m)', ylabel='map y (m)', aspect='equal')
        axes.legend()
        fig.tight_layout()
        fig.savefig(str(figures / 'estimated_trajectory.png'), dpi=180)
        plt.close(fig)
        generated.append('estimated_trajectory.png')
    rows = read_rows(directory / 'csv' / 'poses.csv')
    if rows:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        axes[0].plot([float(r['truth_x']) for r in rows], [float(r['truth_y']) for r in rows], label='ground truth (evaluation only)')
        axes[0].plot([float(r['estimate_x']) for r in rows], [float(r['estimate_y']) for r in rows], label='estimated TF')
        axes[0].set(xlabel='map x (m)', ylabel='map y (m)', aspect='equal')
        axes[0].legend()
        axes[1].plot([float(r['sim_s']) for r in rows], [float(r['actual_pose_error_m']) for r in rows])
        axes[1].set(xlabel=clock_label, ylabel='actual pose error (m)')
        fig.suptitle(value['package'] + ': ' + value['termination_reason'])
        fig.tight_layout()
        fig.savefig(str(figures / 'trajectory_and_actual_error.png'), dpi=180)
        plt.close(fig)
        generated.append('trajectory_and_actual_error.png')
    rows = read_rows(directory / 'csv' / 'clearance.csv')
    if rows:
        fig, axes = plt.subplots(figsize=(8, 4))
        x = [float(r['sim_s']) for r in rows]
        y = [float(r['surface_clearance_m']) for r in rows]
        axes.plot(x, y, label='all observed surface clearances')
        if 'motion_bound_m' in rows[0]:
            axes.plot(x, [float(r['surface_clearance_m'])-float(r['motion_bound_m']) for r in rows],
                      label='conservative continuous lower bound')
        axes.axhline(0.1, color='red', linestyle='--', label='required 0.1 m')
        index = min(range(len(y)), key=y.__getitem__)
        axes.scatter([x[index]], [y[index]], color='red', label='global minimum %.4f m' % y[index])
        axes.set(xlabel=clock_label, ylabel='physical envelope to observed surface (m)')
        axes.legend()
        fig.tight_layout()
        fig.savefig(str(figures / 'clearance_all_samples.png'), dpi=180)
        plt.close(fig)
        generated.append('clearance_all_samples.png')
    rows = read_rows(directory / 'csv' / 'inflation_edge_clearance.csv')
    if rows:
        fig, axes = plt.subplots(figsize=(8, 4))
        x = [float(r['ros_s']) for r in rows]
        y = [float(r['footprint_to_inflation_lower_bound_m'])-float(r['motion_bound_m']) for r in rows]
        axes.plot(x, y, label='published footprint to inflated/unknown cells; conservative bound')
        axes.axhline(.1, color='red', linestyle='--', label='required 0.1 m')
        index = min(range(len(y)), key=y.__getitem__)
        axes.scatter([x[index]], [y[index]], color='red', label='global minimum %.4f m' % y[index])
        axes.set(xlabel=clock_label, ylabel='planning contour to inflation edge (m)')
        axes.legend()
        fig.tight_layout()
        fig.savefig(str(figures / 'inflation_edge_clearance.png'), dpi=180)
        plt.close(fig)
        generated.append('inflation_edge_clearance.png')
    plans = []
    log = directory / 'raw' / 'events.jsonl'
    if log.is_file():
        with log.open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if row['event'] == 'planning_evaluation' and row.get('hausdorff_m') is not None:
                    plans.append(row)
    if plans:
        fig, axes = plt.subplots(figsize=(8, 4))
        for formal, label in ((True, 'formal Nav2 plan'), (False, 'diagnostic only')):
            selected = [r for r in plans if bool(r.get('formal_acceptance_evidence', False)) == formal]
            if selected:
                axes.plot([r['sim_s'] for r in selected], [r['hausdorff_m'] for r in selected], 'o-', label=label)
        axes.axhline(0.2, color='red', linestyle='--')
        axes.set(xlabel=clock_label, ylabel='symmetric Hausdorff (m)')
        axes.legend()
        fig.tight_layout()
        fig.savefig(str(figures / 'path_geometry_change.png'), dpi=180)
        plt.close(fig)
        generated.append('path_geometry_change.png')
    print(json.dumps({'result_dir': str(directory), 'generated_figures': generated,
                      'missing_data_is_not_success': not generated}, ensure_ascii=False, indent=2))
