"""逐个发送NavigateToPose，明确停留、取消及失败策略。"""
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
import yaml

from lite3_lab_5_1.runtime import NavigationExperiment, run_node
from .policy import PatrolSequence, validate_waypoints


class PatrolExperiment(NavigationExperiment):
    def __init__(self):
        default_file = str(Path(get_package_share_directory('lite3_lab_5_2')) / 'config' / 'waypoints.yaml')
        super().__init__('lite3_lab_5_2', {'waypoints_file': default_file, 'trials': 3, 'failure_policy': 'abort'})
        self.sequence = None
        try:
            with Path(self.p('waypoints_file')).expanduser().open(encoding='utf-8') as stream:
                values = yaml.safe_load(stream)
            self.sequence = PatrolSequence(validate_waypoints(values['waypoints']), self.p('trials'), self.p('failure_policy'))
            self.event('patrol_loaded', points=self.sequence.waypoints, trials=self.sequence.trials,
                       failure_policy=self.sequence.policy)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.finish('invalid_patrol_configuration', error=str(exc))

    def send_current(self):
        point = self.sequence.current()
        self.event('patrol_progress', trial=self.sequence.trial, total_trials=self.sequence.trials,
                   waypoint=self.sequence.index + 1, total_waypoints=len(self.sequence.waypoints), name=point['name'])
        self.send_goal(self.goal_pose(point['x'], point['y'], point['yaw']))

    def begin_experiment(self):
        self.send_current()

    def navigation_result(self, status, label):
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.sequence.arrive(self.sim_now())
            self.event('patrol_dwell_started', until_sim_s=self.sequence.dwell_until)
        else:
            self.sequence.fail(label, self.sim_now())
            self.save_trials()
            if self.sequence.done:
                self.finish('patrol_failed_' + label)
            else:
                self.send_current()

    def experiment_tick(self):
        if self.cancel_requested or self.sequence is None:
            return
        if self.sequence.advance_if_due(self.sim_now()):
            self.save_trials()
            if self.sequence.done:
                self.finish('patrol_complete', successful=self.sequence.success)
            else:
                self.send_current()

    def save_trials(self):
        import json
        if self.sequence is None:
            return
        for trial in range(1, self.sequence.trials + 1):
            rows = [r for r in self.sequence.records if r['trial'] == trial]
            trial_complete = self.sequence.trial > trial
            (self.result_dir / 'trials' / ('trial_%02d.json' % trial)).write_text(
                json.dumps({'trial': trial, 'waypoint_results': rows,
                            'complete': trial_complete,
                            'all_waypoints_succeeded': trial_complete and bool(rows) and len(rows) == len(self.sequence.waypoints)
                            and all(r['status'] == 'succeeded' and r.get('dwell_completed', False) for r in rows)}, ensure_ascii=False, indent=2), encoding='utf-8')

    def summary_extra(self):
        self.save_trials()
        return {'patrol_records': [] if self.sequence is None else self.sequence.records,
                'required_trials': self.p('trials'), 'failure_policy': self.p('failure_policy')}


def main(args=None):
    run_node(PatrolExperiment, args)
