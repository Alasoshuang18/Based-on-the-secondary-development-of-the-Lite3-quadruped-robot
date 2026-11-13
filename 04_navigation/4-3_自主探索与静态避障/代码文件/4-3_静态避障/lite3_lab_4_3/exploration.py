"""Continue a restored SLAM map while retaining the original clearance audit."""
from lite3_lab_5_2.exploration import ExplorationExperiment, run_node
from .experiment import StaticAvoidanceExperiment


class StaticExploration(StaticAvoidanceExperiment, ExplorationExperiment):
    def begin_experiment(self):
        self.first_navigation_stamp = self.sim_now()
        ExplorationExperiment.begin_experiment(self)


def main(args=None):
    run_node(StaticExploration, args)
