"""5-1入口：等待真实健康条件，再发送单目标。"""
from .runtime import NavigationExperiment, run_node


def main(args=None):
    run_node(lambda: NavigationExperiment('lite3_lab_5_1'), args)
