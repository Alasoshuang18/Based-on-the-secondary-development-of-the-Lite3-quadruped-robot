import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
pytest.importorskip('launch_ros')
from launch import LaunchContext
from launch_ros.actions import Node
from launch.utilities import perform_substitutions,normalize_to_list_of_substitutions
from lite3_lab_5_2.lean_mapping import prune_legacy_consumers


def test_lean_mapping_keeps_sensors_and_slam_but_removes_unused_legacy_consumers():
    context=LaunchContext()
    def action(name):
        return Node(package='fixture_package',executable=name)
    sensor=action('sensor_adapter')
    lio=action('lio_starter')
    slam=action('async_slam_toolbox_node')
    archive=action('map_archive')
    result=prune_legacy_consumers([sensor,lio,slam,archive,action('velocity_adapter'),
                                   action('continuous_adapter'),action('experiment')],context,'/tmp/fixture')
    assert result[:4]==[sensor,lio,slam,archive]
    names=[perform_substitutions(context,normalize_to_list_of_substitutions(node.node_executable)) for node in result]
    assert names==['sensor_adapter','lio_starter','async_slam_toolbox_node','map_archive','session_registry']


def test_changed_shared_bringup_is_rejected_instead_of_silently_losing_session_registry():
    with pytest.raises(RuntimeError,match='exactly one'):
        prune_legacy_consumers([],LaunchContext(),'/tmp/fixture')
