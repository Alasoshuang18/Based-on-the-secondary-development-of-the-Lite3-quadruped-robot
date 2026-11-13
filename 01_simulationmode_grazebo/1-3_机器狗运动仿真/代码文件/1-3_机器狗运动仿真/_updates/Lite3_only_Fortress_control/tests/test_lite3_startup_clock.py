"""Exercise the real Lite3 constructor with ROS node/middleware substitutes.

These checks cover parameter startup and topic contracts, not an executor,
the /clock bridge or Gazebo physics.
"""
from contextlib import ExitStack
import types
import unittest
from unittest.mock import patch

import test_measured_node as measured_tests

implementation = measured_tests.implementation


class StubNode:
    force_system_time = False

    def __init__(self, name, namespace='', parameter_overrides=None):
        self.node_name, self.namespace = name, namespace
        self.parameters = {'use_sim_time': False}
        for parameter in parameter_overrides or []:
            self.parameters[parameter.name] = parameter.value
        if StubNode.force_system_time:
            self.parameters['use_sim_time'] = False
        self.published_topics, self.subscribed_topics = [], []
        self.startup_logs = []

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def declare_parameter(self, name, default):
        self.parameters.setdefault(name, default)
        return self.get_parameter(name)

    def create_publisher(self, message_type, topic, qos):
        self.published_topics.append(topic)
        return types.SimpleNamespace(publish=lambda message: None)

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscribed_topics.append(topic)

    def create_service(self, *args, **kwargs):
        return object()

    def create_client(self, *args, **kwargs):
        return object()

    def create_timer(self, *args, **kwargs):
        return object()

    def get_logger(self):
        return types.SimpleNamespace(info=self.startup_logs.append)


class Lite3StartupClockTests(unittest.TestCase):
    def construct(self, system_time=False):
        with ExitStack() as stack:
            stack.enter_context(patch.object(StubNode, 'force_system_time', system_time))
            stack.enter_context(patch.object(implementation, 'Node', StubNode))
            stack.enter_context(patch.object(implementation, 'validate_model'))
            stack.enter_context(patch.object(implementation, 'URDFCenterOfMass', return_value=object()))
            stack.enter_context(patch.object(implementation, 'get_package_share_directory', return_value='/stub-share'))
            stack.enter_context(patch.object(implementation, 'Clock', return_value=object()))
            stack.enter_context(patch.object(implementation, 'ClockType', types.SimpleNamespace(STEADY_TIME='steady')))
            for method in ('get_parameter', 'declare_parameter', 'create_publisher',
                           'create_subscription', 'create_service', 'create_client',
                           'create_timer', 'get_logger'):
                stack.enter_context(patch.object(implementation.Gait, method,
                                                getattr(StubNode, method), create=True))
            return implementation.Gait()

    def test_constructor_sets_simulation_time_and_initializes_start_q(self):
        node = self.construct()
        self.assertTrue(node.parameters['use_sim_time'])
        self.assertEqual(node.node_name, 'lite3_controller')
        self.assertEqual(node.namespace, 'lite3_sim')
        self.assertIsNone(node.start_q)
        self.assertEqual(node.state, 'DISARMED')
        self.assertTrue(any('lite3-only-v5.2' in message for message in node.startup_logs))

    def test_explicit_system_time_override_stops_startup(self):
        with self.assertRaisesRegex(RuntimeError, 'requires use_sim_time=true'):
            self.construct(system_time=True)

    def test_joint_trajectory_and_feedback_use_the_same_lite3_controller(self):
        node = self.construct()
        self.assertIn('/lite3_sim/lite3_leg_controller/joint_trajectory', node.published_topics)
        self.assertIn('/lite3_sim/lite3_leg_controller/controller_state', node.subscribed_topics)


if __name__ == '__main__':
    unittest.main()
