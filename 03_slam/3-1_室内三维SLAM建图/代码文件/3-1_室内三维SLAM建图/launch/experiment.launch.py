from launch import LaunchDescription
from launch.actions import OpaqueFunction
from lite3_course_common.bringup import make_launch
from lite3_lab_4_1.startup_guard import preflight


def generate_launch_description():
    description = make_launch('lite3_lab_4_1', __file__, stack='lio')
    actions = list(description.entities)
    # Resolve declared arguments, then inspect leftovers before creating nodes.
    for index, action in enumerate(actions):
        if isinstance(action, OpaqueFunction):
            actions.insert(index, OpaqueFunction(function=preflight))
            return LaunchDescription(actions)
    raise RuntimeError('4-1 launch structure changed; startup preflight not installed')
