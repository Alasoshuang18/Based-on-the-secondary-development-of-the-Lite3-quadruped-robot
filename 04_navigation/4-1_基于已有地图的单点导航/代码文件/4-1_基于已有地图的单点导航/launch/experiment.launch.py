"""Shared course bringup with navigation stack."""
from lite3_course_common.bringup import make_launch


def generate_launch_description():
    return make_launch('lite3_lab_5_1', __file__, stack='navigation')
