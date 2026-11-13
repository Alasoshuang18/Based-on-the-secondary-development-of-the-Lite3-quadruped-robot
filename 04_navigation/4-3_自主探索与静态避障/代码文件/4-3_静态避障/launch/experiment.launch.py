"""Online SLAM session; navigation and official control start separately."""
from lite3_lab_5_2.stage_launch import generate_stage_launch

def generate_launch_description():
    return generate_stage_launch(3)
