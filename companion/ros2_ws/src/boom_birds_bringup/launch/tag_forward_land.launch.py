"""tag_forward_land: VIO flight and downward Tag landing, without EGO."""
from boom_birds_bringup.tag_task_launch import taskLaunch


def generate_launch_description():
    return taskLaunch("tag_forward_land")
