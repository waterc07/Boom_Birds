# ******************************************************************************
# @file           : tag_forward_land.launch.py
# @brief          : 启动前飞后用下视 Tag 降落的独立任务。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""tag_forward_land: VIO flight and downward Tag landing, without EGO."""
from boom_birds_bringup.tag_task_launch import taskLaunch


def generate_launch_description():
    return taskLaunch("tag_forward_land")
