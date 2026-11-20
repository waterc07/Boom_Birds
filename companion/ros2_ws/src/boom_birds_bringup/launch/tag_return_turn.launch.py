# ******************************************************************************
# @file           : tag_return_turn.launch.py
# @brief          : 启动返回转向并用下视 Tag 降落的独立任务。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""tag_return_turn: VIO flight and downward Tag landing, without EGO."""
from boom_birds_bringup.tag_task_launch import taskLaunch


def generate_launch_description():
    return taskLaunch("tag_return_turn")
