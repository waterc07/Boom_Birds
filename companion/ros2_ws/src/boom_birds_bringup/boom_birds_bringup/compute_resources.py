# ******************************************************************************
# @file           : compute_resources.py
# @brief          : 在降落交接许可确认后停止本任务持有的计算子进程。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

"""Release only owned child processes after a confirmed landing lease."""


class ComputeResources:
    def __init__(self, vioProcesses, stereoProcesses, *, stopOwned=None):
        self.vio = tuple(vioProcesses)
        self.stereo = tuple(stereoProcesses)
        self.stopOwned = stopOwned
        self.token = None
        self.released = False

    def confirm(self, output):
        if not output.release_compute or not output.token:
            raise ValueError("handoff_not_confirmed")
        if self.token is not None and self.token != output.token:
            raise ValueError("lease_token_changed")
        self.token = output.token

    def release(self, token):
        if not self.token or token != self.token:
            raise ValueError("unconfirmed_compute_release")
        if self.stopOwned is None:
            # 释放必须绑定调用者持有的进程组回读；没有句柄就不允许报告已释放。
            raise ValueError("owned_processes_not_registered")
        self.released = self.stopOwned() is True
        return self.released
