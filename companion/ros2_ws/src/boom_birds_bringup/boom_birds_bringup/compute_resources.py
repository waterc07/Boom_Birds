"""Release only owned child processes after a confirmed landing lease."""
import subprocess


class ComputeResources:
    def __init__(self, vio_processes, stereo_processes):
        self.vio = tuple(vio_processes)
        self.stereo = tuple(stereo_processes)
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
        # 仅操作构造时交付的子进程句柄；等待退出可能阻塞，服务需使用独立 executor。
        for process in (*self.vio,*self.stereo):
            if process.poll() is None: process.terminate()
        for process in (*self.vio,*self.stereo):
            try: process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                # Retain an unconfirmed release; never report a still-running capture as stopped.
                return False
        self.released = all(p.poll() is not None for p in (*self.vio,*self.stereo))
        return self.released
