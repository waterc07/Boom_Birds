"""无 IMU 持续输入有界；正常窗口与乱序输入保持时间顺序。"""
import pathlib
import subprocess


def test_bounded_camera_queue(tmp_path):
    src = pathlib.Path(__file__).resolve().parents[2] / "open_vins" / "ov_msckf"
    executable = tmp_path / "queue_test"
    subprocess.run(["g++", "-std=c++14", "-I", str(src / "src/ros"),
                    str(src / "test/test_camera_queue.cpp"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
