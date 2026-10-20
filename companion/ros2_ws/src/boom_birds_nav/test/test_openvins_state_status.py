"""只有实际通过视觉更新的特征才能刷新导航输出许可。"""
import pathlib
import subprocess


def test_visual_update_status(tmp_path):
    src = pathlib.Path(__file__).resolve().parents[2] / "open_vins" / "ov_msckf"
    executable = tmp_path / "status_test"
    subprocess.run(["g++", "-std=c++14", "-I", str(src / "src"),
                    str(src / "test/test_visual_update_status.cpp"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
