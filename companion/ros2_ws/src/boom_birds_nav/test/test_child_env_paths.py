"""回归：子进程的导入路径不得混入开发期暂存目录。

本会话真实踩过：`.bb_stage` 里的旧 `px4_interface_node.py` 经 `child_env()` 进入
子进程 `sys.path` 后遮蔽了真实源码，表现为"子进程跑的是旧代码"的伪失败。
"""

import os
import pathlib
import sys


def test_child_env_excludes_staging_paths():
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from conftest import child_env

    env = child_env()
    entries = env["PYTHONPATH"].split(os.pathsep)
    bad = [e for e in entries if ".bb_stage" in e]
    assert not bad, f"子进程 PYTHONPATH 混入暂存目录：{bad}"

    # 过滤不能过头：真实源码与 colcon install 的消息包必须仍在
    assert any(e.endswith("boom_birds_nav") for e in entries), f"缺少源码目录：{entries}"
    assert any("site-packages" in e for e in entries), f"缺少 site-packages：{entries}"
