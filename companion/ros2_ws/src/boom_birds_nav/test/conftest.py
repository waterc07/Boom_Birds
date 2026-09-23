"""把 src 布局下的两个包加入导入路径，使 pytest 可直接运行（不依赖 colcon install）。

另外提供 `child_env()`：以子进程方式跑真实 ROS 2 节点时，子进程必须能找到
colcon install 里的消息包（例如 `quadrotor_msgs`）——否则节点会在
`from quadrotor_msgs.msg import PositionCommand` 处直接退出，测试只会看到
「话题没有数据」这种误导性的现象。安装前缀从 COLCON_PREFIX_PATH / AMENT_PREFIX_PATH
推导，因此换机器、换架构（含 ARM64）无需改测试。

同时固定 DDS domain：`rclpy.init()` 在一个进程里只有第一次有效，所以"在测试函数里
setdefault domain"是不可靠的——只要更早的测试建过 ROS 上下文，domain 就已锁定为默认值，
子进程却从 env 继承到别的 domain，表现为「话题看得见、消息永远收不到」。
"""

import os
import pathlib
import sys

# conftest 在测试模块导入之前加载，因此这里设置会早于任何 rclpy.init() 生效。
os.environ.setdefault("ROS_DOMAIN_ID", "194")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

_WS_SRC = pathlib.Path(__file__).resolve().parents[2]      # .../ros2_ws/src
for _p in (_WS_SRC / "stereo_depth", _WS_SRC / "boom_birds_nav"):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


def _install_site_packages():
    """从环境里的 colcon/ament 安装前缀收集 site-packages（只取真实存在的目录）。"""
    found = []
    seen = set()
    for var in ("COLCON_PREFIX_PATH", "AMENT_PREFIX_PATH"):
        for entry in (os.environ.get(var) or "").split(os.pathsep):
            entry = entry.strip()
            if not entry or entry in seen:
                continue
            seen.add(entry)
            root = pathlib.Path(entry)
            if not root.is_dir():
                continue
            candidates = [root] + [d for d in root.iterdir() if d.is_dir()]
            for cand in candidates:
                for sp in sorted(cand.glob("lib/python3*/site-packages")):
                    if sp.is_dir() and str(sp) not in found:
                        found.append(str(sp))
    return found


def _usable_for_child(path: str) -> bool:
    """子进程可安全继承的导入路径。

    排除开发期暂存目录：它可能留着**旧文件快照**，一旦进入子进程 sys.path 就会遮蔽
    真实源码，制造"子进程跑的是旧代码"这类伪失败（本工程实际踩过，排查代价很高）。
    """
    return ".bb_stage" not in path


def child_env(**overrides):
    """构造子进程环境：当前进程可导入的路径 + colcon install 的 site-packages。

    调用方仍可用 overrides 覆盖 ROS_DOMAIN_ID 等。
    """
    env = os.environ.copy()
    paths = [p for p in sys.path if p and os.path.isdir(p) and _usable_for_child(p)]
    paths += [p for p in _install_site_packages() if p not in paths]
    env["PYTHONPATH"] = os.pathsep.join(paths).strip(os.pathsep)
    env["PYTHONUNBUFFERED"] = "1"       # 子进程异常退出时日志不丢
    env.update({k: str(v) for k, v in overrides.items()})
    return env
