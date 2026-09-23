"""OpenVINS 订阅验证：真实启动 `run_subscribe_msckf`，确认它订阅我们的左右图与 IMU 话题。

为什么只验证"订阅"而不验证"初始化"：
- 记录帧回放没有曝光时间戳、也没有同步的真实 IMU，**不可能**得到有意义的 VIO 初始化；
  把它写成"VIO 通过"就是过度声明（见 STATUS.md 的 NOT RUN 行）。
- 但"OpenVINS 是否真的订阅这两个图像话题"是**可验证的集成事实**，而且正是用户指出的缺口，
  所以这里用真实进程 + 真实话题连接数来锁死它。

做法：以 0.5 s 心跳跑 15 s，检查 `/boom_birds/stereo/left_raw` 与 `.../right_raw` 的
订阅者数量 ≥ 1；同时确认 OpenVINS 进程没有立刻退出（退出说明配置无效、验证无效）。
"""

from __future__ import annotations

import os
import pathlib
import shutil
import subprocess
import sys
import time

import pytest

pytest.importorskip("rclpy", reason="需要 ROS 2 运行环境")

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

NAV = pathlib.Path(__file__).resolve().parents[1]
PARAMS = NAV / "config" / "stereo_camera.yaml"
RECORDINGS = pathlib.Path(__file__).resolve().parent / "recordings"

LEFT = "/boom_birds/stereo/left_raw"
RIGHT = "/boom_birds/stereo/right_raw"
IMU = "/boom_birds/imu"
ODOM = "/boom_birds/vio/odom_ego"

def _ov_install_roots():
    """OpenVINS 安装前缀候选（可被 OV_INSTALL 覆盖）。

    注意不要用 `pathlib.Path("")` 当候选：它会退化成 `.`，让探测永远"命中"目录
    而找不到可执行文件（本文件第一版就踩了这个坑，表现为全部 skip）。
    """
    roots = []
    env_root = (os.environ.get("OV_INSTALL") or "").strip()
    if env_root:
        roots.append(pathlib.Path(env_root))
    roots.append(pathlib.Path("/home/waterc/bb_build/ov/install"))
    return roots


def _ov_source_root() -> pathlib.Path:
    """本文件位于 <repo>/companion/ros2_ws/src/boom_birds_nav/test/。"""
    return pathlib.Path(__file__).resolve().parents[3] / "src" / "open_vins"


def _ov_executable() -> pathlib.Path | None:
    for root in _ov_install_roots():
        exe = root / "ov_msckf" / "lib" / "ov_msckf" / "run_subscribe_msckf"
        if exe.is_file():
            return exe
    return None


def _enu_env(domain: str) -> dict:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = domain
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    return env


def test_openvins_subscribes_to_the_published_stereo_and_imu_topics(tmp_path):
    exe = _ov_executable()
    if exe is None:
        pytest.skip("未找到 ov_msckf 的 run_subscribe_msckf（需先构建 OpenVINS）")
    if shutil.which("ros2") is None:
        pytest.skip("无 ros2 CLI")

    domain = os.environ.get("ROS_DOMAIN_ID", "194")
    env = _enu_env(domain)

    # OpenVINS 需要一份估计器配置；这里用仓库自带的 euroc 配置（只用于让节点起来并订阅）
    ov_config = (_ov_source_root() / "config/euroc_mav/estimator_config.yaml")
    if not ov_config.is_file():
        pytest.skip(f"缺少 OpenVINS 配置：{ov_config}")

    ov_log = tmp_path / "openvins.log"
    # 注意：不能用 --params-file。OpenVINS 自己 declare_parameter("topic_imu", ...)，
    # 参数文件会先声明同名参数，节点启动即抛 ParameterAlreadyDeclaredException
    # （实测现象：terminate called after throwing ... has already been declared）。
    # 正确做法是用 ROS 2 remap，把它的默认话题重映射到我们的契约话题。
    ovf = ov_log.open("w")
    ov = subprocess.Popen(
        [str(exe), str(ov_config), "--ros-args",
         "-r", f"/imu0:={IMU}",
         "-r", f"/cam0/image_raw:={LEFT}",
         "-r", f"/cam1/image_raw:={RIGHT}"],
        stdout=ovf, stderr=subprocess.STDOUT, env=env,
    )

    # 只在本进程还没建过上下文时 init；绝不 shutdown（否则同进程后续测试无法再 init）
    if not rclpy.ok():
        rclpy.init()
    node: Node = rclpy.create_node("openvins_subscription_probe")
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        # --- 第一段：只确认 OpenVINS 起得来并且订阅了话题 ---
        deadline = time.monotonic() + 25.0
        counts: dict = {}
        while time.monotonic() < deadline:
            ex.spin_once(timeout_sec=0.05)
            counts = {
                topic: node.count_subscribers(topic)
                for topic in (LEFT, RIGHT, IMU)
            }
            if all(v >= 1 for v in counts.values()):
                break
            if ov.poll() is not None:
                break

        alive = ov.poll() is None
        if not alive:
            # 进程退出 ⇒ 本次验证无效（不能把"没起来"当成"通过"）
            ovf.close()
            raise AssertionError(
                "OpenVINS 进程提前退出，订阅验证无效；日志尾部：\n"
                + ov_log.read_text(encoding="utf-8", errors="replace")[-1500:]
            )

        assert counts.get(LEFT, 0) >= 1, (
            f"OpenVINS 未订阅左图 {LEFT}（当前订阅者 {counts}）；"
            f"日志尾部：\n{ov_log.read_text(encoding='utf-8', errors='replace')[-1200:]}"
        )
        assert counts.get(RIGHT, 0) >= 1, (
            f"OpenVINS 未订阅右图 {RIGHT}（当前订阅者 {counts}）"
        )
        assert counts.get(IMU, 0) >= 1, (
            f"OpenVINS 未订阅 IMU {IMU}（当前订阅者 {counts}）"
        )
    finally:
        try:
            ex.remove_node(node)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        ov.terminate()
        try:
            ov.wait(timeout=8)
        except Exception:  # noqa: BLE001
            ov.kill()
        ovf.close()
        # 不调用 rclpy.shutdown()：一个进程里 Context 只能 init 一次，
        # 这里 shutdown 会让同进程后续的跨进程测试直接失败。



def test_openvins_topic_defaults_differ_from_ours_so_remap_is_required():
    """记录一个**接口事实**：OpenVINS 默认话题(/cam0/image_raw,/imu0)与我们的不同，
    因此集成必须显式 remap/传参；漏配时会静默收不到数据。"""
    src = _ov_source_root() / "ov_msckf/src/ros/ROS2Visualizer.cpp"
    if not src.is_file():
        pytest.skip("缺少 OpenVINS 源码")
    text = src.read_text(encoding="utf-8", errors="replace")
    assert 'declare_parameter<std::string>("topic_imu", "/imu0")' in text
    assert '"/cam" + std::to_string(0) + "/image_raw"' in text
    # 我们的契约话题名与 OpenVINS 默认值不同 ⇒ 必须显式配置
    assert LEFT != "/cam0/image_raw"
    assert IMU != "/imu0"
