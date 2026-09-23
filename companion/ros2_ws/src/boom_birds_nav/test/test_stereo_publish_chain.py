"""任务 A 端到端脱机验证：保存的真实双目帧 → 左右图/CameraInfo → 深度节点订阅。

覆盖三层：
1. **发布语义**：回放一个真实记录帧，断言左右图同尺寸**同时间戳**（同一帧只有一个时间戳）、
   拼接图 == 左右水平拼接、CameraInfo 与标定一致（含尺寸不一致时的显式缩放）；
2. **唯一采集源**：四种模式共用一个节点、一份发布代码；测试用 replay 模式驱动真实节点；
3. **下游订阅**：用真实 `depth_node` 订阅同一对话题并产出深度与校正 CameraInfo
   ——检查消息内容，而不是只看节点能启动。

为什么节点用**子进程**启动：`rclpy.init()` 只第一次生效，整包一起跑时本文件不一定
是第一个初始化 rclpy 的测试；子进程启动（等价 `ros2 run --ros-args`）与初始化顺序无关，
也顺带验证了真实可执行文件与参数文件能被 `ros2 run` 正确加载。

边界：记录帧**没有曝光时间戳**（见 `test/recordings/PROVENANCE.md`），本测试不证明曝光时刻、
不证明相机—IMU 已同步；真机 v4l2 路径只能在设备上验证。
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import subprocess
import sys
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

REPO = pathlib.Path(__file__).resolve().parents[5]          # .../Boom_Birds
RECORDINGS = pathlib.Path(__file__).resolve().parent / "recordings"
CALIB = (REPO / "companion/ros2_ws/src/stereo_depth/calibration"
         / "live_20260916_210120_642136/candidate.npz")

QOS_IMG = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
FULL_W, FULL_H = 1280, 480
CALIB_IMAGE_SIZE = (1280, 960)      # 现有标定的每目分辨率（实测）

_CFG = {"params_file": None}


def _params_file() -> pathlib.Path:
    """按真实节点名写参数覆盖（不用通配符，避免串到同进程的其它节点）。"""
    if _CFG["params_file"] is None:
        import tempfile
        path = pathlib.Path(tempfile.mkdtemp()) / "bb_test_params.yaml"
        path.write_text(f"""boom_birds_stereo_source:
  ros__parameters:
    left_topic: "/bb_test/left"
    right_topic: "/bb_test/right"
    stitched_topic: "/bb_test/stitched"
    # 左右各自与图像配对（由 left_topic/right_topic 派生）
    stats_topic: "/bb_test/source_status"
    stats_rate_hz: 10.0
    mode: "replay"
    path: "{RECORDINGS / 'stereo.png'}"
    rate_hz: 30.0
    replay_fps: 30.0
    calibration_file: "{CALIB}"
    loop: true
boom_birds_depth:
  ros__parameters:
    calibration_file: "{CALIB}"
    left_topic: "/bb_test/left"
    right_topic: "/bb_test/right"
    depth_topic: "/bb_test/depth"
    xyz_topic: "/bb_test/xyz"
    xyz_valid_topic: "/bb_test/xyz_valid"
    camera_info_topic: "/bb_test/depth_camera_info"
    depth_compat_topic: "/bb_test/depth_compat"
""", encoding="utf-8")
        _CFG["params_file"] = path
    return _CFG["params_file"]


def _spawn(module: str, node_name: str, log_path: pathlib.Path):
    args = [sys.executable, "-m", module, "--ros-args",
            "--params-file", str(_params_file()),
            "-r", f"__node:={node_name}"]
    env = os.environ.copy()
    paths = [p for p in sys.path if p and os.path.isdir(p)]
    env["PYTHONPATH"] = os.pathsep.join(paths + [env.get("PYTHONPATH", "")]).strip(os.pathsep)
    env.setdefault("ROS_DOMAIN_ID", "196")
    env.setdefault("ROS_LOCALHOST_ONLY", "1")
    logf = log_path.open("w")
    proc = subprocess.Popen(args, stdout=logf, stderr=subprocess.STDOUT, env=env)
    return proc, logf


def _stop(proc, logf=None):
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except Exception:  # noqa: BLE001
        proc.kill()
    if logf is not None:
        logf.close()


def _log_text(path: pathlib.Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")[-2000:]
    except OSError:
        return ""


class Collector:
    """订阅一组话题并保留全部消息，便于断言消息内容。"""

    def __init__(self, node, topics):
        self.data = {name: [] for name in topics}
        for name, (msg_type, depth) in topics.items():
            node.create_subscription(msg_type, name,
                                     lambda m, n=name: self.data[n].append(m), depth)


def _spin_until(executor, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def _load_calibration():
    with np.load(CALIB) as archive:
        return {k: archive[k].copy() for k in archive.files}


@pytest.fixture()
def replay_source_proc(tmp_path):
    """真实 stereo_source 进程（replay 模式）。"""
    if not RECORDINGS.joinpath("stereo.png").is_file():
        pytest.skip(f"缺少记录帧夹具：{RECORDINGS}")
    if not CALIB.is_file():
        pytest.skip(f"缺少标定文件：{CALIB}")
    log = tmp_path / "stereo_source.log"
    proc, logf = _spawn("boom_birds_nav.stereo_source", "boom_birds_stereo_source", log)
    time.sleep(3.0)
    try:
        yield log
    finally:
        _stop(proc, logf)


def _probe():
    if not rclpy.ok():
        rclpy.init()
    return rclpy.create_node("stereo_publish_probe")


def test_replay_publishes_shared_timestamp_and_calibrated_intrinsics(replay_source_proc):
    """回放链：同一帧的左右图/拼接图/CameraInfo 共享采集时间戳，且内参按分辨率缩放。

    取帧必须**按时间戳配对**：分别取各话题"最后一条"在负载下会跨帧
    （实测左 617.949 / 右 618.283），那是测试脚手架时序问题，不是被测代码的问题。
    """
    log = replay_source_proc
    probe = _probe()
    got = Collector(probe, {
        "/bb_test/left": (Image, QOS_IMG),
        "/bb_test/right": (Image, QOS_IMG),
        "/bb_test/stitched": (Image, QOS_IMG),
        "/bb_test/left/camera_info": (CameraInfo, QOS_IMG),
        "/bb_test/right/camera_info": (CameraInfo, QOS_IMG),
        "/bb_test/source_status": (String, 10),
    })
    ex = SingleThreadedExecutor()
    ex.add_node(probe)

    def _stamp(m):
        return (m.header.stamp.sec, m.header.stamp.nanosec)

    def _stamps(msgs):
        return {_stamp(m) for m in msgs}

    def _common_stamps():
        """五条流都出现过的时间戳 = 真正完整可配对的那一帧。

        左右 CameraInfo 各自独立话题后，任一话题的 QoS 丢帧仍可能让某一帧不完整；
        等一个五条流都有的时间戳再断言，既不掩盖丢帧，也不把测试脚手架的丢帧当成被测代码的错。
        """
        sets = [
            _stamps(got.data["/bb_test/left"]),
            _stamps(got.data["/bb_test/right"]),
            _stamps(got.data["/bb_test/stitched"]),
            _stamps(got.data["/bb_test/left/camera_info"]),
            _stamps(got.data["/bb_test/right/camera_info"]),
        ]
        return set.intersection(*sets) if all(sets) else set()

    try:
        ok = _spin_until(ex, lambda: bool(_common_stamps()), timeout=25.0)
        assert ok, (
            f"未收到五条流齐全的同帧发布：{ {k: len(v) for k, v in got.data.items()} } "
            f"共同时间戳={sorted(_common_stamps())}\n"
            f"--- 节点日志 ---\n{_log_text(log)}"
        )

        paired_stamp = sorted(_common_stamps())[-1]  # 唯一确定的一帧

        def _pick(topic):
            for m in got.data[topic]:
                if _stamp(m) == paired_stamp:
                    return m
            raise AssertionError(f"{topic} 缺 {paired_stamp} 这一帧")

        left = _pick("/bb_test/left")
        right = _pick("/bb_test/right")
        stitched = _pick("/bb_test/stitched")
        infos_left = got.data["/bb_test/left/camera_info"]
        infos_right = got.data["/bb_test/right/camera_info"]

        # --- 尺寸与编码：原始左右目，未校正 ---
        assert left.encoding == right.encoding == "mono8"
        assert (left.height, left.width) == (FULL_H, FULL_W // 2)
        assert (right.height, right.width) == (FULL_H, FULL_W // 2)
        assert left.header.frame_id == "cam0" and right.header.frame_id == "cam1"

        # --- 同一帧：左右与拼接图共享同一采集时间戳 ---
        assert _stamp(left) == _stamp(right)
        assert _stamp(stitched) == _stamp(left)

        # --- 时间戳来自采集/回放，不是 0、也不是发布时间 ---
        #
        # 只断言单调性、量级与「落在 replay_fps 网格上」：回放时间戳由回放器按 replay_fps
        # 合成，但**发布**节奏取决于测试 spin 频率与调度抖动（单次 spin 可能一次跑完多个
        # 到期定时器），因此不能要求相邻帧恰好差 1/30 s——那是在断言测试脚手架。
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                  for m in got.data["/bb_test/left"]]
        assert all(s > 0.0 for s in stamps)
        deltas = [b - a for a, b in zip(stamps, stamps[1:])]
        assert all(d > 0 for d in deltas), f"时间戳必须严格递增：{deltas}"
        assert all(d <= 0.3 for d in deltas), f"相邻发布时间差过大：{deltas}"
        t0 = stamps[0]
        cycles = [(s - t0) * 30.0 for s in stamps]
        assert all(abs(c - round(c)) < 0.05 for c in cycles), (
            f"时间戳未落在 1/30 s 的合成网格上（疑似用了发布时间戳）：{cycles}"
        )

        # --- 内容：拼接图 == 左右水平拼接；且与真实记录帧一致 ---
        larr = np.frombuffer(left.data, np.uint8).reshape(left.height, left.width)
        rarr = np.frombuffer(right.data, np.uint8).reshape(right.height, right.width)
        sarr = np.frombuffer(stitched.data, np.uint8).reshape(stitched.height, stitched.width)
        assert np.array_equal(sarr, np.hstack([larr, rarr])), "拼接图必须等于左右图水平拼接"
        assert not np.array_equal(larr, rarr), "左右内容相同则无法证明切分正确"
        import cv2
        full = cv2.imread(str(RECORDINGS / "stereo.png"), cv2.IMREAD_GRAYSCALE)
        assert full is not None and full.shape == (FULL_H, FULL_W)
        assert np.array_equal(larr, full[:, :FULL_W // 2])
        assert np.array_equal(rarr, full[:, FULL_W // 2:])

        # --- CameraInfo：左右各一条，来自标定；尺寸不一致必须显式缩放 ---
        calib = _load_calibration()
        k1 = np.asarray(calib["K1"], dtype=float).reshape(3, 3)
        k2 = np.asarray(calib["K2"], dtype=float).reshape(3, 3)
        baseline_full = float(np.linalg.norm(np.asarray(calib["T"], dtype=float).ravel()))
        scale = (FULL_W // 2) / CALIB_IMAGE_SIZE[0]
        assert scale == pytest.approx(0.5)      # 现有标定 1280×960 vs 采集每目 640×480

        def _by_stamp(msgs):
            return {_stamp(m): m for m in msgs}

        info_cam0 = _by_stamp(infos_left)
        info_cam1 = _by_stamp(infos_right)
        assert paired_stamp in info_cam0 and paired_stamp in info_cam1, (
            "同帧的左右 CameraInfo 必须都存在："
            f"cam0 有 {len(info_cam0)} 帧、cam1 有 {len(info_cam1)} 帧"
        )
        by_frame = {"cam0": info_cam0[paired_stamp], "cam1": info_cam1[paired_stamp]}
        # 分话题后：每个话题里只应出现本目的 frame_id（不再靠 frame_id 猜左右）
        assert {m.header.frame_id for m in infos_left} == {"cam0"}, (
            f"左目 CameraInfo 话题里出现了 {sorted({m.header.frame_id for m in infos_left})}"
        )
        assert {m.header.frame_id for m in infos_right} == {"cam1"}, (
            f"右目 CameraInfo 话题里出现了 {sorted({m.header.frame_id for m in infos_right})}"
        )

        for frame_id, expected_k in (("cam0", k1), ("cam1", k2)):
            info = by_frame[frame_id]
            assert (info.width, info.height) == (FULL_W // 2, FULL_H)
            assert info.distortion_model == "plumb_bob"
            assert len(info.d) == 5
            scaled_k = expected_k.copy()
            scaled_k[0, :] *= scale
            scaled_k[1, :] *= scale
            assert np.allclose(np.array(info.k), scaled_k.ravel(), atol=1e-6), (
                f"{frame_id} 的 K 未按 {scale} 缩放，与采集分辨率不符"
            )
            assert np.allclose(np.array(info.r), np.eye(3).ravel()), "原始图未校正，R 应为单位阵"
            p = np.array(info.p).reshape(3, 4)
            assert p[0, 0] == pytest.approx(scaled_k[0, 0], abs=1e-6)
            if frame_id == "cam0":
                assert p[0, 3] == pytest.approx(0.0)
            else:
                # P[0][3] = -fx_当前分辨率 · B_物理。B 是米制刚体量，**不随分辨率缩放**。
                # 曾经写成 -fx_scaled · B · scale（即把基线也缩了），在 scale=0.5 时
                # 只有正确值的 1/4；这是 P1 缺陷，此断言即为回归。
                assert p[0, 3] == pytest.approx(-scaled_k[0, 0] * baseline_full, rel=1e-6), (
                    "右目 P[0][3] 应为 -fx_scaled·B（物理基线不缩放）；"
                    f"got={p[0, 3]} expected={-scaled_k[0, 0] * baseline_full}"
                )
                # 明确排除"基线被缩放"的两种错法
                wrong_sq = -scaled_k[0, 0] * baseline_full * scale
                assert not math.isclose(p[0, 3], wrong_sq, rel_tol=1e-6), (
                    "右目 P[0][3] 又变成了 -fx·B·scale（基线被错误缩放）"
                )
                wrong_full = -expected_k[0, 0] * baseline_full
                assert not math.isclose(p[0, 3], wrong_full, rel_tol=1e-6), (
                    "右目 P[0][3] 未按分辨率缩放 fx"
                )
            assert _stamp(info) == _stamp(left), (
                f"{frame_id} 的 CameraInfo 与同帧左图时间戳不一致："
                f"{_stamp(info)} != {_stamp(left)}"
            )

        # --- 诊断必须说明模式、时间戳来源与内参缩放 ---
        _spin_until(ex, lambda: any(
            json.loads(m.data).get("raw_info_scale") is not None
            for m in got.data["/bb_test/source_status"]
        ), timeout=5.0)
        stats = None
        for m in reversed(got.data["/bb_test/source_status"]):
            candidate = json.loads(m.data)
            if candidate.get("raw_info_scale") is not None:
                stats = candidate
                break
        assert stats is not None, (
            "诊断中缺少 raw_info_scale："
            f"{[json.loads(m.data) for m in got.data['/bb_test/source_status']]}"
        )
    finally:
        try:
            ex.remove_node(probe)
        except Exception:  # noqa: BLE001
            pass
        probe.destroy_node()


def test_downstream_depth_node_consumes_the_same_topics(tmp_path):
    """真实 depth_node 订阅同一对话题 → 产出深度与校正 CameraInfo。

    这是「深度与 OpenVINS 共用同一采集源」的下游一半：证明话题名、QoS、帧格式、
    时间戳都被现有算法节点接受，而不是只证明采集节点自己会发。
    """
    if not RECORDINGS.joinpath("stereo.png").is_file() or not CALIB.is_file():
        pytest.skip("缺少记录帧或标定夹具")

    src_log = tmp_path / "stereo_source.log"
    depth_log = tmp_path / "depth.log"
    src, src_f = _spawn("boom_birds_nav.stereo_source", "boom_birds_stereo_source", src_log)
    depth, depth_f = _spawn("boom_birds_nav.depth_node", "boom_birds_depth", depth_log)
    try:
        time.sleep(3.0)
        probe = _probe()
        got = Collector(probe, {
            "/bb_test/depth": (Image, QOS_IMG),
            "/bb_test/depth_camera_info": (CameraInfo, QOS_IMG),
        })
        ex = SingleThreadedExecutor()
        ex.add_node(probe)
        try:
            ok = _spin_until(ex, lambda: len(got.data["/bb_test/depth"]) >= 1
                             and len(got.data["/bb_test/depth_camera_info"]) >= 1, timeout=25.0)
            assert ok, (
                f"深度节点未产出：{ {k: len(v) for k, v in got.data.items()} }\n"
                f"--- stereo 日志 ---\n{_log_text(src_log)}\n"
                f"--- depth 日志 ---\n{_log_text(depth_log)}"
            )

            depth_msg = got.data["/bb_test/depth"][-1]
            info = got.data["/bb_test/depth_camera_info"][-1]
            assert depth_msg.encoding == "32FC1"
            assert depth_msg.header.frame_id == "cam0_rect"
            assert (depth_msg.height, depth_msg.width) == (240, 320)
            arr = np.frombuffer(depth_msg.data, np.float32).reshape(240, 320)
            assert np.isfinite(arr).any(), "深度图中应至少存在有效观测"
            assert (arr[np.isfinite(arr)] > 0).all(), "有效深度必须为正（无效位置保持 NaN）"
            # 校正内参来自 P1（320×240），与原始 CameraInfo 不是同一套
            assert (info.width, info.height) == (320, 240)
            assert info.header.frame_id == "cam0_rect"
            assert info.p[0] > 0
        finally:
            ex.remove_node(probe)
            probe.destroy_node()
    finally:
        _stop(depth, depth_f)
        _stop(src, src_f)
