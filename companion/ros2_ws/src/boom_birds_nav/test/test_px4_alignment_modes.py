"""真实节点上的模式语义测试：**航向核实不能替代原点证据**。

用 params 文件注入参数并以真实 `Px4InterfaceNode` 实例检查闸门判定，
避免只测替身而漏掉参数解析/模式校验。
"""

from __future__ import annotations

import pathlib

import pytest

pytest.importorskip("rclpy", reason="需要 ROS 2 运行环境")

import rclpy  # noqa: E402


def _node_with(tmp_path, name: str, **params):
    """按参数创建一个真实节点（同进程，用 params 文件注入）。"""
    if not rclpy.ok():
        rclpy.init()
    from boom_birds_nav.px4_interface_node import Px4InterfaceNode

    simulate_restart = params.pop("simulate_restart", False)
    backend_kind = params.pop("backend", "fake")
    lines = ["boom_birds_px4_interface:", "  ros__parameters:",
             f'    backend: "{backend_kind}"']
    for key, value in params.items():
        if isinstance(value, bool):
            lines.append(f"    {key}: {'true' if value else 'false'}")
        elif isinstance(value, str):
            lines.append(f'    {key}: "{value}"')
        elif isinstance(value, (list, tuple)):
            lines.append(f"    {key}: [{', '.join(str(v) for v in value)}]")
        else:
            lines.append(f"    {key}: {value}")
    cfg = tmp_path / f"{name}.yaml"
    cfg.write_text("\n".join(lines) + "\n", encoding="utf-8")

    # 用子进程更安全（参数注入 + 避免节点名冲突），但这里只需要判定函数，
    # 因此直接构造节点对象并注入参数
    import subprocess
    import sys

    probe = tmp_path / f"{name}_result.txt"
    code = f'''
import json, sys
import rclpy
from boom_birds_nav.px4_interface_node import Px4InterfaceNode
rclpy.init(args=["--ros-args", "--params-file", r"{cfg}"])
node = Px4InterfaceNode()
before_restart = node._position_allowed_by_alignment()
if {simulate_restart!r}:
    node.backend.simulate_px4_restart()
allow, reason, missing = node._position_allowed_by_alignment()
print(json.dumps({{
    "allow": allow, "reason": reason, "missing": list(missing),
    "before_restart": before_restart[0],
    "report": node.alignment_report(),
}}))
node.destroy_node()
rclpy.shutdown()
'''
    env = __import__("os").environ.copy()
    paths = [p for p in sys.path if p and __import__("os").path.isdir(p)]
    env["PYTHONPATH"] = __import__("os").pathsep.join(paths)
    env["ROS_DOMAIN_ID"] = env.get("ROS_DOMAIN_ID", "194")
    env["ROS_LOCALHOST_ONLY"] = "1"
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                          env=env, timeout=120)
    assert proc.returncode == 0, (
        f"节点构造失败（rc={proc.returncode}）：\n{proc.stdout[-1500:]}\n{proc.stderr[-1500:]}"
    )
    import json

    payload = json.loads(proc.stdout.strip().splitlines()[-1])
    return payload, proc


def test_default_none_blocks_and_reports_missing_evidence(tmp_path):
    payload, _ = _node_with(tmp_path, "default_none")
    assert payload["allow"] is False
    assert payload["reason"] == "frame_alignment_none"
    assert len(payload["missing"]) == 2, "应说明缺「水平朝向」与「原点/平移」两项证据"
    report = payload["report"]
    assert report["mode"] == "none"
    assert report["position_allowed"] is False
    assert report["origin_evidence"] is False


def test_origin_evidence_alone_does_not_allow_position(tmp_path):
    """**只要**原点证据、航向未核实 ⇒ 仍然不放行（缺航向证据）。"""
    payload, _ = _node_with(
        tmp_path, "origin_only",
        frame_alignment="declared",
        frame_alignment_origin_evidence=True,
        frame_alignment_origin_note="EKF 原点由外部视觉定义在 VIO 原点上",
    )
    assert payload["allow"] is False, "只有原点证据时不得放行"
    assert payload["report"]["origin_evidence"] is True
    joined = " ".join(payload["missing"])
    assert "水平朝向" in joined, f"应指出缺水平朝向证据：{payload['missing']}"


def test_yaw_observed_alone_does_not_allow_position(tmp_path):
    """**关键回归**：只有"航向已核实"（observed）、没有原点证据 ⇒ 不得放行。

    旧实现把"航向残差达标"当成完整对齐，直接放行位置 setpoint；这里锁死新语义。
    """
    payload, _ = _node_with(
        tmp_path, "yaw_only",
        frame_alignment="declared",
        frame_alignment_observed=True,          # 只声明航向已核实
        frame_alignment_origin_evidence=False,  # 原点没有证据
    )
    assert payload["allow"] is False, "航向已核实但原点未知时必须仍然拦住"
    assert payload["reason"] == "yaw_verified_origin_unknown"
    joined = " ".join(payload["missing"])
    assert "原点" in joined, f"应指出缺原点证据：{payload['missing']}"


def test_both_evidences_allow_position(tmp_path):
    """两项证据齐备才放行。"""
    payload, _ = _node_with(
        tmp_path, "both",
        frame_alignment="declared",
        frame_alignment_observed=True,
        frame_alignment_origin_evidence=True,
    )
    assert payload["allow"] is True
    assert "origin_evidence" in payload["reason"]


def test_unverified_test_only_allows_but_is_marked(tmp_path):
    payload, _ = _node_with(tmp_path, "test_only",
                            frame_alignment="unverified_test_only")
    assert payload["allow"] is True
    assert payload["reason"] == "frame_alignment_unverified_test_only"
    assert payload["report"]["covers"] == "unverified_test_only"


def test_unverified_test_only_cannot_send_over_real_backend(tmp_path):
    with pytest.raises(AssertionError) as exc:
        _node_with(
            tmp_path, "test_only_real_send", backend="mavlink", dry_run=False,
            connect_on_start=False, frame_alignment="unverified_test_only",
        )
    assert "只允许 Fake 后端或 dry_run=true" in str(exc.value)


def test_px4_restart_invalidates_previously_declared_alignment(tmp_path):
    payload, _ = _node_with(
        tmp_path, "restart_invalidates_alignment",
        frame_alignment="identity", frame_alignment_observed=True,
        frame_alignment_origin_evidence=True, simulate_restart=True,
    )
    assert payload["before_restart"] is True
    assert payload["allow"] is False
    assert payload["reason"] == "alignment_invalidated_after_px4_restart"
    assert payload["report"]["invalidated_after_px4_restart"] is True
    assert payload["report"]["observed_yaw"] is False
    assert payload["report"]["origin_evidence"] is False
    assert payload["report"]["configured_yaw_evidence"] is True
    assert payload["report"]["configured_origin_evidence"] is True


def test_identity_rejects_nonzero_yaw_offset(tmp_path):
    """identity 表示两系同向同原点；配了非零偏移就是自相矛盾，必须报错。"""
    with pytest.raises(AssertionError) as exc:
        _node_with(tmp_path, "identity_bad_yaw",
                   frame_alignment="identity", frame_alignment_yaw_offset_rad=0.3)
    assert "identity" in str(exc.value)


def test_identity_rejects_nonzero_translation(tmp_path):
    with pytest.raises(AssertionError) as exc:
        _node_with(tmp_path, "identity_bad_t",
                   frame_alignment="identity",
                   frame_alignment_translation_m=[1.0, 0.0, 0.0])
    assert "identity" in str(exc.value)


def test_identity_with_zero_offsets_is_accepted(tmp_path):
    payload, _ = _node_with(tmp_path, "identity_ok",
                            frame_alignment="identity",
                            frame_alignment_observed=True,
                            frame_alignment_origin_evidence=True)
    assert payload["allow"] is True
