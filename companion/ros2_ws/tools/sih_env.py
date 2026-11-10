"""TEST-ONLY local SIH helpers shared by the launch and evidence tools.

只做只读探测与有界进程管理：不连接设备、不写 PX4 参数、不发送 setpoint。
本模块只提供公共原语，不改变任何调用方的判定语义。
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil
import signal
import subprocess

SIH_MODEL_ENV = dict(PX4_SIM_MODEL="sihsim_quadx", PX4_SIMULATOR="sihsim",
                     PX4_SYS_AUTOSTART="10040")


def existingPx4Instance(binary):
    """返回正在运行同一 px4 二进制的 pid；没有则返回 None。只读 /proc。"""
    target = Path(binary).resolve()
    for candidate in Path("/proc").iterdir():
        if not candidate.name.isdigit():
            continue
        try:
            if candidate.joinpath("exe").resolve() == target:
                return int(candidate.name)
        except (FileNotFoundError, PermissionError):
            continue
    return None


def sihSimulationEnv(base=None):
    """PX4 SIH 启动环境变量；base 缺省取 os.environ。"""
    env = dict(os.environ if base is None else base)
    env.update(SIH_MODEL_ENV)
    return env


def listenerTopic(binary, topic, timeout=.4):
    """单次读取 SIH uORB 话题（px4-listener -n 1）；调用方须先核验 SIH 进程。"""
    binary = Path(binary)
    return subprocess.run([str(binary.with_name("px4-listener")), topic, "-n", "1"],
                          cwd=binary.parent.parent / "rootfs/0",
                          capture_output=True, text=True, timeout=timeout)


def terminateProcessGroup(process, termTimeout=4., killTimeout=2.):
    """按进程组 TERM→KILL 有界终止；进程已退出时为空操作。"""
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=termTimeout)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=killTimeout)


def copyNewestUlog(source, since, target):
    """把 since 之后生成的最新 ULog 复制到 target，返回 sha256；没有则 None。

    只扫描一次源码树（旧实现前后各 rglob 一次再取差集），按 mtime 判定本次新建。
    """
    source, target = Path(source), Path(target)
    created = [p for p in source.rglob("*.ulg") if p.stat().st_mtime >= since]
    if not created:
        return None
    shutil.copy2(max(created, key=lambda p: p.stat().st_mtime), target)
    return hashlib.sha256(target.read_bytes()).hexdigest()


_HTE_CACHE = {}


def sihHteMode(source):
    """读取 PX4 位置控制器源码判断 HTE 参数形态；同一路径只读一次。

    仅用于 TEST-ONLY 本机 SIH 兼容，不改变生产控制。返回
    dict(hasHteParameter=..., alwaysOn=..., path=...)。
    """
    path = Path(source) / "src/modules/mc_pos_control/MulticopterPositionControl.cpp"
    key = str(path)
    if key not in _HTE_CACHE:
        text = path.read_text()
        _HTE_CACHE[key] = dict(
            hasHteParameter="_param_mpc_use_hte" in text,
            alwaysOn=("_param_mpc_use_hte" not in text
                      and "_control.updateHoverThrust(hte.hover_thrust)" in text),
            path=str(path))
    return _HTE_CACHE[key]
