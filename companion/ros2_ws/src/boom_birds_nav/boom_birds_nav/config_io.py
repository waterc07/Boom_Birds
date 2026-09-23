"""配置读取：外参与标定必须显式提供，缺失即失败，不允许静默占位。"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from .frames import is_rotation, make_transform


class ConfigError(RuntimeError):
    pass


def _matrix4(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape != (4, 4):
        raise ConfigError(f"{name} 必须是 4×4 矩阵，实际形状 {arr.shape}")
    if not is_rotation(arr[:3, :3]):
        raise ConfigError(f"{name} 的旋转块不是正交旋转（det 应为 +1）")
    return arr


def _vector3(value, name: str, default=None) -> np.ndarray:
    if value is None:
        if default is None:
            raise ConfigError(f"缺少必需字段 {name}")
        return np.asarray(default, dtype=float)
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != 3:
        raise ConfigError(f"{name} 必须是 3 元素向量")
    return arr


def load_extrinsics(path: str) -> dict:
    """读取外参配置，返回字典。

    必需字段：
      T_I_C0 : 4×4，把相机光学系坐标变到 IMU 系（即 Kalibr/OpenVINS 的 T_imu_cam 字段）。
    可选字段：
      T_I_B  : 4×4，机体在 IMU 系中的位姿，缺省为单位阵（须显式声明原点重合约束）；
      p_I_B_m: 3 元素，机体原点在 IMU 系中的位置（杆臂），与 T_I_B 同时给出时必须一致；
      source : 来源说明（TEST-ONLY / 真机标定文件），必须写明。
    """
    cfg_path = Path(path)
    if not cfg_path.is_file():
        raise ConfigError(f"外参配置不存在：{cfg_path}")
    with cfg_path.open("r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}

    if "T_I_C0" not in raw:
        raise ConfigError("外参配置缺少 T_I_C0（相机→IMU，Kalibr 的 T_imu_cam）")
    T_i_c0 = _matrix4(raw["T_I_C0"], "T_I_C0")

    if "T_I_B" in raw:
        T_i_b = _matrix4(raw["T_I_B"], "T_I_B")
    else:
        if not raw.get("assume_origin_coincident", False):
            raise ConfigError(
                "缺少 T_I_B 时必须显式设置 assume_origin_coincident: true（本轮限制两原点重合）；"
                "真实设备上该假设不成立"
            )
        T_i_b = np.eye(4)

    p_i_b = _vector3(raw.get("p_I_B_m"), "p_I_B_m", default=T_i_b[:3, 3])
    if not np.allclose(p_i_b, T_i_b[:3, 3], atol=1e-9):
        raise ConfigError("p_I_B_m 与 T_I_B 的平移不一致，二者只能描述同一杆臂")

    return {
        "T_I_C0": T_i_c0,
        "T_I_B": T_i_b,
        "p_I_B": p_i_b,
        "source": str(raw.get("source", "未注明来源")),
        "path": str(cfg_path),
    }


def load_contract(path: str) -> dict:
    cfg_path = Path(path)
    if not cfg_path.is_file():
        raise ConfigError(f"契约配置不存在：{cfg_path}")
    with cfg_path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}
