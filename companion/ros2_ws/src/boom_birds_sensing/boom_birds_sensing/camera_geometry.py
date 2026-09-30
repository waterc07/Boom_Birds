"""深度输出几何契约：三路同源、缩放只按实际输出尺寸、物理基线不缩放。

三条内参路径必须来自**同一标定**与**同一次实际输出尺寸**：

1. ``stereo_source.raw_camera_info``：左右**原始**（未校正）内参，左右各发一路，
   供 OpenVINS 与深度节点共用；
2. ``depth_node``：由同一标定 ``P1`` 推导的**校正后左目**内参，随 ``output_scale`` 缩放；
3. EGO ``grid_map``：只订阅 (2) 发布的那一份 CameraInfo，不再自带静态内参。

本模块是 1↔2↔3 之间**唯一**的 Python 侧校验口径：

* 跨分辨率只缩放 ``fx/fy/cx/cy``（和 ``width/height``），**米制物理基线不缩放**；
* 右目投影项恒等于 ``P[0][3] = -fx_当前分辨率 · B_物理``，只随分辨率变一次；
* 任一不一致必须**拒绝并诊断**，而不是静默沿用默认值或上一次的内参；
* 尚无内参 / 尚无深度时视为**未就绪**，不得用默认值继续。
"""
from __future__ import annotations
from boom_birds_control.runtime_config import DEFAULTS

import hashlib
import math
from pathlib import Path

DEPTH_FRAME = "cam0_rect"
"""深度图与深度 CameraInfo 的光学帧。帧名改变即视为新几何，必须重建地图与轨迹。"""

#: 三路内参互检的默认容差（像素）。远小于任何真实标定差异，又足以吸收
#: 32 位消息字段的往返误差。
GEOMETRY_TOLERANCE_PX = 1e-3


class GeometryError(ValueError):
    """几何不一致。继承 ``ValueError``，调用方按具体类型或 ``ValueError`` 处理都可以。"""


def stereo_baseline_term(fx: float, baseline_m: float) -> float:
    """右目投影矩阵的基线项：``P[0][3] = -fx_当前分辨率 · B_物理``。

    基线是**米制刚体量**，纯降采样不改变它；只有 ``fx``（像素量）随分辨率变化，
    因此该项只随分辨率变化一次，而不是 scale²。
    """
    if isinstance(fx, bool) or not isinstance(fx, (int, float)) or not math.isfinite(fx):
        raise GeometryError("fx 必须是有限数")
    if isinstance(baseline_m, bool) or not isinstance(baseline_m, (int, float))\
            or not math.isfinite(baseline_m):
        raise GeometryError("baseline_m 必须是有限数")
    if fx <= 0.0:
        raise GeometryError("fx 必须为正")
    if baseline_m < 0.0:
        raise GeometryError("物理基线长度不能为负")
    return -float(fx) * float(baseline_m)


def validate_projection_matrix(p, width: int, height: int, *, side: str = "left",
                               baseline_m: float | None = None,
                               tolerance_px: float = GEOMETRY_TOLERANCE_PX) -> float:
    """严格校验一份 3×4 投影矩阵，返回 ``P[0][3]``。

    刻意不接受"大部分字段看起来合理"的矩阵：``P`` 是 ``(u, v, depth) → 3D`` 的唯一依据，
    任何一项不成立都会让整张地图的尺度或方向错掉，因此一律拒绝。
    """
    try:
        values = [float(v) for v in p]
    except (TypeError, ValueError) as exc:
        raise GeometryError("投影矩阵必须是 12 个可转成浮点的元素") from exc
    if len(values) != 12:
        raise GeometryError("投影矩阵必须是 12 个元素")
    if not all(math.isfinite(v) for v in values):
        raise GeometryError("投影矩阵含非有限值")
    if side not in ("left", "right"):
        raise GeometryError(f"side 只能是 left/right，收到 {side!r}")
    if int(width) <= 0 or int(height) <= 0:
        raise GeometryError("图像尺寸必须为正")

    fx, fy, cx, cy = values[0], values[5], values[2], values[6]
    if fx <= 0.0 or fy <= 0.0:
        raise GeometryError("fx/fy 必须为正")
    if values[1] != 0.0 or values[4] != 0.0 or values[7] != 0.0:
        raise GeometryError("投影矩阵不得含倾斜项")
    if values[11] != 0.0:
        raise GeometryError("投影矩阵齐次项 P[11] 必须为 0")
    if values[10] != 1.0:
        raise GeometryError("投影矩阵 P[10] 必须为 1")
    if not 0.0 <= cx < float(width) or not 0.0 <= cy < float(height):
        raise GeometryError("主点必须落在图像内")

    if side == "left":
        if values[3] != 0.0:
            raise GeometryError("左目投影矩阵不得带基线项（P[0][3] != 0）")
    else:
        if baseline_m is None:
            raise GeometryError("校验右目投影矩阵必须给出物理基线 baseline_m")
        expected = stereo_baseline_term(fx, baseline_m)
        if abs(values[3] - expected) > tolerance_px:
            raise GeometryError(
                f"右目 P[0][3] 应为 -fx·B = {expected!r}，收到 {values[3]!r}；"
                "基线是米制量，不随分辨率缩放"
            )
    return float(values[3])


def scaled_camera_info(info: dict, scale: float, *, output_size=None,
                       side: str | None = None) -> dict:
    """按**同一标定 + 同一次实际输出尺寸**缩放内参。

    * ``scale``：输出尺寸相对这份 info 尺寸的缩放比，必须落在 ``[0.5, 1.0]``；
    * ``output_size``：本次**实际**输出尺寸 ``(width, height)``。给了它就以此为准，
      并要求与 ``scale`` 推出的结果一致（1 px 内），避免"算一套、实际发另一套"。

    只缩放 ``fx/fy/cx/cy`` 与 ``width/height``；``baseline_m`` **原样保留**，
    并据此重算右目基线项。原 dict 不被修改。
    """
    # 先校验 scale：调用方可能连 info 都还没拿到，此时也要能明确报错。
    if isinstance(scale, bool) or not isinstance(scale, (int, float))\
            or not math.isfinite(scale) or not 0.5 <= scale <= 1.0:
        raise GeometryError("output_scale 必须处于 0.5 到 1.0")
    if info.get("width", 0) is None or info.get("height", 0) is None:
        raise GeometryError("图像尺寸必须为正")
    if info["width"] <= 0 or info["height"] <= 0:
        raise GeometryError("图像尺寸必须为正")

    result = dict(info)
    if output_size is None:
        target = (round(info["width"] * scale), round(info["height"] * scale))
    else:
        target = (int(output_size[0]), int(output_size[1]))
        implied = (round(info["width"] * scale), round(info["height"] * scale))
        if abs(target[0] - implied[0]) > 1 or abs(target[1] - implied[1]) > 1:
            raise GeometryError(
                f"实际输出尺寸 {target[0]}x{target[1]} 与 scale={scale} 推出的 "
                f"{implied[0]}x{implied[1]} 不一致：内参必须与深度图同一次输出")

    for index, size, keys in ((0, "width", ("fx", "cx")), (1, "height", ("fy", "cy"))):
        result[size] = target[index]
        ratio = target[index] / info[size]
        for key in keys:
            result[key] = float(info[key]) * ratio

    width, height = result["width"], result["height"]
    fx, fy, cx, cy = result["fx"], result["fy"], result["cx"], result["cy"]
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)):
        raise GeometryError("内参含非有限值")
    if fx <= 0.0 or fy <= 0.0:
        raise GeometryError("fx/fy 必须为正")
    if not 0.0 <= cx < width or not 0.0 <= cy < height:
        raise GeometryError("缩放后主点落在图像外")

    baseline = info.get("baseline_m")
    if baseline is not None:
        # 物理基线是米制刚体量：**不缩放**，只用来重算右目基线项。
        if isinstance(baseline, bool) or not isinstance(baseline, (int, float))\
                or not math.isfinite(baseline) or baseline < 0.0:
            raise GeometryError("物理基线必须是有限非负的米制量")
        result["baseline_m"] = float(baseline)
        resolved_side = side if side is not None else info.get("side")
        if "tx" in info and resolved_side is not None:
            result["tx"] = (0.0 if resolved_side == "left"
                            else stereo_baseline_term(fx, float(baseline)))
        if "p" in info and resolved_side is not None:
            matrix = [float(v) for v in info["p"]]
            if len(matrix) != 12:
                raise GeometryError("投影矩阵必须是 12 个元素")
            matrix[0], matrix[2], matrix[5], matrix[6] = fx, cx, fy, cy
            matrix[3] = (0.0 if resolved_side == "left"
                         else stereo_baseline_term(fx, float(baseline)))
            result["p"] = matrix
    return result


def ego_intrinsics(info: dict) -> dict:
    """EGO ``grid_map/{fx,fy,cx,cy}`` 参数：值以字符串给出（ROS 参数覆盖用）。"""
    values = {key: float(info[key]) for key in ("fx", "fy", "cx", "cy")}
    if not all(math.isfinite(v) for v in values.values()) or min(values["fx"], values["fy"]) <= 0:
        raise GeometryError("无效深度内参")
    return {key: str(value) for key, value in values.items()}


def camera_message_geometry(msg, now_s: float, max_age_s: float = 2.0, *,
                            frame_id: str = DEPTH_FRAME, expected_size=None) -> dict:
    """校验一份**深度** CameraInfo，返回本次实际几何（含尺寸与光学帧）。

    过期 / 未来时间戳 / 光学帧不符 / 尺寸不符 / 投影矩阵不合法，全部拒绝。
    """
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    age = now_s - stamp
    if not math.isfinite(age) or not 0 <= age <= max_age_s:
        raise GeometryError("CameraInfo 过期或时间戳在未来")
    if msg.header.frame_id != frame_id:
        raise GeometryError(f"CameraInfo 光学帧应为 {frame_id!r}，收到 {msg.header.frame_id!r}")
    if msg.width <= 0 or msg.height <= 0:
        raise GeometryError("CameraInfo 尺寸非正")
    if expected_size is not None and\
            (int(msg.width), int(msg.height)) != (int(expected_size[0]), int(expected_size[1])):
        raise GeometryError(
            f"CameraInfo 尺寸 {int(msg.width)}x{int(msg.height)} 与本次实际输出尺寸 "
            f"{int(expected_size[0])}x{int(expected_size[1])} 不一致")
    # 深度图必须是**校正后左目**：右目/带基线的矩阵会让反投影整体偏一个基线。
    validate_projection_matrix(msg.p, int(msg.width), int(msg.height), side="left")
    return {
        "width": int(msg.width),
        "height": int(msg.height),
        "frame_id": msg.header.frame_id,
        "fx": float(msg.p[0]),
        "fy": float(msg.p[5]),
        "cx": float(msg.p[2]),
        "cy": float(msg.p[6]),
    }


def camera_message_intrinsics(msg, now_s: float, max_age_s: float = 2.0, **kwargs) -> dict:
    """``camera_message_geometry`` 的字符串形式，直接用于 EGO 参数。"""
    return ego_intrinsics(camera_message_geometry(msg, now_s, max_age_s, **kwargs))


def depth_geometry_ready(geometry) -> bool:
    """是否已有**可用于融合**的深度几何（需求 3 的"未就绪"判据）。

    没有几何、字段不全、尺寸非正或投影矩阵非法，都返回 ``False``；
    调用方必须据此保持在"未就绪"，不得回退到默认值或上一次的内参。
    """
    if not geometry:
        return False
    try:
        validate_projection_matrix(
            [geometry["fx"], 0.0, geometry["cx"], 0.0,
             0.0, geometry["fy"], geometry["cy"], 0.0,
             0.0, 0.0, 1.0, 0.0],
            int(geometry["width"]), int(geometry["height"]), side="left")
    except (KeyError, TypeError, ValueError):
        return False
    return True


def calibration_identity(calibration_path) -> dict:
    """标定身份：内容 SHA-256 + 标定原图尺寸 + 左右 fx + 物理基线。

    三路内参只要来自**同一标定**，这份指纹就必须相同；不同时诊断里直接暴露差异。
    """
    path = Path(calibration_path)
    if not path.is_file():
        raise GeometryError(f"标定文件不存在：{path}")
    import numpy as np

    with np.load(path) as archive:
        data = {key: archive[key] for key in archive.files}
    for key in ("image_size", "K1", "K2", "T"):
        if key not in data:
            raise GeometryError(f"标定缺少必需字段：{key}")
    image_size = tuple(int(v) for v in data["image_size"])
    k1 = np.asarray(data["K1"], dtype=float).reshape(3, 3)
    k2 = np.asarray(data["K2"], dtype=float).reshape(3, 3)
    baseline_m = float(np.linalg.norm(np.asarray(data["T"], dtype=float).ravel()))
    if image_size[0] <= 0 or image_size[1] <= 0 or k1[0, 0] <= 0 or k2[0, 0] <= 0:
        raise GeometryError("标定尺寸或内参非法")
    return {
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "image_size": image_size,
        "fx_left": float(k1[0, 0]),
        "fx_right": float(k2[0, 0]),
        "baseline_m": baseline_m,
    }


def expected_depth_geometry(calibration_path, output_scale: float = 1.0) -> dict:
    """由标定推导深度节点**应当**发布的几何（校正后左目、深度输出尺寸）。"""
    from boom_birds_sensing.depth_core import make_processor, rectified_camera_info

    info = rectified_camera_info(make_processor(str(calibration_path)), DEPTH_FRAME)
    return scaled_camera_info(info, output_scale)


def depth_geometry_mismatch(received: dict, calibration_path,
                            output_scale: float = 1.0,
                            tolerance_px: float = GEOMETRY_TOLERANCE_PX) -> str:
    """比较实际收到的深度几何与标定推导的期望几何；返回空串表示三路同源。

    这是把 (1) stereo_source/depth_node 实际用的标定 与 (2) EGO 实际收到的
    CameraInfo 绑在一起的检查：不一致时 EGO 不得启动。
    """
    expected = expected_depth_geometry(calibration_path, output_scale)
    problems = []
    if (int(received["width"]), int(received["height"])) !=\
            (int(expected["width"]), int(expected["height"])):
        problems.append(
            f"尺寸 {int(received['width'])}x{int(received['height'])} != "
            f"{expected['width']}x{expected['height']}")
    for key in ("fx", "fy", "cx", "cy"):
        if abs(float(received[key]) - float(expected[key])) > tolerance_px:
            problems.append(f"{key} {float(received[key])!r} != {float(expected[key])!r}")
    if received.get("frame_id") not in (None, expected.get("frame_id")):
        problems.append(f"光学帧 {received['frame_id']!r} != {expected.get('frame_id')!r}")
    return "；".join(problems)


def main():
    """读取当前深度节点的几何，用于分段启动 EGO；超时/非法/不同源都拒绝启动。"""
    import argparse
    import time

    import rclpy
    from sensor_msgs.msg import CameraInfo

    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--calibration", default="",
                        help="标定 npz；给出后要求深度 CameraInfo 与该标定同源")
    parser.add_argument("--output-scale", type=float, default=1.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout 必须为有限正数")

    rclpy.init(args=[])
    node = rclpy.create_node("boom_birds_read_depth_geometry")
    #: 成功结果与最近一次拒绝原因分开记：两者都为空才说明"还没收到任何 CameraInfo"。
    received = []
    rejected = []

    def callback(msg):
        try:
            geometry = camera_message_geometry(
                msg, node.get_clock().now().nanoseconds * 1e-9)
            if args.calibration:
                problem = depth_geometry_mismatch(geometry, args.calibration, args.output_scale)
                if problem:
                    rejected.append(f"与标定 {args.calibration} 不同源：{problem}")
                    return
            received.append(ego_intrinsics(geometry))
        except ValueError as exc:
            rejected.append(str(exc))

    node.create_subscription(CameraInfo, DEFAULTS.camera_info_topic, callback, 10)
    deadline = time.monotonic() + args.timeout
    try:
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
        if not received:
            detail = f"；最近一次拒绝原因：{rejected[-1]}" if rejected else ""
            raise SystemExit(f"未收到新鲜且有效的深度 CameraInfo；拒绝启动 EGO{detail}")
        for key, value in received[0].items():
            print(f"{key}:={value}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
