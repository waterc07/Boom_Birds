#!/usr/bin/env python3
"""采集安装方向校正后的预览图；原图保留标定坐标。"""
import argparse
import json
import pathlib
import sys
import threading
import types

import cv2


def orient(image, angle):
    codes = {90: cv2.ROTATE_90_COUNTERCLOCKWISE,
             180: cv2.ROTATE_180, 270: cv2.ROTATE_90_CLOCKWISE}
    if angle == 0:
        return image
    if angle not in codes:
        raise ValueError("rotation must be 0, 90, 180 or 270")
    return cv2.rotate(image, codes[angle])


def write_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"cannot write {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=pathlib.Path,
                        default=pathlib.Path(__file__).with_name("camera_orientation.json"))
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--camera", choices=("both", "stereo", "downward"), default="both")
    parser.add_argument("--stereo-device",
                        default="/dev/v4l/by-id/usb-Generic_USB_Camera_20000001-video-index0")
    parser.add_argument("--mipi-tools",
                        default="/home/gmaster/boombirds/tools/mipi-calibration-20261008")
    parser.add_argument("--mipi-device", default="/dev/video0")
    parser.add_argument("--mipi-media", default="/dev/media1")
    parser.add_argument("--mipi-subdev", default="/dev/v4l-subdev2")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if config["scope"] != "preview_and_export_only":
        raise ValueError("unsupported orientation scope")
    if not config["stereo"]["preserve_stream_slot_order"]:
        raise ValueError("不支持交换双目通道")
    for angle in (config["stereo"]["per_view_rotation_ccw_deg"],
                  config["downward"]["rotation_ccw_deg"]):
        if type(angle) is not int or angle not in (0, 90, 180, 270):
            raise ValueError("旋转角度必须为 0/90/180/270 度整数")
    # 不覆盖旧采集记录。
    args.output.mkdir(parents=True, exist_ok=False)
    result = {"orientation": config, "config": str(args.config.resolve()),
              "raw_geometry_unchanged": True, "cameras": {}}
    try:
        if args.camera in ("both", "stereo"):
            cap = cv2.VideoCapture(args.stereo_device, cv2.CAP_V4L2)
            try:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
                for _ in range(12):
                    ok, frame = cap.read()
                if not ok or frame.shape[:2] != (960, 2560):
                    raise RuntimeError("stereo capture failed or unexpected dimensions")
                split = frame.shape[1] // 2
                angle = config["stereo"]["per_view_rotation_ccw_deg"]
                # 分目旋转；整幅拼接图旋转会交换两个通道。
                corrected = cv2.hconcat([orient(frame[:, :split], angle),
                                        orient(frame[:, split:], angle)])
                write_image(args.output / "stereo-raw.jpg", frame)
                write_image(args.output / "stereo.jpg", corrected)
                result["cameras"]["stereo"] = {"device": args.stereo_device,
                                                "shape": list(corrected.shape)}
            finally:
                cap.release()
        if args.camera in ("both", "downward"):
            sys.path.insert(0, args.mipi_tools)
            from mipi_raw import RawCamera, demosaic
            params = types.SimpleNamespace(device=args.mipi_device, media=args.mipi_media,
                                           subdev=args.mipi_subdev, width=1640, height=1232,
                                           exposure=1600, gain=128)
            camera = RawCamera(params)
            try:
                camera.configure()
                for _ in range(5):
                    payload = camera.read(threading.Event())
                frame = demosaic(payload, params.width, params.height, camera.stride)
                corrected = orient(frame, config["downward"]["rotation_ccw_deg"])
                write_image(args.output / "downward-raw.jpg", frame)
                write_image(args.output / "downward.jpg", corrected)
                result["cameras"]["downward"] = {"device": args.mipi_device,
                                                  "shape": list(corrected.shape)}
            finally:
                camera.close()
    except Exception as exc:
        result["error"] = str(exc)
        raise
    finally:
        (args.output / "capture.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
