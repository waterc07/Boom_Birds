"""棋盘格 VIO 标定 CLI；求解结果仅导出候选，不修改部署。"""

from __future__ import annotations

import argparse
import json
import shlex
import shutil
import subprocess
from pathlib import Path

from .vio_calibration import (
    atomic_json,
    export_bag,
    export_candidate,
    imu_noise,
    inspect,
    prepare,
    write_yaml,
)


def solver_capture_checks(manifest, report, allow_incomplete_capture=False):
    failed = [key for key, value in report["checks"].items() if not value]
    allowed = {"recording_completed", "no_abort", "duration"} if allow_incomplete_capture else set()
    if manifest["synthetic"] or any(key not in allowed for key in failed):
        raise ValueError("采集未通过筛查或为合成输入，拒绝真实求解")
    if report["status"] != "READY_FOR_SOLVER" and not (allow_incomplete_capture and failed):
        raise ValueError("采集未通过筛查，拒绝真实求解")
    return failed


def run_solver(session, noise_file, out, prefix="", allow_incomplete_capture=False):
    session, out = Path(session).resolve(), Path(out).resolve()
    manifest = json.loads((session / "session.json").read_text())
    report = inspect(session)
    capture_exceptions = solver_capture_checks(manifest, report, allow_incomplete_capture)
    noise = imu_noise(noise_file, report["streams"]["imu"]["hz"])
    noise["rostopic"] = "/imu0"
    lead = shlex.split(prefix)
    tools = ["kalibr_calibrate_cameras", "kalibr_calibrate_imu_camera"]
    for tool in tools:
        executable = lead[0] if lead else tool
        if shutil.which(executable) is None:
            raise ValueError(
                "未安装求解后端：" + executable + "；采集和 bag 导出可独立运行"
            )
    out.mkdir(parents=True, exist_ok=False)
    bag = out / "capture.bag"
    export_bag(session, bag)
    shutil.copyfile(session / "target.yaml", out / "target.yaml")
    write_yaml(out / "imu.yaml", noise)
    commands = []
    fit = not manifest["camera_model_compatible"]
    if fit:
        commands.append(
            lead
            + [
                "kalibr_calibrate_cameras",
                "--bag",
                str(bag),
                "--topics",
                "/cam0/image_raw",
                "/cam1/image_raw",
                "--models",
                "pinhole-radtan",
                "pinhole-radtan",
                "--target",
                str(out / "target.yaml"),
                "--bag-freq",
                "5",
                "--dont-show-report",
            ]
        )
        chain = out / "capture-camchain.yaml"
    else:
        shutil.copyfile(session / "camchain.yaml", out / "camchain.yaml")
        chain = out / "camchain.yaml"
    last = lead + [
        "kalibr_calibrate_imu_camera",
        "--bag",
        str(bag),
        "--cam",
        str(chain),
        "--imu",
        str(out / "imu.yaml"),
        "--target",
        str(out / "target.yaml"),
        "--dont-show-report",
    ]
    commands.append(last)
    record = dict(
        status="RUNNING",
        capture_inspection_status=report["status"],
        accepted_incomplete_checks=capture_exceptions,
        commands=commands,
        camera_refit=fit,
        hardware_verified=False,
        note="仅输出候选。检查 PDF 中的残差、偏置、时间间隔及棋盘格姿态翻转。",
    )
    atomic_json(out / "solver.json", record)
    try:
        for i, argv in enumerate(commands):
            with (out / f"solver-{i}.log").open("w") as log:
                result = subprocess.run(
                    argv, cwd=out, stdout=log, stderr=subprocess.STDOUT, check=False
                )
            if result.returncode:
                raise RuntimeError(
                    f"Kalibr 退出码 {result.returncode}，见 solver-{i}.log"
                )
            if i == 0 and fit and not chain.is_file():
                raise RuntimeError("缺少 capture-camchain.yaml，检查后端版本与输出")
        result_chain = out / "capture-camchain-imucam.yaml"
        if not result_chain.is_file():
            raise RuntimeError("缺少 capture-camchain-imucam.yaml，检查后端输出")
        record.update(status="SOLVED_REQUIRES_REVIEW", result_chain=str(result_chain))
    except BaseException as exc:
        record.update(status="FAILED", error=str(exc))
        raise
    finally:
        atomic_json(out / "solver.json", record)
    return record


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    a = sub.add_parser("prepare", help="创建新会话，复用原棋盘格与双目标定资料")
    a.add_argument("--out", required=True)
    a.add_argument("--calibration", required=True)
    a.add_argument("--config")
    a.add_argument("--width", type=int, default=640)
    a.add_argument("--height", type=int, default=480)
    a = sub.add_parser(
        "capture", help="受限串口＋传感器＋浏览器的一键采集；只用于未解锁标定"
    )
    a.add_argument("--session", required=True)
    a.add_argument("--device", default="/dev/ttyAMA0")
    a.add_argument("--port", type=int, default=8084)
    a.add_argument("--maximum-seconds", type=int, default=180)
    a.add_argument("--usb-powered", action="store_true", help="飞控 USB 供电，本会话不检查电池")
    a = sub.add_parser("record", help="只订阅已有的受限相机/IMU输入，等待浏览器开始")
    a.add_argument("--session", required=True)
    a.add_argument("--port", type=int, default=8084)
    a = sub.add_parser("inspect", help="检查已有数据；不打开设备")
    a.add_argument("--session", required=True)
    a = sub.add_parser("bag", help="导出带原始ns时间戳的 ROS1 bag")
    a.add_argument("--session", required=True)
    a.add_argument("--out", required=True)
    a = sub.add_parser("solve", help="调用 Kalibr；在新目录保存全部输出")
    a.add_argument("--allow-incomplete-capture", action="store_true",
        help="明确采用提前中止的数据；只豁免完成/中止/时长，其他检查仍须通过")
    a.add_argument("--session", required=True)
    a.add_argument("--imu-noise", required=True)
    a.add_argument("--out", required=True)
    a.add_argument(
        "--kalibr-prefix",
        default="",
        help="例如 rosrun kalibr；按参数数组执行，不使用shell",
    )
    a = sub.add_parser("candidate", help="从求解结果生成独立 OpenVINS/双目候选")
    a.add_argument("--session", required=True)
    a.add_argument("--chain", required=True)
    a.add_argument("--imu-noise", required=True)
    a.add_argument(
        "--template",
        required=True,
        help="非合成 estimator 模板；相机/IMU参数会全部替换",
    )
    a.add_argument(
        "--body-extrinsics", help="有来源的 T_I_B；缺失时仅生成 pending 文件"
    )
    a.add_argument("--out", required=True)
    a = sub.add_parser("doctor", help="检查本机软件，不连接设备")
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "prepare":
            result = prepare(
                args.out, args.calibration, args.width, args.height, args.config
            )
        elif args.command == "capture":
            from .vio_calibration_inputs import capture

            if not 1 <= args.port <= 65535:
                raise ValueError("端口非法")
            return capture(args.session, args.device, args.port, args.maximum_seconds,
                usb_powered=args.usb_powered)
        elif args.command == "record":
            from .vio_calibration_record import run_record

            if not 1 <= args.port <= 65535:
                raise ValueError("端口非法")
            return run_record(args.session, port=args.port)
        elif args.command == "inspect":
            result = inspect(args.session)
        elif args.command == "bag":
            result = export_bag(args.session, args.out)
        elif args.command == "solve":
            result = run_solver(
                args.session, args.imu_noise, args.out, args.kalibr_prefix,
                allow_incomplete_capture=args.allow_incomplete_capture,
            )
        elif args.command == "candidate":
            result = export_candidate(
                args.session,
                args.chain,
                args.imu_noise,
                args.template,
                args.out,
                args.body_extrinsics,
            )
        else:
            import importlib.util

            result = dict(
                packages={
                    x: importlib.util.find_spec(x) is not None
                    for x in (
                        "rclpy",
                        "cv_bridge",
                        "cv2",
                        "numpy",
                        "yaml",
                        "mavros_msgs",
                        "pymavlink",
                        "rosbags",
                    )
                },
                executables={x: shutil.which(x) for x in ("ros2", "fuser")},
                kalibr={
                    x: shutil.which(x)
                    for x in ("kalibr_calibrate_cameras", "kalibr_calibrate_imu_camera")
                },
                scope="软件检查；不证明实机采集或标定通过",
            )
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 2 if args.command == "inspect" and result["status"] == "REJECTED" else 0
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print(json.dumps(dict(status="BLOCKED", reason=str(exc)), ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
