"""标定工作流回归；所有输入均为测试数据，不连接设备。"""

import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
import pytest

from tools.vio_calibration.vio_calibration import (
    DatasetWriter,
    atomicJson,
    cameraChain,
    exportBag,
    exportCandidate,
    imuNoise,
    inspect,
    phaseAt,
    prepare,
    rigid,
    writeYaml,
)
from tools.vio_calibration.vio_calibration_cli import runSolver


@pytest.fixture
def source(tmp_path):
    path = tmp_path / "source.npz"
    k = np.array([[100.0, 0, 64], [0, 100.0, 48], [0, 0, 1]])
    np.savez(
        path,
        K1=k,
        K2=k,
        D1=np.array([0.01, 0, 0, 0, -0.02]),
        D2=np.zeros(5),
        R=np.eye(3),
        T=np.array([-0.067, 0, 0]),
        image_size=[128, 96],
    )
    return path


@pytest.fixture
def session(tmp_path, source):
    path = tmp_path / "session"
    manifest = prepare(path, source, 64, 48)
    manifest["phases"] = [["fixture", 1, "TEST ONLY"]]
    atomicJson(path / "session.json", manifest)
    writer = DatasetWriter(path)
    base = 1_792_000_000_000_000_123
    image = np.zeros((48, 64), np.uint8)
    for i in range(31):
        for cam in (0, 1):
            writer.image(
                cam, base + i * 50_000_000, base + i * 50_000_000 + 1000, image
            )
    for i in range(81):
        writer.imu(
            base + i * 20_000_000,
            base + i * 20_000_000 + 1000,
            [np.sin(i * 0.2 + j) * 0.1 for j in range(3)]
            + [np.sin(i * 0.2 + j) for j in range(3)],
        )
    for i in range(5):
        writer.event(base + i * 100_000_000, "board", dict(found=True))
    writer.event(base + 2_000_000_000, "finished", {})
    writer.close()
    return path


def noise(tmp_path):
    path = tmp_path / "noise.yaml"
    writeYaml(
        path,
        dict(
            accelerometer_noise_density=0.01,
            accelerometer_random_walk=0.001,
            gyroscope_noise_density=0.001,
            gyroscope_random_walk=0.0001,
            update_rate=50.0,
            source="TEST fixture only",
        ),
    )
    return path


def test_original_checkerboard_and_nonzero_k3_are_preserved(tmp_path, source):
    out = tmp_path / "prepared"
    m = prepare(out, source, 64, 48)
    assert m["board"] == dict(columns=11, rows=8, square_m=0.02)
    assert not m["camera_model_compatible"] and m["model_action"] == "fit_same_capture"
    assert not (out / "camchain.yaml").exists()
    assert np.load(out / "source-stereo.npz")["D1"][-1] == -0.02
    assert (out / "source-camera-model.yaml").is_file()
    assert m["raw_rotation_deg"] == 0
    with pytest.raises(FileExistsError):
        prepare(out, source, 64, 48)


def test_camera_geometry_scales_and_rejects_anisotropic(source):
    chain, ok = cameraChain(source, 64, 48)
    assert chain["cam0"]["intrinsics"] == [50.0, 50.0, 32.0, 24.0]
    assert chain["cam1"]["T_cn_cnm1"][0][3] == pytest.approx(-0.067)
    assert not ok
    with pytest.raises(ValueError):
        cameraChain(source, 64, 40)


def test_phase_boundaries():
    assert phaseAt(0)["name"] == "静止"
    assert phaseAt(5)["name"] == "覆盖画面"
    assert phaseAt(70)["name"] == "完成"


def test_data_screening_is_not_hardware_acceptance(session):
    r = inspect(session)
    assert r["status"] == "READY_FOR_SOLVER"
    assert not r["hardware_verified"]
    assert r["streams"]["imu"]["hz"] == pytest.approx(50)
    assert r["streams"]["cam0"]["hz"] == pytest.approx(20)


@pytest.mark.parametrize(
    "fault", ["gap", "stationary", "board_missing", "aborted", "incomplete"]
)
def test_invalid_capture_rejected(session, fault):
    with sqlite3.connect(session / "capture.sqlite3") as db:
        if fault == "gap":
            db.execute(
                "delete from imu where rowid in (select rowid from imu order by stamp_ns limit 21 offset 10)"
            )
        if fault == "stationary":
            db.execute("update imu set wx=0,wy=0,wz=0,ax=0,ay=0,az=-9.81")
        if fault == "board_missing":
            db.execute("delete from events where kind='board'")
        if fault == "aborted":
            db.execute("insert into events values(0,'abort','{}')")
        if fault == "incomplete":
            db.execute("delete from events where kind='finished'")
    assert inspect(session)["status"] == "REJECTED"


def test_committed_data_survives_process_exit(tmp_path, source):
    import subprocess, sys, os

    path = tmp_path / "crash"
    prepare(path, source, 64, 48)
    code = (
        "import os;from tools.vio_calibration.vio_calibration import DatasetWriter;"
        f"w=DatasetWriter({str(path)!r});w.imu(1792000000000000123,1792000000000001123,[0.,0.,0.,0.,0.,-9.81]);"
        "w.commit();os._exit(7)"
    )
    assert subprocess.run([sys.executable, "-c", code]).returncode == 7
    with sqlite3.connect(path / "capture.sqlite3") as db:
        assert db.execute("pragma quick_check").fetchone()[0] == "ok"
        assert (
            db.execute("select stamp_ns from imu").fetchone()[0] == 1792000000000000123
        )
    assert inspect(path)["status"] == "REJECTED"


def test_rosbag_roundtrip_preserves_ns_pixels_and_axes(session, tmp_path):
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore

    bag = tmp_path / "capture.bag"
    counts = exportBag(session, bag)
    assert counts == {"/cam0/image_raw": 31, "/cam1/image_raw": 31, "/imu0": 81}
    store = get_typestore(Stores.ROS1_NOETIC)
    stamps = []
    with Reader(bag) as reader:
        for conn, stamp, data in reader.messages():
            msg = store.deserialize_ros1(data, conn.msgtype)
            assert msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec == stamp
            stamps.append(stamp)
            if conn.topic == "/cam0/image_raw":
                assert msg.width == 64 and msg.height == 48 and not msg.data.any()
    assert min(stamps) == 1_792_000_000_000_000_123
    assert stamps == sorted(stamps)
    with pytest.raises(ValueError):
        exportBag(session, bag)


@pytest.mark.parametrize("bad", [-1, 0, None, float("nan")])
def test_noise_parameters_cannot_be_placeholders(tmp_path, bad):
    path = noise(tmp_path)
    import yaml

    data = yaml.safe_load(path.read_text())
    data["gyroscope_noise_density"] = bad
    writeYaml(path, data)
    with pytest.raises(ValueError):
        imuNoise(path, 50.0)


def test_solver_refuses_synthetic_and_missing_dependencies(
    session, tmp_path, monkeypatch
):
    manifest = json.loads((session / "session.json").read_text())
    manifest["synthetic"] = True
    atomicJson(session / "session.json", manifest)
    with pytest.raises(ValueError, match="合成"):
        runSolver(session, noise(tmp_path), tmp_path / "solve")
    manifest["synthetic"] = False
    atomicJson(session / "session.json", manifest)
    monkeypatch.setattr("shutil.which", lambda _: None)
    with pytest.raises(ValueError, match="后端"):
        runSolver(session, noise(tmp_path), tmp_path / "solve")


def test_candidate_keeps_full_resolution_npz_and_pending_body(session, tmp_path):
    from tools.vio_calibration.vio_calibration import loadYaml

    cams = {}
    base = np.eye(4)
    base[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    base[:3, 3] = [0.01, 0.02, 0.03]
    for i in (0, 1):
        relative = np.eye(4)
        relative[0, 3] = -0.067 * i
        t = relative @ base
        cams[f"cam{i}"] = dict(
            camera_model="pinhole",
            distortion_model="radtan",
            intrinsics=[50.0, 50.0, 32.0, 24.0],
            distortion_coeffs=[0.0] * 4,
            resolution=[64, 48],
            T_cam_imu=t.tolist(),
            timeshift_cam_imu=0.012,
            rostopic=f"/cam{i}/image_raw",
        )
    chain = tmp_path / "result.yaml"
    writeYaml(chain, cams)
    template = tmp_path / "template.yaml"
    writeYaml(template, dict(max_cameras=2, use_stereo=True))
    out = tmp_path / "candidate"
    result = exportCandidate(session, chain, noise(tmp_path), template, out)
    assert not result["hardware_verified"] and not result["body_transform_complete"]
    assert (out / "extrinsics.pending.yaml").is_file() and not (
        out / "extrinsics.yaml"
    ).exists()
    data = np.load(out / "stereo.npz")
    assert list(data["image_size"]) == [128, 96]
    assert data["K1"][0, 0] == pytest.approx(100)
    assert data["T"][0, 0] == pytest.approx(-0.067)
    left = loadYaml(out / "kalibr_imucam_chain.yaml")["cam0"]
    assert left["timeshift_cam_imu"] == pytest.approx(0.012)
    assert "T_cam_imu" not in left and "T_imu_cam" in left
    assert left["rostopic"] == "/boom_birds/stereo/left_raw"
    np.testing.assert_allclose(left["T_imu_cam"], np.linalg.inv(base))
    imu = loadYaml(out / "kalibr_imu_chain.yaml")["imu0"]
    assert imu["model"] == "calibrated"
    for key in ("Tw", "Ta", "R_IMUtoACC", "R_IMUtoGYRO"):
        np.testing.assert_array_equal(imu[key], np.eye(3))
    np.testing.assert_array_equal(imu["Tg"], np.zeros((3, 3)))


def test_bad_transform_rejected():
    t = np.eye(4)
    t[0, 0] = -1
    with pytest.raises(ValueError):
        rigid(t)


def test_recorder_has_no_control_publisher_service_or_serial():
    import ast

    path = Path(__file__).parents[1] / "vio_calibration_record.py"
    tree = ast.parse(path.read_text())
    names = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    assert not names.intersection(
        {"create_publisher", "create_client", "create_service"}
    )
    imports = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    assert not any(
        x and (x.startswith("pymavlink") or x.startswith("serial")) for x in imports
    )


@pytest.mark.parametrize("command", [22, 21, 176, 209, 310, 400])
def test_wire_filter_blocks_actuator_mode_and_arm(command):
    from pymavlink.dialects.v20 import common as mav
    from tools.vio_calibration.vio_calibration_inputs import allowed

    msg = mav.MAVLink(None).command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0)
    assert not allowed(msg)


def test_wire_filter_restricts_telemetry_parameters():
    from pymavlink.dialects.v20 import common as mav
    from tools.vio_calibration.vio_calibration_inputs import allowed

    e = mav.MAVLink(None)
    assert allowed(e.command_long_encode(1, 1, 511, 0, 105, 20000, 0, 0, 0, 0, 0))
    assert not allowed(e.command_long_encode(1, 1, 511, 0, 105, 1000, 0, 0, 0, 0, 0))
    assert not allowed(e.set_mode_encode(1, 1, 0))
    assert not allowed(e.heartbeat_encode(2, 12, 128, 0, 0))
    assert not allowed(e.param_set_encode(1, 1, b"TEST", 1, 9))


@pytest.mark.parametrize("final_sync_failure", [False, True])
def test_uart_emulator_restores_intervals_and_filters_tx(tmp_path, monkeypatch, final_sync_failure):
    import os, threading, time, select
    from pymavlink.dialects.v20 import common as mav
    from tools.vio_calibration.vio_calibration_inputs import TelemetryLink, PROFILE

    master, slave = os.openpty()
    device = os.ttyname(slave)
    os.close(slave)
    stop = threading.Event()
    seen = []
    intervals = {mid: 100000 for mid in PROFILE}
    originals = dict(intervals)

    def emulator():
        encoder = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        parser = mav.MAVLink(None)
        parser.robust_parsing = True

        def send(m):
            try:
                os.write(master, m.pack(encoder))
            except OSError:
                pass

        nextHb = 0
        while not stop.is_set():
            if time.monotonic() > nextHb:
                send(encoder.heartbeat_encode(2, 12, 29, 50593792, 0))
                send(
                    encoder.battery_status_encode(
                        0, 1, 1, 32767, [12300] + [65535] * 9, 100, 1, -1, 80
                    )
                )
                nextHb = time.monotonic() + 0.2
            try:
                if select.select([master], [], [], 0.01)[0]:
                    data = os.read(master, 65536)
                    for msg in parser.parse_buffer(data) or []:
                        seen.append(msg)
                        if msg.get_type() == "COMMAND_LONG":
                            mid = int(msg.param1)
                            if msg.command == 511:
                                intervals[mid] = int(msg.param2)
                            elif msg.command == 510:
                                send(
                                    encoder.message_interval_encode(mid, intervals[mid])
                                )
            except OSError:
                time.sleep(0.01)

    thread = threading.Thread(target=emulator, daemon=True)
    thread.start()
    link = None
    try:
        link = TelemetryLink(device, tmp_path / "link")
        link.configure()
        assert intervals[105] == 20000 and intervals[31] == -1
        dangerous = link.encoder.command_long_encode(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        assert not link.send(dangerous)
    finally:
        if link:
            if final_sync_failure:
                def failSync():
                    raise OSError("injected final sync failure")
                monkeypatch.setattr(link.journal, "_sync", failSync)
            link.close()
        stop.set()
        thread.join(timeout=2)
        os.close(master)
    assert intervals == originals
    assert not any(m.get_type() == "COMMAND_LONG" and m.command == 400 for m in seen)
    result = json.loads((tmp_path / "link/report.json").read_text())
    assert result["all_restored"] and not result["armed_seen"]
    assert result["journal"]["complete"] is not final_sync_failure
    if final_sync_failure:
        assert "injected final sync failure" in result["error"]


def test_opencv_yaml_is_readable_by_filestorage(tmp_path):
    path = tmp_path / "opencv.yaml"
    writeYaml(
        path,
        dict(imu0=dict(Tw=np.eye(3).tolist(), intrinsics=[1.0, 2.0, 3.0, 4.0])),
        opencv=True,
    )
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    try:
        assert storage.isOpened()
        node = storage.getNode("imu0").getNode("Tw")
        assert node.size() == 3 and node.at(0).at(0).real() == 1.0
    finally:
        storage.release()


def test_zero_k3_can_use_four_parameter_model(source):
    with np.load(source) as data:
        values = {key: data[key] for key in data.files}
    values["D1"][-1] = 0.0
    np.savez(source, **values)
    chain, compatible = cameraChain(source, 64, 48)
    assert compatible
    assert len(chain["cam0"]["distortion_coeffs"]) == 4
    assert len(chain["cam1"]["distortion_coeffs"]) == 4


def test_solver_orchestration_uses_bag_imu_topic_and_upstream_outputs(
    session, tmp_path, monkeypatch
):
    from types import SimpleNamespace
    from tools.vio_calibration.vio_calibration import loadYaml
    import tools.vio_calibration.vio_calibration_cli as cli

    noiseFile = noise(tmp_path)
    data = loadYaml(noiseFile)
    data["rostopic"] = "/wrong/input/topic"
    writeYaml(noiseFile, data)
    calls = []
    monkeypatch.setattr(cli.shutil, "which", lambda name: "/fake/" + name)

    def fakeRun(argv, **kwargs):
        calls.append(argv)
        folder = Path(kwargs["cwd"])
        if argv[0] == "kalibr_calibrate_cameras":
            (folder / "capture-camchain.yaml").write_text("TEST ONLY")
        else:
            assert (folder / "capture-camchain.yaml").is_file()
            assert loadYaml(folder / "imu.yaml")["rostopic"] == "/imu0"
            (folder / "capture-camchain-imucam.yaml").write_text("TEST ONLY")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(cli.subprocess, "run", fakeRun)
    report = runSolver(session, noiseFile, tmp_path / "solve")
    assert len(calls) == 2
    assert report["status"] == "SOLVED_REQUIRES_REVIEW"
    assert not report["hardware_verified"]
    assert report["result_chain"].endswith("capture-camchain-imucam.yaml")


def test_result_page_survives_capture_exit_and_rejects_control(session):
    import socket
    import threading
    import time
    import urllib.error
    import urllib.request
    from tools.vio_calibration.vio_calibration_inputs import serveResult

    atomicJson(session / "status.json", dict(state="ABORTED", error="fixture fault"))
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    worker = threading.Thread(target=serveResult, args=(session, port, 1.5))
    worker.start()
    try:
        for _ in range(50):
            try:
                response = urllib.request.urlopen(f"http://127.0.0.1:{port}/status", timeout=.2)
                break
            except urllib.error.URLError:
                time.sleep(.02)
        else:
            pytest.fail("result server did not start")
        with response:
            assert json.load(response) == dict(state="ABORTED", error="fixture fault")
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/") as page:
            assert "连接中断" in page.read().decode()
        request = urllib.request.Request(f"http://127.0.0.1:{port}/start", method="POST")
        with pytest.raises(urllib.error.HTTPError) as exc:
            urllib.request.urlopen(request)
        assert exc.value.code == 409
    finally:
        worker.join(timeout=3)
    assert not worker.is_alive()


def test_usb_power_keeps_armed_and_heartbeat_guards():
    import time
    from tools.vio_calibration.vio_calibration_inputs import TelemetryLink
    link = object.__new__(TelemetryLink)
    link.checkBattery = False
    link.armed = False
    link.hb = time.monotonic()
    link.battery = None
    link.batteryAt = None
    link.check()
    link.armed = True
    with pytest.raises(RuntimeError, match="已解锁"):
        link.check()
    link.armed = False
    link.hb -= 5
    with pytest.raises(RuntimeError, match="心跳过期"):
        link.check()
    link.hb = time.monotonic()
    link.checkBattery = True
    with pytest.raises(RuntimeError, match="电量数据过期"):
        link.check()


def test_capture_usb_power_is_explicit():
    from tools.vio_calibration.vio_calibration_cli import parser
    default = parser().parse_args(["capture", "--session", "/tmp/fixture"])
    usb = parser().parse_args(["capture", "--session", "/tmp/fixture", "--usb-powered"])
    assert default.usb_powered is False
    assert usb.usb_powered is True


def test_incomplete_capture_needs_explicit_acceptance():
    from tools.vio_calibration.vio_calibration_cli import solverCaptureChecks
    report = dict(status="REJECTED", checks=dict(imu_gaps=True,
        recording_completed=False, no_abort=False, duration=False))
    with pytest.raises(ValueError):
        solverCaptureChecks(dict(synthetic=False), report)
    assert set(solverCaptureChecks(dict(synthetic=False), report, True)) == {
        "recording_completed", "no_abort", "duration"}
    report["checks"]["imu_gaps"] = False
    with pytest.raises(ValueError):
        solverCaptureChecks(dict(synthetic=False), report, True)
    report["checks"]["imu_gaps"] = True
    with pytest.raises(ValueError):
        solverCaptureChecks(dict(synthetic=True), report, True)
