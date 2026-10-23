"""棋盘格相机—IMU标定会话与离线数据；不访问飞控出口。"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import sqlite3
from pathlib import Path

import cv2
from stereo_depth.orientation import SENSOR, UPRIGHT, imageOrientation, cameraMatrix, scaleMatrix

import numpy as np
import yaml

PHASES = [
    ("静止", 5, "整机保持静止，棋盘格清晰可见。"),
    ("覆盖画面", 15, "缓慢改变距离和位置，让棋盘格覆盖画面中心及四周，避免翻转。"),
    ("平移", 15, "整机小幅左右、上下、前后移动，棋盘格保持可见。"),
    ("旋转", 15, "整机分别抬头低头、左右倾斜、机头左右转，避免急甩。"),
    ("组合运动", 15, "平移与小角度旋转组合，保持棋盘格清晰，不翻倒机体。"),
    ("结束静止", 5, "整机静止，等待保存完成。"),
]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomicJson(path, data):
    path = Path(path)
    temp = path.with_suffix(path.suffix + ".tmp")
    with temp.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)


def loadYaml(path):
    text = Path(path).read_text(encoding="utf-8")
    return yaml.safe_load(
        "\n".join(x for x in text.splitlines() if not x.startswith("%YAML"))
    )


class _OpenCVDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)


def writeYaml(path, data, opencv=False):
    if opencv:
        # FileStorage 不接受 PyYAML 默认的 indentless sequence。
        text = "%YAML:1.0\n---\n" + yaml.dump(
            data, Dumper=_OpenCVDumper, sort_keys=False, default_flow_style=None
        )
    else:
        text = yaml.safe_dump(data, allow_unicode=True, sort_keys=False)
    Path(path).write_text(text, encoding="utf-8")


def phaseAt(elapsed, phases=PHASES):
    for name, seconds, instruction in phases:
        if elapsed < seconds:
            return dict(
                name=name,
                remaining_s=math.ceil(seconds - elapsed),
                instruction=instruction,
            )
        elapsed -= seconds
    return dict(name="完成", remaining_s=0, instruction="采集已结束，请等待写盘完成。")


def cameraChain(npz, width, height):
    with np.load(npz, allow_pickle=False) as data:
        size = np.asarray(data["image_size"], dtype=float)
        scales = np.array([width, height]) / size
        if (
            not np.isfinite(scales).all()
            or not np.isclose(*scales)
            or not 0 < scales[0] <= 1
        ):
            raise ValueError("标定图像必须按相同比例缩小")
        chain = {}
        compatible = True
        for i in range(2):
            k = cameraMatrix(data, i+1, (width, height))
            d = np.asarray(data[f"D{i+1}"], dtype=float).reshape(-1)
            if (
                k.shape != (3, 3)
                or not np.isfinite(k).all()
                or k[0, 0] <= 0
                or k[1, 1] <= 0
            ):
                raise ValueError("相机内参非法")
            if len(d) not in (4, 5) or not np.isfinite(d).all():
                raise ValueError("只支持原始 4/5 项 radtan 内参")
            valid = len(d) == 4 or abs(d[4]) <= 1e-12
            compatible &= valid
            chain[f"cam{i}"] = dict(
                image_orientation=imageOrientation(data),
                intrinsics_scale_offset=float(data.get("intrinsics_scale_offset", 0.)),
                camera_model="pinhole",
                distortion_model="radtan",
                intrinsics=(
                    np.array([k[0, 0], k[1, 1], k[0, 2], k[1, 2]])
                ).tolist(),
                distortion_coeffs=(d[:4] if valid else d).tolist(),
                resolution=[width, height],
                rostopic=f"/cam{i}/image_raw",
                cam_overlaps=[1 - i],
            )
        relative = np.eye(4)
        relative[:3, :3] = data["R"]
        relative[:3, 3] = np.asarray(data["T"]).reshape(3)
        rigid(relative)
        chain["cam1"]["T_cn_cnm1"] = relative.tolist()
    return chain, bool(compatible)


def prepare(out, calibration, width=640, height=480, config=None):
    out = Path(out).resolve()
    cfg = loadYaml(config) if config else {}
    board = cfg.get("board", dict(columns=11, rows=8, square_m=0.02))
    if (
        type(board["columns"]) is not int
        or type(board["rows"]) is not int
        or min(board["columns"], board["rows"]) < 3
        or not math.isfinite(board["square_m"])
        or board["square_m"] <= 0
    ):
        raise ValueError("棋盘格参数非法")
    chain, compatible = cameraChain(calibration, width, height)
    out.mkdir(parents=True, exist_ok=False)
    import shutil

    shutil.copyfile(calibration, out / "source-stereo.npz")
    target = dict(
        target_type="checkerboard",
        targetCols=board["columns"],
        targetRows=board["rows"],
        rowSpacingMeters=board["square_m"],
        colSpacingMeters=board["square_m"],
    )
    writeYaml(out / "target.yaml", target)
    # 非零 k3 原样保存为资料，不生成误标为 Kalibr 四项模型的输入。
    writeYaml(
        out / ("camchain.yaml" if compatible else "source-camera-model.yaml"), chain
    )
    manifest = dict(
        schema=1,
        board=board,
        width=width,
        height=height,
        image_topics=cfg.get(
            "image_topics",
            ["/boom_birds/stereo/left_raw", "/boom_birds/stereo/right_raw"],
        ),
        imu_topic="/boom_birds/imu",
        phases=cfg.get("phases", PHASES),
        image_orientation=chain["cam0"]["image_orientation"],
        intrinsics_scale_offset=chain["cam0"]["intrinsics_scale_offset"],
        raw_rotation_deg=180 if chain["cam0"]["image_orientation"] == UPRIGHT else 0,
        raw_swap_eyes=chain["cam0"]["image_orientation"] == UPRIGHT,
        source_stereo_sha256=digest(out / "source-stereo.npz"),
        camera_model_compatible=compatible,
        synthetic=False,
        hardware_verified=False,
        recommended_imu_hz=200,
        program_sha256={
            p.name: digest(p) for p in Path(__file__).parent.glob("vio_calibration*.py")
        },
        opencv_version=cv2.__version__,
        model_action="reuse" if compatible else "fit_same_capture",
        gates=cfg.get(
            "gates",
            dict(
                min_image_hz=15.0,
                min_imu_hz=45.0,
                max_image_gap_s=0.2,
                max_imu_gap_s=0.05,
                min_board_fraction=0.6,
                min_gyro_std=0.02,
                min_accel_std=0.1,
            ),
        ),
    )
    atomicJson(out / "session.json", manifest)
    writeYaml(
        out / "imu-noise.required.yaml",
        dict(
            accelerometer_noise_density=None,
            accelerometer_random_walk=None,
            gyroscope_noise_density=None,
            gyroscope_random_walk=None,
            update_rate=None,
            rostopic="/imu0",
            source="待填写实际测量来源，不能使用示例值",
        ),
    )
    return manifest


class DatasetWriter:
    """单写线程持有；每批事务提交，WAL 保留断电前已提交数据。"""

    def __init__(self, session):
        self.path = Path(session) / "capture.sqlite3"
        self.db = sqlite3.connect(self.path)
        if self.db.execute("select count(*) from sqlite_master").fetchone()[0]:
            self.db.close()
            raise ValueError("会话已有采集数据，不能覆盖")
        self.db.execute("pragma journal_mode=WAL")
        self.db.execute("pragma synchronous=FULL")
        self.db.executescript("""
            create table images(cam integer, stamp_ns integer, received_ns integer,
                width integer, height integer, png blob, primary key(cam,stamp_ns));
            create table imu(stamp_ns integer primary key, received_ns integer,
                wx real,wy real,wz real,ax real,ay real,az real);
            create table events(received_ns integer, kind text, payload text);
        """)
        self.last = {}

    def _stamp(self, key, stamp):
        if type(stamp) is not int or stamp <= self.last.get(key, 0):
            raise ValueError(f"{key} 时间戳非递增")
        self.last[key] = stamp

    def image(self, cam, stamp, received, gray):
        self._stamp(f"cam{cam}", stamp)
        if gray.ndim != 2 or gray.dtype != np.uint8:
            raise ValueError("需要与会话标定方向一致的 mono8 图像")
        ok, data = cv2.imencode(".png", gray, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            raise ValueError("PNG 编码失败")
        h, w = gray.shape
        self.db.execute(
            "insert into images values(?,?,?,?,?,?)",
            (cam, stamp, received, w, h, data.tobytes()),
        )

    def imu(self, stamp, received, values):
        self._stamp("imu", stamp)
        if len(values) != 6 or not np.isfinite(values).all():
            raise ValueError("IMU 需要六个有限数")
        self.db.execute(
            "insert into imu values(?,?,?,?,?,?,?,?)", (stamp, received, *values)
        )

    def event(self, received, kind, data):
        self.db.execute(
            "insert into events values(?,?,?)",
            (received, kind, json.dumps(data, ensure_ascii=False, allow_nan=False)),
        )

    def commit(self):
        self.db.commit()

    def close(self):
        self.db.commit()
        self.db.execute("pragma wal_checkpoint(TRUNCATE)")
        self.db.close()


def readDb(session):
    path = Path(session) / "capture.sqlite3"
    return sqlite3.connect(f"file:{path}?mode=ro", uri=True)


def streamMetrics(stamps):
    values = np.asarray(stamps, dtype=np.int64)
    gaps = np.diff(values) * 1e-9
    return dict(
        count=len(values),
        hz=(
            float((len(values) - 1) * 1e9 / (values[-1] - values[0]))
            if len(values) > 1 and values[-1] > values[0]
            else 0.0
        ),
        max_gap_s=float(gaps.max()) if len(gaps) else None,
        non_increasing=int(np.sum(gaps <= 0)),
    )


def inspect(session):
    session = Path(session)
    manifest = json.loads((session / "session.json").read_text())
    if digest(session / "source-stereo.npz") != manifest["source_stereo_sha256"]:
        raise ValueError("会话双目标定文件已被修改")
    with readDb(session) as db:
        if db.execute("pragma quick_check").fetchone()[0] != "ok":
            raise ValueError("SQLite 检查失败")
        streams = {}
        for i in range(len(manifest["image_topics"])):
            stamps = [
                r[0]
                for r in db.execute(
                    "select stamp_ns from images where cam=? order by rowid", (i,)
                )
            ]
            streams[f"cam{i}"] = streamMetrics(stamps)
        rows = np.asarray(
            list(db.execute("select wx,wy,wz,ax,ay,az from imu order by rowid"))
        )
        events = [
            (kind, json.loads(payload))
            for kind, payload in db.execute("select kind,payload from events")
        ]
    with readDb(session) as db:
        streams["imu"] = streamMetrics(
            [r[0] for r in db.execute("select stamp_ns from imu order by rowid")]
        )
        ranges = [db.execute("select min(stamp_ns),max(stamp_ns) from imu").fetchone()]
        ranges += [
            db.execute(
                "select min(stamp_ns),max(stamp_ns) from images where cam=?", (i,)
            ).fetchone()
            for i in range(len(manifest["image_topics"]))
        ]
    motion = (
        dict(
            gyro_std=np.std(rows[:, :3], axis=0).tolist(),
            accel_std=np.std(rows[:, 3:6], axis=0).tolist(),
        )
        if len(rows)
        else dict(gyro_std=[0.0] * 3, accel_std=[0.0] * 3)
    )
    detections = [x["found"] for k, x in events if k == "board"]
    fraction = float(np.mean(detections)) if detections else 0.0
    g = manifest["gates"]
    checks = {}
    for name, data in streams.items():
        checks[name + "_rate"] = (
            data["hz"] >= g["min_imu_hz" if name == "imu" else "min_image_hz"]
        )
        checks[name + "_gaps"] = (
            data["max_gap_s"] is not None
            and data["max_gap_s"]
            <= g["max_imu_gap_s" if name == "imu" else "max_image_gap_s"]
        )
        checks[name + "_monotonic"] = data["non_increasing"] == 0
    checks["board_visible"] = fraction >= g["min_board_fraction"]
    checks["rotation_each_axis"] = min(motion["gyro_std"]) >= g["min_gyro_std"]
    checks["translation_excitation"] = max(motion["accel_std"]) >= g["min_accel_std"]
    checks["recording_completed"] = any(k == "finished" for k, _ in events)
    checks["no_abort"] = not any(k == "abort" for k, _ in events)
    checks["time_overlap"] = all(
        a is not None and b is not None for a, b in ranges
    ) and max(a for a, _ in ranges) < min(b for _, b in ranges)
    expected = sum(float(x[1]) for x in manifest["phases"])
    checks["duration"] = (
        bool(checks["time_overlap"])
        and (min(b for _, b in ranges) - max(a for a, _ in ranges)) * 1e-9
        >= 0.9 * expected
    )
    with readDb(session) as db:
        paired = db.execute(
            "select count(*) from images a join images b on a.stamp_ns=b.stamp_ns where a.cam=0 and b.cam=1"
        ).fetchone()[0]
    checks["stereo_same_frame"] = paired >= 0.95 * max(
        streams["cam0"]["count"], streams["cam1"]["count"], 1
    )
    result = dict(
        status="READY_FOR_SOLVER" if all(checks.values()) else "REJECTED",
        checks=checks,
        streams=streams,
        board_fraction=fraction,
        motion=motion,
        synthetic=manifest["synthetic"],
        hardware_verified=False,
        note="运动阈值仅作采集筛查，不证明可观性、标定精度或 VIO 通过。",
    )
    atomicJson(session / "inspection.json", result)
    return result


def exportBag(session, output):
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore

    session, output = Path(session), Path(output)
    if output.exists():
        raise ValueError("不覆盖已有 bag")
    types = get_typestore(Stores.ROS1_NOETIC)
    make = types.types
    Time, Header = make["builtin_interfaces/msg/Time"], make["std_msgs/msg/Header"]
    Image, Imu = make["sensor_msgs/msg/Image"], make["sensor_msgs/msg/Imu"]
    Vec, Quat = make["geometry_msgs/msg/Vector3"], make["geometry_msgs/msg/Quaternion"]

    def header(stamp, frame, seq):
        return Header(seq, Time(stamp // 10**9, stamp % 10**9), frame)

    temp = output.with_suffix(".partial.bag")
    if temp.exists():
        raise ValueError("存在未完成 bag，请另选输出路径")
    manifest = json.loads((session / "session.json").read_text())
    counts = {}
    with Writer(temp) as writer, readDb(session) as db:
        connections = {
            str(i): writer.add_connection(
                f"/cam{i}/image_raw", Image.__msgtype__, typestore=types
            )
            for i in range(len(manifest["image_topics"]))
        }
        imuCon = writer.add_connection("/imu0", Imu.__msgtype__, typestore=types)
        # 按时间合并，避免某条连接的记录时间跨流回退。
        query = """select stamp_ns,0 as kind,cam,rowid from images
                   union all select stamp_ns,1,-1,rowid from imu order by stamp_ns"""
        for seq, (stamp, kind, cam, rowid) in enumerate(db.execute(query)):
            if kind == 0:
                w, h, png = db.execute(
                    "select width,height,png from images where rowid=?", (rowid,)
                ).fetchone()
                image = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_GRAYSCALE)
                if image is None or image.shape != (h, w):
                    raise ValueError("图像损坏")
                msg = Image(
                    header(stamp, f"cam{cam}", seq),
                    h,
                    w,
                    "mono8",
                    0,
                    w,
                    image.reshape(-1),
                )
                conn = connections[str(cam)]
            else:
                values = db.execute(
                    "select wx,wy,wz,ax,ay,az from imu where rowid=?", (rowid,)
                ).fetchone()
                oc = np.zeros(9)
                oc[0] = -1.0
                msg = Imu(
                    header(stamp, "imu", seq),
                    Quat(0.0, 0.0, 0.0, 1.0),
                    oc,
                    Vec(*values[:3]),
                    np.zeros(9),
                    Vec(*values[3:]),
                    np.zeros(9),
                )
                conn = imuCon
            writer.write(conn, stamp, types.serialize_ros1(msg, msg.__msgtype__))
            counts[conn.topic] = counts.get(conn.topic, 0) + 1
    os.replace(temp, output)
    atomicJson(
        output.with_suffix(".manifest.json"),
        dict(
            bag_sha256=digest(output),
            counts=counts,
            session_sha256=digest(session / "session.json"),
            synthetic=manifest["synthetic"],
            timestamp_policy="original header stamps, integer nanoseconds",
        ),
    )
    return counts


def rigid(value):
    a = np.asarray(value, dtype=float)
    if (
        a.shape != (4, 4)
        or not np.isfinite(a).all()
        or not np.allclose(a[3], [0, 0, 0, 1], atol=1e-8)
        or not np.allclose(a[:3, :3].T @ a[:3, :3], np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(a[:3, :3]), 1.0, atol=1e-5)
    ):
        raise ValueError("外参不是有效 SE(3) 矩阵")
    return a


def imuNoise(path, rate):
    data = loadYaml(path)
    if "imu0" in data:
        data = data["imu0"]
    for name in (
        "accelerometer_noise_density",
        "accelerometer_random_walk",
        "gyroscope_noise_density",
        "gyroscope_random_walk",
    ):
        value = data.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("缺少正的实测 IMU 噪声参数：" + name)
    if not str(data.get("source", "")).strip() or "待填写" in str(data["source"]):
        raise ValueError("IMU 噪声参数必须有测量来源")
    if not math.isfinite(float(data.get("update_rate", 0))) or not np.isclose(
        data["update_rate"], rate, rtol=0.1
    ):
        raise ValueError("IMU 噪声配置 update_rate 与实际采集频率不匹配")
    return data


def exportCandidate(session, chainPath, noisePath, template, out, bodyPath=None):
    session, out = Path(session), Path(out)
    manifest = json.loads((session / "session.json").read_text())
    stats = inspect(session)
    if stats["status"] != "READY_FOR_SOLVER" or manifest["synthetic"]:
        raise ValueError("拒绝导出未通过采集筛查或合成数据的实机候选")
    cams = loadYaml(chainPath)
    noise = imuNoise(noisePath, stats["streams"]["imu"]["hz"])
    model = []
    for i in range(2):
        cam = cams[f"cam{i}"]
        k, d = np.asarray(cam["intrinsics"], float), np.asarray(
            cam["distortion_coeffs"], float
        )
        if (
            cam["camera_model"] != "pinhole"
            or cam["distortion_model"] != "radtan"
            or k.shape != (4,)
            or d.shape != (4,)
            or not np.isfinite(k).all()
            or not np.isfinite(d).all()
            or min(k[:2]) <= 0
            or cam["resolution"] != [manifest["width"], manifest["height"]]
        ):
            raise ValueError("求解相机模型与采集尺寸不匹配")
        transform = rigid(cam["T_cam_imu"])
        shift = float(cam["timeshift_cam_imu"])
        if not math.isfinite(shift):
            raise ValueError("时间偏移非法")
        model.append((k, d, transform, shift))
    if abs(model[0][3] - model[1][3]) > 1e-4:
        raise ValueError("同帧双目求解的时间偏移不一致，须人工检查")
    relative = model[1][2] @ np.linalg.inv(model[0][2])
    if not 0.01 <= np.linalg.norm(relative[:3, 3]) <= 0.3:
        raise ValueError("双目基线不在本机候选检查范围内")
    config = loadYaml(template)
    # OpenCV 模板中的 1e-6 被 PyYAML 识别为字符串；保持原数值类型。
    for key, value in config.items():
        if isinstance(value, str) and re.fullmatch(
            r"[-+]?\d+(?:\.\d*)?[eE][-+]?\d+", value
        ):
            config[key] = float(value)
    if config.get("boombirds_test_only"):
        raise ValueError("不能使用合成 estimator 模板")
    body = None
    if bodyPath:
        body = loadYaml(bodyPath)
        body["T_I_B"] = rigid(body["T_I_B"]).tolist()
        if not str(body.get("source", "")).strip():
            raise ValueError("机体系外参缺少来源")
    out.mkdir(parents=True, exist_ok=False)
    matrices = [
        np.array([[k[0], 0, k[2]], [0, k[1], k[3]], [0, 0, 1.0]])
        for k, _, _, _ in model
    ]
    with np.load(session / "source-stereo.npz", allow_pickle=False) as source:
        size = tuple(int(x) for x in source["image_size"])
    scale = size[0] / manifest["width"]
    fullMatrices = [k.copy() for k in matrices]
    offset = float(manifest.get("intrinsics_scale_offset", 0.))
    fullMatrices = [scaleMatrix(k, (manifest["width"], manifest["height"]), size, offset)
                     for k in matrices]
    r1, r2, p1, p2, q, *_ = cv2.stereoRectify(
        fullMatrices[0],
        model[0][1],
        fullMatrices[1],
        model[1][1],
        size,
        relative[:3, :3],
        relative[:3, 3],
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0,
    )
    np.savez_compressed(
        out / "stereo.npz",
        K1=fullMatrices[0],
        K2=fullMatrices[1],
        D1=model[0][1],
        D2=model[1][1],
        R=relative[:3, :3],
        T=relative[:3, 3, None],
        image_size=np.array(size),
        square_size_m=manifest["board"]["square_m"],
        image_orientation=manifest.get("image_orientation", SENSOR),
        intrinsics_scale_offset=offset,
        R1=r1,
        R2=r2,
        P1=p1,
        P2=p2,
        Q=q,
    )
    cam = dict(cams["cam0"])
    cam["image_orientation"] = manifest.get("image_orientation", SENSOR)
    cam["intrinsics_scale_offset"] = offset
    cam.pop("T_cam_imu")
    cam.pop("T_cn_cnm1", None)
    cam["T_imu_cam"] = np.linalg.inv(model[0][2]).tolist()
    cam["rostopic"] = manifest["image_topics"][0]
    cam["cam_overlaps"] = []
    writeYaml(out / "kalibr_imucam_chain.yaml", dict(cam0=cam), opencv=True)
    # HIGHRES_IMU 已经过 PX4 校正；这里不再施加第二次 IMU 内参校正。
    noise.update(
        rostopic=manifest["imu_topic"],
        model="calibrated",
        Tw=np.eye(3).tolist(),
        Ta=np.eye(3).tolist(),
        R_IMUtoACC=np.eye(3).tolist(),
        R_IMUtoGYRO=np.eye(3).tolist(),
        Tg=np.zeros((3, 3)).tolist(),
    )
    writeYaml(out / "kalibr_imu_chain.yaml", dict(imu0=noise), opencv=True)
    config.update(
        use_stereo=False,
        max_cameras=1,
        downsample_cameras=False,
        boombirds_test_only=False,
        calib_cam_extrinsics=False,
        calib_cam_intrinsics=False,
        calib_cam_timeoffset=False,
        calib_imu_intrinsics=False,
        calib_imu_g_sensitivity=False,
        relative_config_imu="kalibr_imu_chain.yaml",
        relative_config_imucam="kalibr_imucam_chain.yaml",
    )
    writeYaml(out / "estimator_config.yaml", config, opencv=True)
    extr = dict(
        T_I_C0=cam["T_imu_cam"],
        source=str(Path(chainPath).resolve()),
        hardware_verified=False,
    )
    if body is not None:
        extr["T_I_B"] = body["T_I_B"]
        extr["body_source"] = body["source"]
    else:
        extr["T_I_B"] = None
    writeYaml(
        out / ("extrinsics.yaml" if bodyPath else "extrinsics.pending.yaml"), extr
    )
    result = dict(
        status="CANDIDATE_REQUIRES_REVIEW",
        hardware_verified=False,
        synthetic=False,
        source_session_sha256=digest(session / "session.json"),
        source_chain_sha256=digest(chainPath),
        source_noise_sha256=digest(noisePath),
        body_transform_complete=bool(bodyPath),
        imu_intrinsics_model="PX4-calibrated HIGHRES_IMU; no additional correction",
        review_required=[
            "Kalibr PDF 残差及棋盘格姿态翻转",
            "独立数据回放",
            "相机曝光时间模型",
            "真实 VIO 初始化、方向、尺度和漂移",
        ],
    )
    atomicJson(out / "candidate.json", result)
    return result
