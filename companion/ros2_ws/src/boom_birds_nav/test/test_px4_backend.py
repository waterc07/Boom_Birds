"""`px4_backend` 全离线测试：协议隔离 / 安全闸门 / MAVLink 回环 / 故障容忍。

测试边界（务必如实理解）
------------------------
- **不接飞控、不开串口、不起 SITL、不解锁任何真东西。**
- 所有「飞控」都是本进程内建在 `127.0.0.1` 上的 UDP 对端
  （`_LoopbackPx4Peer`），只回放本文件自己构造的 MAVLink 帧。
- 需要联网的用例只有回环 socket；不依赖外部进程。
- pymavlink 在本环境解析出的线上版本是 MAVLink 1.0（未设 `MAVLINK20`），
  对端与被测后端都用同一套方言，消息 id 与线上格式一致。

覆盖范围对应任务要求
--------------------
1. 协议/隔离：两个后端都满足 `Px4Backend`，且协议面**真实断言**没有
   电机/PWM/DShot/执行器命令（FC-004）。
2. 安全闸门：非回环默认拒绝、`allow_arming` 默认 False、`dry_run` 默认 True
   且一个字节都不发、`serial:` 没有显式开关就拒绝、非回环解锁需要第二个开关。
3. 回环 MAVLink：msg 84 的 NED 值 / type_mask / coordinate_frame /
   `time_boot_ms` 规则、`is_connected()` 跟随心跳年龄、命令收发与 ACK。
4. 重启检测：boot 时间回退恰好报一次。
5. 故障容忍：乱码、坏 CRC、截断帧之后线程仍活、畸形计数可见。
6. FakePx4Backend：可注入时钟与故障，命令已发 ≠ 已确认。
"""

from __future__ import annotations

import io
import math
import select
import socket
import time
from dataclasses import dataclass

import pytest

mavutil = pytest.importorskip("pymavlink.mavutil")

from boom_birds_nav import px4_backend as pb  # noqa: E402
from boom_birds_nav.px4_backend import (  # noqa: E402
    FakePx4Backend,
    ManualClock,
    MavlinkPx4Backend,
    Px4Backend,
)

_MAV = mavutil.mavlink

#: 回环对端发心跳时用的默认 PX4 主模式（MANUAL=1）
_MANUAL = pb.PX4_CUSTOM_MAIN_MODE_MANUAL


# ============================================================================
# 测试替身：setpoint 与「假 PX4」UDP 对端
# ============================================================================
@dataclass
class _TestSetpoint:
    """`Px4LocalSetpoint` 的测试替身——**不是**生产类型。

    生产 setpoint 由另一位协作者的 `boom_birds_nav/px4_frames.py` 提供；
    本文件只按 `Px4SetpointLike` 的结构形状造数据，用来驱动后端。
    """

    type_mask: int
    position_m: tuple[float, float, float] | None = None
    velocity_m_s: tuple[float, float, float] | None = None
    acceleration_m_s2: tuple[float, float, float] | None = None
    yaw_rad: float | None = None
    yaw_rate_rad_s: float | None = None


@dataclass
class _NoMaskSetpoint:
    """像真实 `Px4LocalSetpoint` 那样**只有值、没有 mask** 的替身。"""

    position_m: tuple[float, float, float] | None = None
    velocity_m_s: tuple[float, float, float] | None = None
    acceleration_m_s2: tuple[float, float, float] | None = None
    yaw_rad: float | None = None
    yaw_rate_rad_s: float | None = None


def _position_setpoint(x: float, y: float, z: float) -> _TestSetpoint:
    return _TestSetpoint(type_mask=pb.TYPEMASK_POSITION_ONLY, position_m=(x, y, z))


def _velocity_setpoint(vx: float, vy: float, vz: float) -> _TestSetpoint:
    return _TestSetpoint(type_mask=pb.TYPEMASK_VELOCITY_ONLY, velocity_m_s=(vx, vy, vz))


#: 只使用加速度（位置/速度/偏航全忽略）。在测试里按位组合出来，
#: 顺便证明这些位值的语义没被写错。已知值应为 3528 = 0x0DC8。
TYPEMASK_ACCEL_ONLY = (
    pb.POSITION_TARGET_TYPEMASK_X_IGNORE
    | pb.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | pb.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | pb.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | pb.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | pb.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | pb.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | pb.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


class _LoopbackPx4Peer:
    """测试进程内的假 PX4：绑 `127.0.0.1:0`，学会后端地址后收发。

    它**不是** SITL，也不假装是：只回放本测试构造的帧，并如实记录收到什么。
    """

    def __init__(self, *, sysid: int = 1, compid: int = 1, boot_ms: int = 10_000) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.setblocking(False)
        self.port = int(self.sock.getsockname()[1])
        self.sysid = int(sysid)
        self.compid = int(compid)
        self.boot_ms = int(boot_ms)
        self.backend_addr: tuple[str, int] | None = None
        self.rx: list = []            # 后端 → 对端 已解码消息
        self.rx_bytes = 0
        self.tx_bytes = 0
        self._enc = None
        self._dec = None

    # ---- 编解码 -----------------------------------------------------------
    def _encoder(self):
        # 延迟构造：后端 connect() 会调用 pymavlink 的 set_dialect，
        # 那时才知道全局方言是哪个；等第一次真正用的时候再取。
        if self._enc is None:
            self._enc = _MAV.MAVLink(
                io.BytesIO(), srcSystem=self.sysid, srcComponent=self.compid
            )
        return self._enc

    def _decoder(self):
        if self._dec is None:
            dec = _MAV.MAVLink(io.BytesIO(), srcSystem=255, srcComponent=190)
            # 对端也要能吞垃圾：不然畸形帧会以异常形式打断测试而不是被后端处理
            dec.robust_parsing = True
            self._dec = dec
        return self._dec

    # ---- 传输 -------------------------------------------------------------
    def attach(self, endpoint: str) -> None:
        """学会后端的本地地址（`link_state()["local_endpoint"]`）。"""
        host, _, port = str(endpoint).rpartition(":")
        assert host and port.isdigit(), f"不像 host:port: {endpoint!r}"
        self.backend_addr = (host, int(port))

    def send_raw(self, data: bytes) -> None:
        assert self.backend_addr is not None, "先 attach()"
        self.sock.sendto(data, self.backend_addr)
        self.tx_bytes += len(data)

    def send(self, msg, *, seq: int | None = None) -> None:
        enc = self._encoder()
        if seq is not None:
            enc.seq = int(seq) & 0xFF
        self.send_raw(msg.pack(enc))

    def drain(self, timeout_s: float = 0.3) -> list:
        """收下所有已到达的回包（累加到 `self.rx`）。"""
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            ready, _, _ = select.select([self.sock], [], [], min(remaining, 0.02))
            if not ready:
                continue
            data, addr = self.sock.recvfrom(4096)
            self.rx_bytes += len(data)
            if self.backend_addr is None:
                self.backend_addr = (addr[0], int(addr[1]))
            for msg in (self._decoder().parse_buffer(data) or []):
                if msg.get_type() != "BAD_DATA":
                    self.rx.append(msg)
        return list(self.rx)

    def wait_for(self, msgid: int, timeout_s: float = 1.0, count: int = 1):
        """等到收到第 `count` 条 `msgid` 的消息（没有则返回 None）。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.drain(0.05)
            found = [m for m in self.rx if m.get_msgId() == msgid]
            if len(found) >= count:
                return found[count - 1]
        return None

    def messages_of(self, msgid: int) -> list:
        self.drain(0.05)
        return [m for m in self.rx if m.get_msgId() == msgid]

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    # ---- 模拟 PX4 上行 -----------------------------------------------------
    def heartbeat(
        self,
        *,
        armed: bool = False,
        custom_main: int = _MANUAL,
        sub_mode: int = 0,
        autopilot: int = pb.MAV_AUTOPILOT_PX4,
        system_status: int = pb.MAV_STATE_ACTIVE,
        seq: int | None = None,
    ):
        msg = self._encoder().heartbeat_encode(
            _MAV.MAV_TYPE_QUADROTOR,
            int(autopilot),
            pb.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0,
            pb.px4_custom_mode(int(custom_main), int(sub_mode)),
            int(system_status),
        )
        self.send(msg, seq=seq)
        return msg

    def attitude(
        self, *, yaw: float = 0.0, roll: float = 0.0, pitch: float = 0.0,
        yaw_rate: float = 0.0, boot_ms: int | None = None,
    ):
        msg = self._encoder().attitude_encode(
            self.boot_ms if boot_ms is None else int(boot_ms),
            float(roll), float(pitch), float(yaw), 0.0, 0.0, float(yaw_rate),
        )
        self.send(msg)
        return msg

    def local_position_ned(
        self,
        x: float, y: float, z: float,
        vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
        boot_ms: int | None = None,
    ):
        msg = self._encoder().local_position_ned_encode(
            self.boot_ms if boot_ms is None else int(boot_ms),
            float(x), float(y), float(z), float(vx), float(vy), float(vz),
        )
        self.send(msg)
        return msg

    def sys_status(self):
        msg = self._encoder().sys_status_encode(
            0, 0, 0, 0, 12_000, -1, 100, 0, 0, 0, 0, 0, 0
        )
        self.send(msg)
        return msg

    def status_text(self, text: bytes = b"px4 test peer"):
        msg = self._encoder().statustext_encode(6, text)
        self.send(msg)
        return msg

    def command_ack(self, command: int, result: int):
        msg = self._encoder().command_ack_encode(int(command), int(result))
        self.send(msg)
        return msg

    def heartbeat_frame_with_bad_crc(self) -> bytes:
        """一帧**完整但 CRC 坏掉**的 HEARTBEAT：pymavlink 会报 BAD_DATA。"""
        enc = self._encoder()
        msg = enc.heartbeat_encode(
            _MAV.MAV_TYPE_QUADROTOR, pb.MAV_AUTOPILOT_PX4, 0,
            pb.px4_custom_mode(_MANUAL), pb.MAV_STATE_ACTIVE,
        )
        data = bytearray(msg.pack(enc))
        data[-1] ^= 0xFF
        data[-2] ^= 0xFF
        return bytes(data)

    def heartbeat_frame_truncated(self, keep: int = 10) -> bytes:
        enc = self._encoder()
        msg = enc.heartbeat_encode(
            _MAV.MAV_TYPE_QUADROTOR, pb.MAV_AUTOPILOT_PX4, 0,
            pb.px4_custom_mode(_MANUAL), pb.MAV_STATE_ACTIVE,
        )
        return bytes(msg.pack(enc))[:keep]


class _Rig:
    """一个「假 PX4 + MavlinkPx4Backend」的成套装置。

    后端绑 `udpin:127.0.0.1:0`（内核分配端口），再把它告诉对端——
    这样 dry_run 下（后端一个字节都不发）对端仍然知道往哪发心跳。
    """

    def __init__(self, **backend_kwargs) -> None:
        backend_kwargs.setdefault("read_timeout_s", 0.02)
        backend_kwargs.setdefault("heartbeat_rate_hz", 20.0)
        self.peer = _LoopbackPx4Peer()
        self.backend = MavlinkPx4Backend("udpin:127.0.0.1:0", **backend_kwargs)
        self._started = False

    def start(self, *, heartbeat: bool = True, **hb_kwargs) -> "_Rig":
        assert self.backend.connect() is True, "回环 udpin 应能建链"
        endpoint = self.backend.link_state()["local_endpoint"]
        assert endpoint, "udpin 后端必须报告本地绑定地址"
        self.peer.attach(endpoint)
        if heartbeat:
            self.peer.heartbeat(**hb_kwargs)
            # 等后端真的把对端认出来（udpin 必须先收到过数据才允许发送）：
            # 否则紧接着发的命令会以 tx_path_not_ready 被拒，测试会假失败。
            assert self.wait_connected(2.0) is True, "对端心跳后应判定连通"
        self._started = True
        return self

    def wait_connected(self, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.backend.is_connected():
                return True
            time.sleep(0.005)
        return self.backend.is_connected()

    def wait_until(self, predicate, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.005)
        return bool(predicate())

    def close(self) -> None:
        self.backend.close()
        self.peer.close()

    def __enter__(self) -> "_Rig":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


@pytest.fixture
def rig():
    """默认装置：建链、对端发一条 MANUAL 心跳。"""
    r = _Rig()
    r.start()
    try:
        yield r
    finally:
        r.close()


class _StubConn:
    """最小连接替身：不碰任何串口/网卡，只记录被写了什么。

    用于「非回环 / 串口链路上不许解锁」这类只需要走通发送路径的用例。
    """

    def __init__(self) -> None:
        self.mav = _MAV.MAVLink(io.BytesIO(), srcSystem=255, srcComponent=190)
        self.written: list[bytes] = []
        self.closed = False

    def write(self, buf: bytes) -> None:
        self.written.append(bytes(buf))

    def recv_match(self, blocking: bool = True, timeout: float | None = None):
        time.sleep(0.01)
        return None

    def close(self) -> None:
        self.closed = True

    def decoded(self) -> list:
        """把写出去的帧解回来，便于断言「到底发的是哪条命令」。"""
        parser = _MAV.MAVLink(io.BytesIO(), srcSystem=1, srcComponent=1)
        parser.robust_parsing = True
        out: list = []
        for buf in self.written:
            out.extend(m for m in (parser.parse_buffer(buf) or [])
                       if m.get_type() != "BAD_DATA")
        return out

    def command_longs(self) -> list:
        return [m for m in self.decoded()
                if m.get_type() == "COMMAND_LONG"]


def _stub_factory(stub: "_StubConn"):
    def factory(connection, **kwargs):
        return stub
    return factory


def _refusal_reasons(backend) -> list[str]:
    return [r["reason"] for r in backend.stream_diagnostics()["refusals"]]


def _never_called_factory(*args, **kwargs):  # pragma: no cover - 被调用即失败
    raise AssertionError(f"不该建链，却调用了工厂：args={args!r} kwargs={kwargs!r}")


# ============================================================================
# 1. 常量：与 pymavlink 方言逐条对照（写错就红）
# ============================================================================
def test_constants_match_pymavlink_dialect():
    """模块里的字面量必须与 pymavlink 方言一致，防止手抄出错。"""
    expected = {
        "MAVLINK_MSG_ID_HEARTBEAT": "MAVLINK_MSG_ID_HEARTBEAT",
        "MAVLINK_MSG_ID_SYS_STATUS": "MAVLINK_MSG_ID_SYS_STATUS",
        "MAVLINK_MSG_ID_ATTITUDE": "MAVLINK_MSG_ID_ATTITUDE",
        "MAVLINK_MSG_ID_LOCAL_POSITION_NED": "MAVLINK_MSG_ID_LOCAL_POSITION_NED",
        "MAVLINK_MSG_ID_COMMAND_LONG": "MAVLINK_MSG_ID_COMMAND_LONG",
        "MAVLINK_MSG_ID_COMMAND_ACK": "MAVLINK_MSG_ID_COMMAND_ACK",
        "MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED":
            "MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED",
        "MAV_CMD_COMPONENT_ARM_DISARM": "MAV_CMD_COMPONENT_ARM_DISARM",
        "MAV_CMD_DO_SET_MODE": "MAV_CMD_DO_SET_MODE",
        "MAV_FRAME_LOCAL_NED": "MAV_FRAME_LOCAL_NED",
        "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED": "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED",
        "MAV_MODE_FLAG_SAFETY_ARMED": "MAV_MODE_FLAG_SAFETY_ARMED",
        "MAV_AUTOPILOT_PX4": "MAV_AUTOPILOT_PX4",
        "MAV_AUTOPILOT_INVALID": "MAV_AUTOPILOT_INVALID",
        "MAV_TYPE_GCS": "MAV_TYPE_GCS",
        "MAV_STATE_ACTIVE": "MAV_STATE_ACTIVE",
        "MAV_RESULT_ACCEPTED": "MAV_RESULT_ACCEPTED",
        "MAV_RESULT_TEMPORARILY_REJECTED": "MAV_RESULT_TEMPORARILY_REJECTED",
        "MAV_RESULT_DENIED": "MAV_RESULT_DENIED",
        "MAV_RESULT_UNSUPPORTED": "MAV_RESULT_UNSUPPORTED",
        "MAV_RESULT_IN_PROGRESS": "MAV_RESULT_IN_PROGRESS",
        "POSITION_TARGET_TYPEMASK_X_IGNORE": "POSITION_TARGET_TYPEMASK_X_IGNORE",
        "POSITION_TARGET_TYPEMASK_Y_IGNORE": "POSITION_TARGET_TYPEMASK_Y_IGNORE",
        "POSITION_TARGET_TYPEMASK_Z_IGNORE": "POSITION_TARGET_TYPEMASK_Z_IGNORE",
        "POSITION_TARGET_TYPEMASK_VX_IGNORE": "POSITION_TARGET_TYPEMASK_VX_IGNORE",
        "POSITION_TARGET_TYPEMASK_VY_IGNORE": "POSITION_TARGET_TYPEMASK_VY_IGNORE",
        "POSITION_TARGET_TYPEMASK_VZ_IGNORE": "POSITION_TARGET_TYPEMASK_VZ_IGNORE",
        "POSITION_TARGET_TYPEMASK_AX_IGNORE": "POSITION_TARGET_TYPEMASK_AX_IGNORE",
        "POSITION_TARGET_TYPEMASK_AY_IGNORE": "POSITION_TARGET_TYPEMASK_AY_IGNORE",
        "POSITION_TARGET_TYPEMASK_AZ_IGNORE": "POSITION_TARGET_TYPEMASK_AZ_IGNORE",
        "POSITION_TARGET_TYPEMASK_FORCE_SET": "POSITION_TARGET_TYPEMASK_FORCE_SET",
        "POSITION_TARGET_TYPEMASK_YAW_IGNORE": "POSITION_TARGET_TYPEMASK_YAW_IGNORE",
        "POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE":
            "POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE",
    }
    for ours, theirs in expected.items():
        assert getattr(pb, ours) == getattr(_MAV, theirs), f"{ours} 与方言不符"

    # STATUS_TEXT 在这个方言里没有 MAVLINK_MSG_ID_* 常量，按消息表核验
    # （pymavlink 里类名/消息名是 STATUSTEXT，没有下划线）
    assert pb.MAVLINK_MSG_ID_STATUS_TEXT == 253
    assert _MAV.mavlink_map[pb.MAVLINK_MSG_ID_STATUS_TEXT].msgname == "STATUSTEXT"
    # COMMAND_LONG 是 76（我们只发送它，不接收）
    assert _MAV.mavlink_map[pb.MAVLINK_MSG_ID_COMMAND_LONG].msgname == "COMMAND_LONG"


def test_msg84_field_order_is_the_verified_one():
    """msg 84 线上字段顺序（PX4 receiver 解码依赖它，写错不会报错）。"""
    assert _MAV.mavlink_map[84].ordered_fieldnames == [
        "time_boot_ms", "x", "y", "z", "vx", "vy", "vz",
        "afx", "afy", "afz", "yaw", "yaw_rate", "type_mask",
        "target_system", "target_component", "coordinate_frame",
    ]


def test_type_mask_combination_values_are_known_good():
    """3576 / 3527 / 3135 是 PX4 setpoint 的经典掩码，按字面值钉住。"""
    assert pb.TYPEMASK_POSITION_ONLY == 3576 == 0x0DF8
    assert pb.TYPEMASK_VELOCITY_ONLY == 3527 == 0x0DC7
    assert TYPEMASK_ACCEL_ONLY == 3135 == 0x0C3F
    # 位置 + yaw（速度/加速度/yaw_rate 忽略）
    assert (pb.TYPEMASK_POSITION_ONLY & ~pb.POSITION_TARGET_TYPEMASK_YAW_IGNORE) == 2552


def test_offboard_custom_mode_is_six():
    """PX4_CUSTOM_MAIN_MODE_OFFBOARD=6；custom_mode 布局是 main<<16 | sub<<24。

    PX4 `union px4_custom_mode`（src/modules/commander/px4_custom_mode.h）:
        struct { uint16_t reserved; uint8_t main_mode; uint8_t sub_mode; };
    小端下 reserved 占低 16 位，因此 main 在 bit16-23、sub 在 bit24-31。

    这条断言曾经写成 0x0600 / 0x030400（即 >>8 / >>16 的错误布局），等于把 bug
    固化成"期望值"；SITL 实收 HEARTBEAT.custom_mode=0x03040000(AUTO/LOITER 3)
    才对不上，因此改为真实值。括号里的 0x… 是按 PX4 定义直接算出来的。
    """
    assert pb.PX4_CUSTOM_MAIN_MODE_OFFBOARD == 6
    assert pb.px4_custom_mode(pb.PX4_CUSTOM_MAIN_MODE_OFFBOARD) == 0x00060000
    assert pb.px4_custom_main_mode_name(0x00060000) == "offboard"
    assert pb.px4_custom_mode(pb.PX4_CUSTOM_MAIN_MODE_AUTO, 3) == 0x03040000
    assert pb.px4_custom_main_mode_name(0x03040000) == "auto"
    assert pb.px4_custom_sub_mode(0x03040000) == 3
    assert pb.px4_custom_main_mode_name(0x7F0000) is None  # 未知不要编名字
    # 边界：低位 reserved 不得被当成 main_mode 的一部分
    assert pb.px4_custom_main_mode_name(0x0000FF00 | 0x00060000) == "offboard"


# ============================================================================
# 2. 协议隔离（SW-001 + FC-004）
# ============================================================================
def test_both_backends_satisfy_the_protocol(rig):
    """两个实现都必须满足 `Px4Backend`（runtime_checkable 的真实 isinstance）。"""
    assert isinstance(FakePx4Backend(), Px4Backend)
    assert isinstance(rig.backend, Px4Backend)
    # 协议本身也必须能被 isinstance 检查（否则这层断言是空的）
    assert isinstance(MavlinkPx4Backend("udpin:127.0.0.1:0"), Px4Backend)


def test_protocol_has_no_actuator_surface():
    """FC-004 的**真实断言**：协议与实现都不得暴露电机/PWM/DShot/执行器命令面。"""
    # 禁令词表本身要覆盖 FC-004 点名的四类
    for word in ("pwm", "dshot", "actuator", "motor"):
        assert word in pb.FORBIDDEN_ACTUATOR_NAME_PATTERNS

    pb.assert_no_actuator_surface(Px4Backend, label="Px4Backend")
    pb.assert_no_actuator_surface(FakePx4Backend, label="FakePx4Backend")
    pb.assert_no_actuator_surface(MavlinkPx4Backend, label="MavlinkPx4Backend")
    pb.assert_no_actuator_surface(pb.VehicleState, label="VehicleState")

    public = [n for n in dir(Px4Backend) if not n.startswith("_")]
    assert public, "协议不该是空的"
    for attr in public:
        for pattern in pb.FORBIDDEN_ACTUATOR_NAME_PATTERNS:
            assert pattern not in attr.lower(), (
                f"协议暴露了 {attr!r}（命中 {pattern!r}）：FC-004 禁止直接电机命令"
            )


def test_protocol_surface_is_the_required_minimum():
    """协议面必须是任务要求的那些方法，多出来的公开名字要显式交代。"""
    required = {
        "connect", "close", "is_connected", "link_state", "read_vehicle_state",
        "send_setpoint", "arm", "disarm", "set_offboard_mode", "set_position_mode",
        "set_mode", "stream_diagnostics",
    }
    public = {n for n in dir(Px4Backend) if not n.startswith("_")}
    assert required <= public, f"协议缺少：{sorted(required - public)}"
    assert public == required, f"协议多出未交代的公开成员：{sorted(public - required)}"


def test_backends_share_a_common_diagnostic_vocabulary(rig):
    """节点应当能用同一套键读两个后端——这是「换后端不改算法」的最低要求。"""
    common_keys = {
        "connection", "connected", "heartbeat_age_s", "heartbeat_timeout_s",
        "last_error", "counters", "refusal_count", "last_refusal", "dry_run",
        "allow_arming", "restart_epoch", "px4_restart_events",
    }
    mav_diag = rig.backend.stream_diagnostics()
    fake_diag = FakePx4Backend().stream_diagnostics()
    assert common_keys <= set(mav_diag)
    assert common_keys <= set(fake_diag)
    mav_link = rig.backend.link_state()
    fake_link = FakePx4Backend().link_state()
    assert common_keys <= set(mav_link)
    assert common_keys <= set(fake_link)


def _px4_frames_api():
    """取 `px4_frames` 里与本模块对接相关的 API；模块不在/尚未成型就 skip。

    该文件由另一位协作者负责，可能仍在改动；本文件只读它、绝不改它，
    接口不齐时跳过而不是报假失败。
    """
    try:
        from boom_birds_nav import px4_frames as pf  # type: ignore
    except (ImportError, SyntaxError) as exc:
        pytest.skip(f"px4_frames 尚不可用（{type(exc).__name__}）")
    setpoint_cls = getattr(pf, "Px4LocalSetpoint", None)
    if setpoint_cls is None:
        pytest.skip("px4_frames 尚无 Px4LocalSetpoint")
    return pf, setpoint_cls


def test_px4_frames_setpoint_shape_matches_this_backend_expectation():
    """核对真实 `Px4LocalSetpoint` 的形状。

    **实测结论**（读文件得到，另一协作者可能继续改）：它只有 5 个字段，
    没有 `type_mask`；mask 由 `TypeMask.for_mode(...)` 单独构造。
    所以本后端的 `send_setpoint(setpoint, type_mask=...)` 必须支持显式 mask，
    且在没有 mask 时**拒绝**而不是猜。
    """
    pf, setpoint_cls = _px4_frames_api()
    declared: set[str] = set()
    for source in ("__dataclass_fields__", "__annotations__"):
        mapping = getattr(setpoint_cls, source, None)
        if isinstance(mapping, dict):
            declared |= set(mapping)
    for attr in ("position_m", "velocity_m_s", "acceleration_m_s2",
                 "yaw_rad", "yaw_rate_rad_s"):
        assert attr in declared, f"Px4LocalSetpoint 缺少 {attr}：本模块的取值会失败"
    if "type_mask" not in declared:
        # 这正是当前的真实形状：mask 必须显式传
        assert hasattr(pf, "TypeMask"), (
            "Px4LocalSetpoint 没有 type_mask，且 px4_frames 也没有 TypeMask："
            "后端将无从取得掩码"
        )


def test_compose_with_px4_frames_setpoint_and_typemask():
    """用真实 `Px4LocalSetpoint` + `TypeMask` 走一遍回环，断言线上内容一致。

    这是 SW-001 的接缝在实际数据类型上的端到端检查：`px4_frames` 出 NED 值
    与 mask，本后端只做搬运，不改轴向/符号。
    """
    pf, setpoint_cls = _px4_frames_api()
    try:
        mask_obj = pf.TypeMask.for_mode("position_velocity", "yaw")
    except (AttributeError, TypeError, ValueError) as exc:
        pytest.skip(f"px4_frames.TypeMask API 尚不可用（{type(exc).__name__}: {exc}）")
    setpoint = setpoint_cls(
        position_m=(1.25, -2.5, -0.75),
        velocity_m_s=(0.0, 0.0, 0.0),
        acceleration_m_s2=(0.0, 0.0, 0.0),
        yaw_rad=0.25,
    )
    mask = int(mask_obj)

    with _Rig() as rig:
        rig.start()
        rig.backend.dry_run = False
        assert rig.backend.send_setpoint(setpoint, type_mask=mask) is True
        msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)
        assert msg is not None
        assert msg.type_mask == mask
        assert msg.coordinate_frame == pb.MAV_FRAME_LOCAL_NED
        assert (msg.x, msg.y, msg.z) == pytest.approx((1.25, -2.5, -0.75))
        assert msg.yaw == pytest.approx(0.25)


def test_setpoint_without_any_mask_is_refused_not_guessed():
    """没有 mask 就不许猜轴：真实 Px4LocalSetpoint 不带 mask，必须显式传。"""
    pf, setpoint_cls = _px4_frames_api()
    if "type_mask" in set((getattr(setpoint_cls, "__dataclass_fields__", {}) or {})):
        pytest.skip("该 Px4LocalSetpoint 自带 type_mask，无需显式掩码")
    setpoint = setpoint_cls(
        position_m=(0.0, 0.0, -1.0),
        velocity_m_s=(0.0, 0.0, 0.0),
        acceleration_m_s2=(0.0, 0.0, 0.0),
        yaw_rad=0.0,
    )
    with _Rig() as rig:
        rig.start()
        rig.backend.dry_run = False
        assert rig.backend.send_setpoint(setpoint) is False
        diag = rig.backend.stream_diagnostics()
        assert diag["last_refusal"]["reason"] == pb.REFUSAL_SETPOINT_MASK_MISSING
        assert "TypeMask" in diag["last_refusal"]["detail"]
        assert diag["counters"]["setpoints_sent"] == 0
        assert rig.peer.messages_of(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) == []


# ============================================================================
# 3. 安全闸门
# ============================================================================
@pytest.mark.parametrize(
    "connection",
    [
        "udpin:0.0.0.0:14540",        # 监听所有网卡：不是回环
        "udpout:192.168.1.20:14550",  # 真机网段
        "10.1.2.3:14550",             # 裸 host:port（pymavlink 视为 udpout）
        "tcp:drone.example.com:5760",  # 域名：刻意不做 DNS 解析
        "tcp:127.0.0.1.evil.com:5760",  # 前缀像回环的域名——必须拒绝
    ],
)
def test_non_loopback_hosts_refused_by_default(connection):
    backend = MavlinkPx4Backend(connection, mavlink_connection_factory=_never_called_factory)
    assert backend.guard_reason == pb.REFUSAL_NON_LOOPBACK_HOST
    assert backend.connect() is False
    diag = backend.stream_diagnostics()
    assert diag["connected"] is False
    assert diag["rx_thread_alive"] is False
    assert diag["last_refusal"]["reason"] == pb.REFUSAL_NON_LOOPBACK_HOST
    assert "allow_non_loopback" in diag["last_refusal"]["detail"]
    assert diag["counters"][f"refused_{pb.REFUSAL_NON_LOOPBACK_HOST}"] == 1
    assert diag["counters"]["packets_tx"] == 0
    assert diag["counters"]["bytes_tx"] == 0
    backend.close()


@pytest.mark.parametrize(
    "connection",
    [
        "udpin:127.0.0.1:14540",
        "udpout:127.0.0.1:14550",
        "udpin:localhost:14540",
        "udpin:LOCALHOST:14540",       # 主机名大小写不敏感
        "127.0.0.1:14550",
        "udpin:[::1]:14540",           # 仅校验允许清单（mavudp 不支持 IPv6 socket）
        "::1:14540",
        "udpin:127.5.5.5:14540",       # 整个 127/8 都是回环
    ],
)
def test_loopback_hosts_pass_the_guard(connection):
    """只评估闸门，不建链：确认回环允许清单本身是对的。"""
    backend = MavlinkPx4Backend(connection, mavlink_connection_factory=_never_called_factory)
    assert backend.guard_reason is None, f"{connection} 应被认作回环"


def test_non_loopback_allowed_with_explicit_optin():
    """显式 opt-in 后可以建非回环链路——但仍不能解锁，且默认仍不发字节。

    这里用 `_StubConn` 替身而不是真的绑 `0.0.0.0`：闸门逻辑与 socket 无关，
    测试也就完全不碰网卡。
    """
    stub = _StubConn()
    backend = MavlinkPx4Backend(
        "udpin:0.0.0.0:14540", allow_non_loopback=True,
        mavlink_connection_factory=_stub_factory(stub),
    )
    assert backend.guard_reason is None
    try:
        assert backend.connect() is True
        diag = backend.stream_diagnostics()
        assert diag["allow_non_loopback"] is True
        assert diag["dry_run"] is True
        # 默认拒绝解锁：非回环 + allow_arming 默认 False
        assert backend.arm() is False
        reasons = _refusal_reasons(backend)
        assert pb.REFUSAL_ARMING_NOT_ALLOWED in reasons
        diag = backend.stream_diagnostics()
        assert diag["counters"]["arm_refused"] == 1
        assert stub.command_longs() == [], "被拒的解锁不许写出 COMMAND_LONG"
        assert stub.written == [], "dry_run 下连字节都不该写"
    finally:
        backend.close()


def test_arm_refused_unless_allow_arming_true(rig):
    """`allow_arming` 默认 False：`arm()` 必须拒绝，并留下可见原因。"""
    assert rig.backend.allow_arming is False
    assert rig.backend.arm() is False
    diag = rig.backend.stream_diagnostics()
    assert diag["last_refusal"]["reason"] == pb.REFUSAL_ARMING_NOT_ALLOWED
    assert diag["counters"]["arm_refused"] == 1
    assert diag["counters"]["commands_sent"] == 0
    # 拒绝发生在构造帧之前：对端一个 COMMAND_LONG 都不该看到
    assert rig.peer.messages_of(pb.MAVLINK_MSG_ID_COMMAND_LONG) == []
    assert rig.backend.read_vehicle_state().armed is False
    assert rig.backend.read_vehicle_state().commanded_armed is None


def test_arm_allowed_on_loopback_when_opted_in(rig):
    """回环 + allow_arming=True + dry_run=False：命令真的发出去，但状态仍未被确认。"""
    rig.backend.allow_arming = True
    rig.backend.dry_run = False
    assert rig.backend.arm() is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_COMMAND_LONG)
    assert msg is not None, "对端应收到 COMMAND_LONG"
    assert msg.command == pb.MAV_CMD_COMPONENT_ARM_DISARM == 400
    assert msg.param1 == float(pb.ARMING_ACTION_ARM) == 1.0
    assert msg.target_system == 1 and msg.target_component == 1
    assert msg.confirmation == 0
    # 只发了命令：还没收到任何 ACK 或 armed 心跳，绝不能报「已解锁」
    state = rig.backend.read_vehicle_state()
    assert state.armed is False
    assert state.commanded_armed is True
    entry = rig.backend.stream_diagnostics()["commands"]["400"]
    assert entry["ack_result"] is None, "写了命令不等于收到 ACK"
    assert rig.backend.stream_diagnostics()["counters"]["commands_sent"] == 1


def test_non_loopback_arming_needs_the_second_flag():
    """非回环链路上解锁需要**第二个**显式开关。"""
    stub = _StubConn()
    backend = MavlinkPx4Backend(
        "udpin:0.0.0.0:14540",
        allow_non_loopback=True,
        allow_arming=True,                  # 第一个开关
        mavlink_connection_factory=_stub_factory(stub),
    )
    assert backend.guard_reason is None
    try:
        assert backend.connect() is True
        assert backend.arm() is False, "非回环链路缺第二个开关时必须拒绝解锁"
        diag = backend.stream_diagnostics()
        assert diag["last_refusal"]["reason"] == pb.REFUSAL_ARMING_NOT_ALLOWED
        assert "allow_arming_on_non_loopback" in diag["last_refusal"]["detail"]
        assert diag["counters"]["arm_refused"] == 1
        assert diag["counters"]["commands_sent"] == 0
        assert stub.command_longs() == [], "被拒的解锁不许写出 COMMAND_LONG"
        # 解除第二个开关后仍受 dry_run 约束：dry_run 下命令返回 False 而不是假成功
        backend.allow_arming_on_non_loopback = True
        assert backend.arm() is False
        assert backend.stream_diagnostics()["counters"]["commands_suppressed_dry_run"] == 1
        assert stub.command_longs() == [], "dry_run 下仍不许写字节"
    finally:
        backend.close()


def test_serial_refused_without_explicit_uart_optin():
    """`serial:` 默认拒绝，且**绝不**调用建链工厂（不打开任何串口）。"""
    backend = MavlinkPx4Backend(
        "serial:/dev/ttyAMA0", mavlink_connection_factory=_never_called_factory
    )
    assert backend.guard_reason == pb.REFUSAL_SERIAL_NOT_ALLOWED
    assert backend.connect() is False
    diag = backend.stream_diagnostics()
    assert diag["last_refusal"]["reason"] == pb.REFUSAL_SERIAL_NOT_ALLOWED
    assert diag["rx_thread_alive"] is False
    assert diag["counters"]["packets_tx"] == 0
    assert backend.is_connected() is False
    backend.close()


def test_serial_optin_connects_but_arming_still_blocked():
    """`allow_serial=True` 只放开建链；解锁仍被「非回环」规则挡住。"""
    stub = _StubConn()
    backend = MavlinkPx4Backend(
        "serial:/dev/ttyAMA0",
        allow_serial=True,
        allow_arming=True,          # 想解锁
        dry_run=False,
        baud=115200,
        read_timeout_s=0.01,
        mavlink_connection_factory=_stub_factory(stub),
    )
    assert backend.guard_reason is None
    try:
        assert backend.connect() is True
        assert backend.arm() is False, "串口链路是非回环：必须有第二个开关"
        diag = backend.stream_diagnostics()
        assert diag["last_refusal"]["reason"] == pb.REFUSAL_ARMING_NOT_ALLOWED
        assert "allow_arming_on_non_loopback" in diag["last_refusal"]["detail"]
        assert stub.command_longs() == [], "被拒绝的解锁不该写出 COMMAND_LONG"
        # 但非解锁命令是允许的（模式切换不是解锁）
        assert backend.set_offboard_mode() is True
        commands = stub.command_longs()
        assert len(commands) == 1
        assert commands[0].command == pb.MAV_CMD_DO_SET_MODE
        assert commands[0].param2 == float(pb.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        assert backend.stream_diagnostics()["counters"]["commands_sent"] == 1
    finally:
        backend.close()
    assert stub.closed is True


@pytest.mark.parametrize(
    "connection",
    ["file:flight.tlog", "udpin", "udpin:127.0.0.1", "tcpin:not-a-host"],
)
def test_unparsable_or_unsupported_schemes_refused(connection):
    backend = MavlinkPx4Backend(connection, mavlink_connection_factory=_never_called_factory)
    assert backend.guard_reason in (
        pb.REFUSAL_UNSUPPORTED_SCHEME, pb.REFUSAL_BAD_CONNECTION_STRING
    )
    assert backend.connect() is False
    backend.close()


# ---------------------------------------------------------------- dry_run
def test_dry_run_transmits_nothing_at_all(rig):
    """`dry_run=True`（默认）：setpoint 完整构造计数，但线上**零字节**。"""
    assert rig.backend.dry_run is True
    rig.backend.allow_arming = True     # 即使放开解锁，dry_run 也不许写字节
    for index in range(3):
        assert rig.backend.send_setpoint(_position_setpoint(1.0 + index, 2.0, -3.0)) is True
    # 命令在 dry_run 下必须如实返回 False（没发出去就不许说成功）
    assert rig.backend.arm() is False
    assert rig.backend.set_offboard_mode() is False

    rig.peer.drain(0.4)
    counters = rig.backend.stream_diagnostics()["counters"]
    assert counters["setpoints_built"] == 3, "dry_run 仍要完整构造 setpoint"
    assert counters["setpoints_suppressed_dry_run"] == 3
    assert counters["setpoints_sent"] == 0
    assert counters["setpoints_rejected"] == 0
    assert counters["packets_tx"] == 0
    assert counters["bytes_tx"] == 0, "dry_run 不许往线上写任何字节"
    assert counters["heartbeats_tx"] == 0, "连心跳也不发：dry_run 的定义就是零字节"
    assert counters["commands_suppressed_dry_run"] == 2
    assert rig.peer.rx_bytes == 0, "对端一个字节都不该收到"
    reasons = _refusal_reasons(rig.backend)
    assert reasons.count(pb.REFUSAL_DRY_RUN_COMMAND) == 2
    # 但对端的上行仍被正常处理（dry_run 只闭嘴，不装聋）
    assert rig.wait_connected() is True
    assert rig.backend.stream_diagnostics()["counters"]["heartbeats_rx"] >= 1


def test_dry_run_still_rejects_invalid_setpoints(rig):
    """dry_run 也要执行完整校验：坏 setpoint 不许悄悄「成功」。"""
    bad = _TestSetpoint(type_mask=pb.TYPEMASK_POSITION_ONLY, position_m=None)
    assert rig.backend.send_setpoint(bad) is False
    counters = rig.backend.stream_diagnostics()["counters"]
    assert counters["setpoints_built"] == 0
    assert counters["setpoints_rejected"] == 1
    assert counters["setpoints_suppressed_dry_run"] == 0
    assert pb.REFUSAL_SETPOINT_MISSING_VALUE in _refusal_reasons(rig.backend)


# ============================================================================
# 4. 回环 MAVLink：心跳、状态、msg 84、命令
# ============================================================================
def test_is_connected_follows_heartbeat_age():
    """`is_connected()` 由心跳年龄决定，而不是「socket 还在」。"""
    with _Rig(heartbeat_timeout_s=0.3) as rig:
        rig.start()
        assert rig.wait_connected() is True
        state = rig.backend.read_vehicle_state()
        assert state.connected is True
        assert state.heartbeat_age_s is not None and state.heartbeat_age_s < 0.3
        assert state.heartbeat_timeout_s == 0.3
        # 让对端彻底闭嘴，但不关 socket
        assert rig.wait_until(lambda: not rig.backend.is_connected(), timeout_s=2.0) is True
        link = rig.backend.link_state()
        assert link["connected"] is False
        assert link["heartbeat_age_s"] > 0.3
        assert link["connection"] == "udpin:127.0.0.1:0"
        # 心跳回来即恢复
        rig.peer.heartbeat()
        assert rig.wait_connected() is True


def test_heartbeat_with_other_autopilot_does_not_connect():
    """防误连：非 PX4 的心跳不能把链路判成「通着」。"""
    with _Rig() as rig:
        rig.start(heartbeat=False)
        rig.peer.heartbeat(autopilot=pb.MAV_AUTOPILOT_INVALID)
        assert rig.wait_until(
            lambda: rig.backend.stream_diagnostics()["counters"]["heartbeats_other_autopilot"] >= 1
        ) is True
        assert rig.backend.is_connected() is False
        state = rig.backend.read_vehicle_state()
        assert state.armed is False and state.mode_name is None
        diag = rig.backend.stream_diagnostics()
        assert "unexpected_autopilot" in (diag["last_error"] or "")


def test_attitude_and_local_position_are_exposed(rig):
    rig.peer.attitude(yaw=0.42, roll=0.11, pitch=-0.07, yaw_rate=0.03)
    rig.peer.local_position_ned(1.5, -2.5, -0.75, 0.1, 0.2, -0.3)
    assert rig.wait_until(
        lambda: rig.backend.read_vehicle_state().position_ned_m is not None
    ) is True
    state = rig.backend.read_vehicle_state()
    assert state.yaw_rad == pytest.approx(0.42)
    assert state.roll_rad == pytest.approx(0.11)
    assert state.pitch_rad == pytest.approx(-0.07)
    assert state.yaw_rate_rad_s == pytest.approx(0.03)
    assert state.attitude_received_mono_s is not None
    assert state.attitude_received_mono_s <= time.monotonic()
    assert state.position_ned_m == pytest.approx((1.5, -2.5, -0.75))
    assert state.velocity_ned_m_s == pytest.approx((0.1, 0.2, -0.3))
    assert 0.0 <= state.attitude_age_s < 1.0
    assert 0.0 <= state.position_age_s < 1.0
    assert state.source == "mavlink"


def test_sys_status_and_status_text_are_counted(rig):
    rig.peer.sys_status()
    rig.peer.status_text(b"armed by test peer")
    assert rig.wait_until(
        lambda: rig.backend.stream_diagnostics()["counters"]["status_text_rx"] >= 1
    ) is True
    diag = rig.backend.stream_diagnostics()
    assert diag["counters"]["sys_status_rx"] >= 1
    assert "armed by test peer" in (diag["last_error"] or "")


def test_heartbeat_identity_is_gcs(rig):
    """后端发出去的心跳必须是 GCS 身份（SITL/真机联调要用）。"""
    rig.backend.dry_run = False
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_HEARTBEAT)
    assert msg is not None, "dry_run=False 时应发出 GCS 心跳"
    assert msg.type == pb.MAV_TYPE_GCS == 6
    assert msg.autopilot == pb.MAV_AUTOPILOT_INVALID == 8
    assert msg.get_srcSystem() == 255
    assert msg.get_srcComponent() == 190
    assert rig.backend.stream_diagnostics()["counters"]["heartbeats_tx"] >= 1


def test_setpoint_position_only_on_the_wire(rig):
    """msg 84 的 NED 值、type_mask、coordinate_frame 逐字段断言。"""
    rig.backend.dry_run = False
    assert rig.backend.send_setpoint(_position_setpoint(1.5, -2.25, -3.75)) is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)
    assert msg is not None
    assert msg.get_msgId() == 84
    assert msg.get_type() == "SET_POSITION_TARGET_LOCAL_NED"
    assert msg.target_system == 1 and msg.target_component == 1
    assert msg.coordinate_frame == pb.MAV_FRAME_LOCAL_NED == 1
    assert msg.type_mask == pb.TYPEMASK_POSITION_ONLY == 3576
    assert (msg.x, msg.y, msg.z) == pytest.approx((1.5, -2.25, -3.75))
    # 被忽略的轴必须写 0（PX4 自己会覆盖成 NaN，但线上要有确定值）
    assert (msg.vx, msg.vy, msg.vz) == (0.0, 0.0, 0.0)
    assert (msg.afx, msg.afy, msg.afz) == (0.0, 0.0, 0.0)
    assert (msg.yaw, msg.yaw_rate) == (0.0, 0.0)
    counters = rig.backend.stream_diagnostics()["counters"]
    assert counters["setpoints_sent"] == 1
    assert counters["setpoints_built"] == 1
    assert counters["packets_tx"] >= 1 and counters["bytes_tx"] > 0
    assert counters["tx_errors"] == 0


def test_setpoint_velocity_only_on_the_wire(rig):
    rig.backend.dry_run = False
    assert rig.backend.send_setpoint(_velocity_setpoint(0.5, -1.25, 0.75)) is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)
    assert msg is not None
    assert msg.type_mask == pb.TYPEMASK_VELOCITY_ONLY == 3527
    assert msg.coordinate_frame == 1
    assert (msg.vx, msg.vy, msg.vz) == pytest.approx((0.5, -1.25, 0.75))
    assert (msg.x, msg.y, msg.z) == (0.0, 0.0, 0.0)


def test_setpoint_position_plus_yaw_on_the_wire(rig):
    """位置 + yaw（yaw_rate 忽略）：mask 2552，yaw 原样上线。"""
    rig.backend.dry_run = False
    mask = pb.TYPEMASK_POSITION_ONLY & ~pb.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    setpoint = _TestSetpoint(type_mask=mask, position_m=(4.0, 5.0, -6.0), yaw_rad=0.7)
    assert rig.backend.send_setpoint(setpoint) is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)
    assert msg is not None
    assert msg.type_mask == mask == 2552
    assert (msg.x, msg.y, msg.z) == pytest.approx((4.0, 5.0, -6.0))
    assert msg.yaw == pytest.approx(0.7)
    assert msg.yaw_rate == 0.0


def test_ignored_axes_are_zeroed_and_counted(rig):
    """给了值但 mask 说忽略：以 mask 为准，线上填 0 并计数（不静默丢信息）。"""
    rig.backend.dry_run = False
    setpoint = _TestSetpoint(
        type_mask=pb.TYPEMASK_VELOCITY_ONLY,
        position_m=(9.0, 9.0, 9.0),      # mask 忽略位置
        velocity_m_s=(1.0, 2.0, 3.0),
    )
    assert rig.backend.send_setpoint(setpoint) is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)
    assert msg is not None
    assert (msg.x, msg.y, msg.z) == (0.0, 0.0, 0.0)
    assert rig.backend.stream_diagnostics()["counters"]["setpoint_ignored_axes_zeroed"] == 3


def test_time_boot_ms_is_zero_without_boot_evidence(rig):
    """文档规则第 2 条：没收到过带 time_boot_ms 的上行 → 填 0。"""
    rig.backend.dry_run = False
    diag = rig.backend.stream_diagnostics()
    assert diag["px4_boot_time_ms"] is None
    assert rig.backend.send_setpoint(_position_setpoint(0.0, 0.0, -1.0)) is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)
    assert msg is not None
    assert msg.time_boot_ms == 0
    assert rig.backend.stream_diagnostics()["last_setpoint"]["time_boot_ms"] == 0


def test_time_boot_ms_extrapolates_the_observed_boot_clock(rig):
    """文档规则第 1 条：观测值 + 自观测以来的单调毫秒数。"""
    rig.backend.dry_run = False
    observed = 250_000
    rig.peer.attitude(boot_ms=observed)
    assert rig.wait_until(
        lambda: rig.backend.read_vehicle_state().boot_time_ms == observed
    ) is True
    time.sleep(0.06)                       # 让外推量明显可测
    assert rig.backend.send_setpoint(_position_setpoint(0.0, 0.0, -1.0)) is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)
    assert msg is not None
    assert observed + 10 <= msg.time_boot_ms <= observed + 1_000, (
        f"应外推到发帧时刻，实际 time_boot_ms={msg.time_boot_ms}"
    )
    assert rig.backend.read_vehicle_state().boot_time_ms == observed


@pytest.mark.parametrize(
    "setpoint,expected_reason",
    [
        (None, pb.REFUSAL_SETPOINT_NOT_LIKE),
        (_TestSetpoint(type_mask=None), pb.REFUSAL_SETPOINT_MASK_INVALID),
        # 完全没有 mask（真实 Px4LocalSetpoint 就是这个形状）：拒绝而不是猜轴
        (_NoMaskSetpoint(position_m=(0.0, 0.0, -1.0)),
         pb.REFUSAL_SETPOINT_MASK_MISSING),
        (_TestSetpoint(type_mask=-1, position_m=(0.0, 0.0, 0.0)),
         pb.REFUSAL_SETPOINT_MASK_INVALID),
        (_TestSetpoint(type_mask=pb.TYPEMASK_POSITION_ONLY, position_m=None),
         pb.REFUSAL_SETPOINT_MISSING_VALUE),
        (_TestSetpoint(type_mask=pb.TYPEMASK_POSITION_ONLY, position_m=(0.0, 0.0)),
         pb.REFUSAL_SETPOINT_MISSING_VALUE),
        (_TestSetpoint(type_mask=pb.TYPEMASK_POSITION_ONLY,
                       position_m=(0.0, float("nan"), 0.0)),
         pb.REFUSAL_SETPOINT_NONFINITE),
        (_TestSetpoint(type_mask=pb.TYPEMASK_POSITION_ONLY,
                       position_m=(0.0, 0.0, float("inf"))),
         pb.REFUSAL_SETPOINT_NONFINITE),
        # 位置/速度/加速度全被忽略：PX4 会直接丢弃，我们在本地就拒
        (_TestSetpoint(type_mask=0xFFFF), pb.REFUSAL_SETPOINT_ALL_AXES_IGNORED),
        # yaw 与 yaw_rate 同时有效：MAVLink 规范不允许
        (_TestSetpoint(type_mask=pb.TYPEMASK_POSITION_ONLY & ~1024 & ~2048,
                       position_m=(1.0, 2.0, 3.0), yaw_rad=0.1, yaw_rate_rad_s=0.2),
         pb.REFUSAL_SETPOINT_YAW_AND_YAW_RATE),
        # FORCE_SET + 加速度有效：PX4 明确不支持
        (_TestSetpoint(type_mask=TYPEMASK_ACCEL_ONLY
                       | pb.POSITION_TARGET_TYPEMASK_FORCE_SET,
                       acceleration_m_s2=(0.1, 0.2, 0.3)),
         pb.REFUSAL_SETPOINT_FORCE_NOT_SUPPORTED),
    ],
)
def test_invalid_setpoints_are_refused_with_visible_reasons(rig, setpoint, expected_reason):
    """每一条拒绝都必须有 reason，且**不发上线**。"""
    rig.backend.dry_run = False
    assert rig.backend.send_setpoint(setpoint) is False
    diag = rig.backend.stream_diagnostics()
    assert diag["last_refusal"]["reason"] == expected_reason
    assert expected_reason in _refusal_reasons(rig.backend)
    assert diag["counters"]["setpoints_rejected"] == 1
    assert diag["counters"]["setpoints_sent"] == 0
    assert diag["counters"]["packets_tx"] <= diag["counters"]["heartbeats_tx"]
    assert rig.peer.messages_of(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) == []


def test_setpoint_before_connect_is_not_sent():
    """没连接时不假装发出去了。"""
    other = MavlinkPx4Backend("udpin:127.0.0.1:0", dry_run=False)
    try:
        assert other.send_setpoint(_position_setpoint(0.0, 0.0, -1.0)) is False
        diag = other.stream_diagnostics()
        assert diag["counters"]["setpoints_built"] == 1    # 构造过了
        assert diag["counters"]["setpoints_not_sent"] == 1
        assert diag["counters"]["setpoints_sent"] == 0
        assert diag["counters"]["packets_tx"] == 0
        assert pb.REFUSAL_TX_PATH_NOT_READY in _refusal_reasons(other)
    finally:
        other.close()


# ---------------------------------------------------------------- 命令
def test_offboard_mode_command_and_ack_tracking(rig):
    """DO_SET_MODE：param1=CUSTOM_MODE_ENABLED, param2=OFFBOARD(6), param3=0。"""
    rig.backend.dry_run = False
    assert rig.backend.set_offboard_mode() is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_COMMAND_LONG)
    assert msg is not None
    assert msg.command == pb.MAV_CMD_DO_SET_MODE == 176
    assert msg.param1 == float(pb.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) == 1.0
    assert msg.param2 == float(pb.PX4_CUSTOM_MAIN_MODE_OFFBOARD) == 6.0
    assert msg.param3 == 0.0
    assert msg.target_system == 1 and msg.target_component == 1
    # 写出去了 ≠ 被接受
    entry = rig.backend.stream_diagnostics()["commands"]["176"]
    assert entry["ack_result"] is None
    assert entry["ack_result_name"] is None
    assert rig.backend.read_vehicle_state().mode_name == "manual"
    assert rig.backend.read_vehicle_state().commanded_mode_name == "offboard"
    # 状态里的模式只认 HEARTBEAT
    rig.peer.heartbeat(custom_main=pb.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
    assert rig.wait_until(
        lambda: rig.backend.read_vehicle_state().mode_name == "offboard"
    ) is True
    assert rig.backend.read_vehicle_state().is_offboard is True
    # 再补上 ACK
    rig.peer.command_ack(pb.MAV_CMD_DO_SET_MODE, pb.MAV_RESULT_ACCEPTED)
    assert rig.wait_until(
        lambda: rig.backend.stream_diagnostics()["commands"]["176"]["ack_result"] == 0
    ) is True
    diag = rig.backend.stream_diagnostics()
    assert diag["commands"]["176"]["ack_result_name"] == "ACCEPTED"
    assert diag["counters"]["commands_accepted"] >= 1


def test_rejected_command_ack_is_recorded(rig):
    rig.backend.dry_run = False
    assert rig.backend.set_offboard_mode() is True
    assert rig.peer.wait_for(pb.MAVLINK_MSG_ID_COMMAND_LONG) is not None
    rig.peer.command_ack(pb.MAV_CMD_DO_SET_MODE, pb.MAV_RESULT_DENIED)
    assert rig.wait_until(
        lambda: rig.backend.stream_diagnostics()["commands"]["176"]["ack_result"] == 2
    ) is True
    diag = rig.backend.stream_diagnostics()
    assert diag["commands"]["176"]["ack_result_name"] == "DENIED"
    assert diag["counters"]["commands_rejected"] >= 1
    assert "command_rejected" in (diag["last_error"] or "")
    # 被拒之后模式不能变成 offboard
    assert rig.backend.read_vehicle_state().mode_name == "manual"


def test_in_progress_ack_is_not_treated_as_final(rig):
    rig.backend.dry_run = False
    assert rig.backend.set_offboard_mode() is True
    assert rig.peer.wait_for(pb.MAVLINK_MSG_ID_COMMAND_LONG) is not None
    rig.peer.command_ack(pb.MAV_CMD_DO_SET_MODE, pb.MAV_RESULT_IN_PROGRESS)
    assert rig.wait_until(
        lambda: rig.backend.stream_diagnostics()["counters"]["commands_in_progress"] >= 1
    ) is True
    entry = rig.backend.stream_diagnostics()["commands"]["176"]
    assert entry["ack_result"] is None, "IN_PROGRESS 不是终态，不许写成结果"
    assert "in_progress_mono_s" in entry


def test_set_position_mode_uses_posctl(rig):
    rig.backend.dry_run = False
    assert rig.backend.set_position_mode() is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_COMMAND_LONG)
    assert msg is not None
    assert msg.command == pb.MAV_CMD_DO_SET_MODE
    assert msg.param2 == float(pb.PX4_CUSTOM_MAIN_MODE_POSCTL) == 3.0


@pytest.mark.parametrize(
    "name,param2,param3",
    [
        ("offboard", 6, 0),
        ("posctl", 3, 0),
        ("manual", 1, 0),
        ("altctl", 2, 0),
        ("auto", 4, 0),            # AUTO 且 sub=0 → PX4 落到 AUTO_MISSION（已核验）
        ("auto:loiter", 4, 3),
        ("auto:rtl", 4, 5),
        ("AUTO:MISSION", 4, 4),    # 大小写不敏感
    ],
)
def test_set_mode_name_mapping(rig, name, param2, param3):
    rig.backend.dry_run = False
    before = len(rig.peer.messages_of(pb.MAVLINK_MSG_ID_COMMAND_LONG))
    assert rig.backend.set_mode(name) is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_COMMAND_LONG, count=before + 1)
    assert msg is not None
    assert msg.command == pb.MAV_CMD_DO_SET_MODE
    assert msg.param1 == 1.0
    assert msg.param2 == float(param2)
    assert msg.param3 == float(param3)


def test_unknown_mode_is_refused_without_bytes(rig):
    rig.backend.dry_run = False
    assert rig.backend.set_mode("hyperdrive") is False
    diag = rig.backend.stream_diagnostics()
    assert diag["last_refusal"]["reason"] == pb.REFUSAL_UNKNOWN_MODE
    assert diag["counters"]["commands_sent"] == 0
    assert rig.peer.messages_of(pb.MAVLINK_MSG_ID_COMMAND_LONG) == []


def test_armed_state_only_after_acknowledged_heartbeat(rig):
    """命令发出 → 收到 ACK → 仍然不算已解锁；只有 HEARTBEAT 的 ARMED 位算。"""
    rig.backend.allow_arming = True
    rig.backend.dry_run = False
    assert rig.backend.arm() is True
    assert rig.peer.wait_for(pb.MAVLINK_MSG_ID_COMMAND_LONG) is not None

    # 1) 只有 ACK：状态仍是未解锁
    rig.peer.command_ack(pb.MAV_CMD_COMPONENT_ARM_DISARM, pb.MAV_RESULT_ACCEPTED)
    assert rig.wait_until(
        lambda: rig.backend.stream_diagnostics()["counters"]["commands_accepted"] >= 1
    ) is True
    assert rig.backend.read_vehicle_state().armed is False, (
        "ACK 只说明命令被接受，不等于飞控已经解锁"
    )
    assert rig.backend.stream_diagnostics()["acknowledged_armed"] is False

    # 2) 收到带 ARMED 位的心跳：这才算已解锁
    rig.peer.heartbeat(armed=True)
    assert rig.wait_until(
        lambda: rig.backend.read_vehicle_state().armed is True
    ) is True
    assert rig.backend.stream_diagnostics()["acknowledged_armed"] is True

    # 3) 心跳回到未解锁：状态跟着回去（不做单向闩锁）
    rig.peer.heartbeat(armed=False)
    assert rig.wait_until(
        lambda: rig.backend.read_vehicle_state().armed is False
    ) is True


def test_disarm_sends_param1_zero(rig):
    rig.backend.allow_arming = True
    rig.backend.dry_run = False
    assert rig.backend.disarm() is True
    msg = rig.peer.wait_for(pb.MAVLINK_MSG_ID_COMMAND_LONG)
    assert msg is not None
    assert msg.command == pb.MAV_CMD_COMPONENT_ARM_DISARM
    assert msg.param1 == float(pb.ARMING_ACTION_DISARM) == 0.0
    assert rig.backend.read_vehicle_state().commanded_armed is False


# ============================================================================
# 5. 重启检测
# ============================================================================
def test_px4_restart_detected_exactly_once():
    """对端停心跳后带**重置的 boot 时间**回来 → 恰好报一次重启。"""
    with _Rig(heartbeat_timeout_s=0.25) as rig:
        rig.start()
        rig.peer.attitude(boot_ms=200_000)
        rig.peer.heartbeat(armed=True, custom_main=pb.PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        assert rig.wait_until(
            lambda: rig.backend.read_vehicle_state().boot_time_ms == 200_000
        ) is True
        assert rig.wait_until(
            lambda: rig.backend.read_vehicle_state().armed is True
        ) is True
        assert rig.backend.link_state()["restart_epoch"] == 0

        # 对端消失（停心跳 + 停所有上行）
        assert rig.wait_until(lambda: not rig.backend.is_connected(), timeout_s=2.0) is True

        # 对端以「刚启动」的姿态回来：boot 时间回到 1500 ms
        rig.peer.boot_ms = 1_500
        rig.peer.heartbeat()
        rig.peer.attitude(boot_ms=1_500)
        assert rig.wait_until(
            lambda: rig.backend.link_state()["restart_epoch"] == 1
        ) is True

        # 新时间线上继续正常上行：不许再报第二次
        for step in range(1, 6):
            rig.peer.boot_ms = 1_500 + step * 200
            rig.peer.heartbeat()
            rig.peer.attitude(boot_ms=rig.peer.boot_ms)
        rig.peer.drain(0.1)
        link = rig.backend.link_state()
        assert link["restart_epoch"] == 1, "重启事件必须恰好一次"
        assert link["px4_restart_events"] == 1
        assert link["counters"]["px4_restarts"] == 1
        events = rig.backend.stream_diagnostics()["restart_events"]
        assert len(events) == 1
        assert events[0]["reason"] == "px4_boot_time_regressed"
        assert events[0]["old_boot_time_ms"] == 200_000
        assert events[0]["new_boot_time_ms"] == 1_500
        assert rig.backend.read_vehicle_state().restart_epoch == 1

        # 重启后「已确认」状态必须作废：等新心跳把它重新建立起来
        rig.peer.heartbeat(armed=False, custom_main=_MANUAL)
        assert rig.wait_until(
            lambda: rig.backend.read_vehicle_state().armed is False
            and rig.backend.read_vehicle_state().mode_name == "manual"
        ) is True


def test_small_boot_time_jitter_is_not_a_restart():
    """容差内的乱序/抖动不算重启；超过容差才算。"""
    with _Rig(restart_boot_tolerance_ms=1_000) as rig:
        rig.start()
        rig.peer.attitude(boot_ms=100_000)
        assert rig.wait_until(
            lambda: rig.backend.read_vehicle_state().boot_time_ms == 100_000
        ) is True

        rig.peer.attitude(boot_ms=99_500)      # 容差内（落后 500 ms）
        rig.peer.drain(0.05)
        assert rig.backend.link_state()["restart_epoch"] == 0

        rig.peer.attitude(boot_ms=98_000)      # 落后 2000 ms > 1000 ms 容差
        assert rig.wait_until(
            lambda: rig.backend.link_state()["restart_epoch"] == 1
        ) is True
        assert rig.backend.read_vehicle_state().boot_time_ms == 98_000


def test_heartbeat_seq_reset_is_evidence_but_not_a_restart():
    """心跳序号回退只作旁证计数；255→0 的正常回绕不算回退。"""
    with _Rig() as rig:
        rig.start(heartbeat=False)
        rig.peer.heartbeat(seq=10)
        rig.peer.heartbeat(seq=11)
        assert rig.wait_connected() is True
        assert rig.backend.stream_diagnostics()["counters"]["heartbeat_seq_resets"] == 0

        # 正常回绕 254 → 255 → 0：步进连续，不许计数
        rig.peer.heartbeat(seq=254)
        rig.peer.heartbeat(seq=255)
        rig.peer.heartbeat(seq=0)
        rig.peer.drain(0.05)
        assert rig.backend.stream_diagnostics()["counters"]["heartbeat_seq_resets"] == 0

        # 真正的回退：从 200 跳回 0
        rig.peer.heartbeat(seq=200)
        rig.peer.heartbeat(seq=0)
        assert rig.wait_until(
            lambda: rig.backend.stream_diagnostics()["counters"]["heartbeat_seq_resets"] >= 1
        ) is True
        assert rig.backend.link_state()["restart_epoch"] == 0, (
            "序号回退不是重启判据：重启只认 boot 时间回退（保证恰好一次）"
        )


# ============================================================================
# 6. 故障容忍
# ============================================================================
def test_garbage_and_truncated_frames_do_not_kill_the_receive_thread():
    """乱码 / 坏 CRC / 截断帧之后：线程仍活、计数可见、功能照旧。"""
    with _Rig(heartbeat_timeout_s=1.0) as rig:
        rig.start(heartbeat=False)
        rig.peer.send_raw(b"\xff\xff\xff\xff not a mavlink frame at all \x00\x01\x02")
        rig.peer.send_raw(rig.peer.heartbeat_frame_with_bad_crc())
        rig.peer.send_raw(rig.peer.heartbeat_frame_truncated(keep=10))
        rig.peer.send_raw(b"\x00" * 1500)          # 超长垃圾块

        # 之后仍然能正常连通（解析器会重新同步）
        for _ in range(40):
            rig.peer.heartbeat()
            rig.peer.attitude(yaw=0.1)
            if rig.wait_connected(timeout_s=0.05):
                break
        assert rig.wait_connected(timeout_s=2.0) is True, "垃圾输入后应能恢复连通"

        diag = rig.backend.stream_diagnostics()
        assert diag["rx_thread_alive"] is True
        counters = diag["counters"]
        assert counters["parse_errors"] >= 2, "坏前缀与坏 CRC 至少各计一次"
        assert counters["pymavlink_receive_errors"] >= 2, "pymavlink 自己的计数也要涨"
        assert counters["bad_data_bytes"] > 0
        assert counters["rx_exceptions"] == 0, "畸形输入不该以异常形式打断接收循环"
        assert counters["handle_exceptions"] == 0
        assert counters["heartbeats_rx"] >= 1

        # 故障之后功能照旧：setpoint 仍能正常上线
        rig.backend.dry_run = False
        assert rig.backend.send_setpoint(_position_setpoint(0.25, 0.5, -0.75)) is True
        assert rig.peer.wait_for(pb.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) is not None


def test_close_is_idempotent_and_stops_the_thread(rig):
    assert rig.wait_connected() is True
    rig.backend.close()
    assert rig.backend.is_connected() is False
    rig.backend.close()                     # 再关一次不许炸
    rig.backend.close()
    assert rig.backend.link_state()["connected"] is False
    assert rig.backend.stream_diagnostics()["rx_thread_alive"] is False


# ============================================================================
# 7. FakePx4Backend（节点与 failsafe 接线的离线替身）
# ============================================================================
def test_fake_never_connected_without_heartbeat():
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock, heartbeat_active=False)
    assert fake.connect() is True
    assert fake.is_connected() is False
    assert fake.disconnect_reason() == "heartbeat_absent"
    assert fake.read_vehicle_state().connected is False
    fake.feed_heartbeat()
    assert fake.is_connected() is True
    assert fake.link_state()["heartbeat_age_s"] == 0.0


def test_fake_heartbeat_timeout_with_injected_clock():
    """注入时钟让「心跳超时」完全可复现，不靠 sleep。"""
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock, heartbeat_timeout_s=3.0)
    fake.connect()
    fake.feed_heartbeat()
    assert fake.is_connected() is True

    clock.advance(2.999)
    assert fake.is_connected() is True
    clock.advance(0.002)
    assert fake.is_connected() is False
    assert fake.disconnect_reason() == "heartbeat_timeout"
    assert fake.link_state()["heartbeat_age_s"] == pytest.approx(3.001)
    assert fake.read_vehicle_state().heartbeat_age_s == pytest.approx(3.001)


def test_fake_simulates_link_loss_and_recovery():
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock)
    fake.connect()
    fake.feed_heartbeat()
    assert fake.send_setpoint(_position_setpoint(0.0, 0.0, -1.0)) is True

    fake.simulate_link_loss("cable_unplugged")
    assert fake.is_connected() is False
    assert fake.disconnect_reason() == "link_down"
    assert fake.link_state()["last_error"] == "cable_unplugged"
    assert fake.send_setpoint(_position_setpoint(0.0, 0.0, -1.0)) is False
    assert fake.counters["setpoints_not_sent"] == 1
    assert fake.counters["setpoints_sent"] == 1
    assert "link_down" in [r["reason"] for r in fake.refusals.records]

    fake.restore_link()
    assert fake.is_connected() is True
    assert fake.send_setpoint(_position_setpoint(0.0, 0.0, -2.0)) is True
    assert fake.counters["setpoints_sent"] == 2


def test_fake_restart_clears_state_and_reports_once():
    """PX4 重启：boot 归零、解锁/模式作废、事件恰好一条。"""
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock)
    fake.connect()
    fake.feed_heartbeat()
    assert fake.arm() is True
    assert fake.set_offboard_mode() is True
    assert fake.read_vehicle_state().armed is True
    assert fake.read_vehicle_state().is_offboard is True

    event = fake.simulate_px4_restart(new_boot_time_ms=0)
    assert event["epoch"] == 1
    assert len(fake.restart_events) == 1
    assert fake.link_state()["restart_epoch"] == 1
    assert fake.link_state()["px4_restart_events"] == 1
    assert fake.counters["px4_restarts"] == 1

    state = fake.read_vehicle_state()
    assert state.restart_epoch == 1
    assert state.armed is False, "重启后必须回到未解锁"
    assert state.commanded_armed is None
    assert state.mode_name == "manual"
    assert state.boot_time_ms == 0
    assert fake.is_connected() is False, "重启后旧心跳作废"

    fake.feed_heartbeat()
    assert fake.is_connected() is True
    assert fake.read_vehicle_state().armed is False
    # 事件不会因为后续心跳重复
    assert fake.link_state()["restart_epoch"] == 1


def test_fake_setpoint_rejection_then_acceptance():
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock)
    fake.connect()
    fake.feed_heartbeat()
    fake.reject_next_setpoints(2)
    assert fake.send_setpoint(_position_setpoint(0.0, 0.0, -1.0)) is False
    assert fake.send_setpoint(_position_setpoint(0.0, 0.0, -1.0)) is False
    assert fake.send_setpoint(_position_setpoint(0.0, 0.0, -1.0)) is True
    assert fake.counters["setpoints_built"] == 3
    assert fake.counters["setpoints_rejected"] == 2
    assert fake.counters["setpoints_sent"] == 1
    assert len(fake.setpoints) == 1
    assert fake.stream_diagnostics()["setpoints_sent"] == 1

    fake.set_setpoint_acceptance(False)
    assert fake.send_setpoint(_position_setpoint(0.0, 0.0, -1.0)) is False
    assert fake.counters["setpoints_rejected"] == 3


def test_fake_arm_transitions_commanded_vs_acknowledged():
    """`auto_ack=False` 时暴露「命令已发但未确认」的窗口。"""
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock, auto_ack=False)
    fake.connect()
    fake.feed_heartbeat()
    assert fake.arm() is True
    state = fake.read_vehicle_state()
    assert state.armed is False, "命令已发 ≠ 已解锁"
    assert state.commanded_armed is True
    assert fake.stream_diagnostics()["acknowledged_armed"] is False

    fake.acknowledge_arm(True)
    assert fake.read_vehicle_state().armed is True
    assert fake.stream_diagnostics()["acknowledged_armed"] is True

    assert fake.disarm() is True
    assert fake.read_vehicle_state().armed is True, "auto_ack=False：确认不会自动跟随"
    assert fake.read_vehicle_state().commanded_armed is False
    fake.acknowledge_arm(False)
    assert fake.read_vehicle_state().armed is False


def test_fake_respects_allow_arming_false():
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock, allow_arming=False)
    fake.connect()
    fake.feed_heartbeat()
    assert fake.arm() is False
    assert fake.counters["arm_refused"] == 1
    assert fake.stream_diagnostics()["last_refusal"]["reason"] == pb.REFUSAL_ARMING_NOT_ALLOWED
    assert fake.arm() is False and fake.counters["arm_refused"] == 2
    assert fake.disarm() is False and fake.counters["arm_refused"] == 3


def test_fake_mode_transitions_and_unknown_mode():
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock)
    fake.connect()
    fake.feed_heartbeat()
    assert fake.set_offboard_mode() is True
    assert fake.read_vehicle_state().mode_name == "offboard"
    assert fake.read_vehicle_state().is_offboard is True
    assert fake.commands[pb.MAV_CMD_DO_SET_MODE]["param"] == pb.px4_custom_mode(
        pb.PX4_CUSTOM_MAIN_MODE_OFFBOARD
    )
    assert fake.set_mode("auto:loiter") is True
    state = fake.read_vehicle_state()
    assert state.mode_name == "auto" and state.custom_sub_mode == 3
    assert fake.set_position_mode() is True
    assert fake.read_vehicle_state().mode_name == "posctl"
    assert fake.set_mode("hyperdrive") is False
    assert fake.stream_diagnostics()["last_refusal"]["reason"] == pb.REFUSAL_UNKNOWN_MODE


def test_fake_records_every_call():
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock)
    fake.connect()
    fake.feed_heartbeat()
    fake.set_mode("offboard")
    fake.arm()
    fake.send_setpoint(_position_setpoint(1.0, 2.0, 3.0))
    names = [call.name for call in fake.calls]
    assert names == ["connect", "set_mode:offboard", "arm", "send_setpoint"]
    arm_calls = fake.calls_named("arm")
    assert len(arm_calls) == 1
    assert arm_calls[0].accepted is True
    assert arm_calls[0].args == (pb.ARMING_ACTION_ARM,)
    assert arm_calls[0].mono_s == clock()
    spoof = fake.calls_named("send_setpoint")[0].args[0]
    assert spoof.position_m == (1.0, 2.0, 3.0)
    fake.clear_calls()
    assert fake.calls == []


def test_fake_command_rejection_and_connect_failure_are_visible():
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock, connect_succeeds=False)
    assert fake.connect() is False
    assert fake.counters["connect_failures"] == 1
    assert "connect_failed" in (fake.link_state()["last_error"] or "")

    ok = FakePx4Backend(clock=clock)
    ok.connect()
    ok.feed_heartbeat()
    ok.set_command_acceptance(False)
    assert ok.set_offboard_mode() is False
    assert ok.stream_diagnostics()["last_refusal"]["reason"] == (
        pb.REFUSAL_COMMAND_REJECTED_BY_FAKE
    )
    assert ok.counters["commands_sent"] == 0


def test_fake_feeds_attitude_and_position_with_ages():
    clock = ManualClock()
    fake = FakePx4Backend(clock=clock)
    fake.connect()
    fake.feed_heartbeat()
    fake.feed_attitude(0.33)
    fake.feed_position_ned(1.0, 2.0, -3.0)
    fake.feed_velocity_ned(0.1, 0.2, 0.3)
    clock.advance(0.5)
    state = fake.read_vehicle_state()
    assert state.yaw_rad == pytest.approx(0.33)
    assert state.position_ned_m == (1.0, 2.0, -3.0)
    assert state.velocity_ned_m_s == (0.1, 0.2, 0.3)
    assert state.attitude_age_s == pytest.approx(0.5)
    assert state.attitude_received_mono_s == pytest.approx(clock() - 0.5)
    assert state.position_age_s == pytest.approx(0.5)
    assert state.source == "fake"
    assert fake.counters["attitude_rx"] == 1
    assert fake.counters["local_position_rx"] == 1


def test_fake_has_no_actuator_surface_either():
    """Fake 也不能成为绕过 FC-004 的后门。"""
    fake = FakePx4Backend()
    pb.assert_no_actuator_surface(fake, label="FakePx4Backend 实例")
    for attr in dir(fake):
        if attr.startswith("_"):
            continue
        for pattern in pb.FORBIDDEN_ACTUATOR_NAME_PATTERNS:
            assert pattern not in attr.lower(), f"Fake 暴露了 {attr!r}（命中 {pattern!r}）"


def test_no_float_nan_leaks_into_diagnostics(rig):
    """诊断必须能 JSON 序列化：不许把 NaN/inf 塞进 dict。"""
    import json

    rig.peer.attitude(yaw=0.1)
    rig.peer.local_position_ned(1.0, 2.0, -3.0)
    assert rig.wait_until(
        lambda: rig.backend.read_vehicle_state().position_ned_m is not None
    ) is True
    text = json.dumps(rig.backend.stream_diagnostics(), allow_nan=False)
    assert "restart_epoch" in text
    fake_text = json.dumps(FakePx4Backend().stream_diagnostics(), allow_nan=False)
    assert "counters" in fake_text
    # 顺带确认没有把无穷大年龄写进状态
    state = rig.backend.read_vehicle_state()
    for value in (state.heartbeat_age_s, state.attitude_age_s, state.position_age_s):
        assert value is None or math.isfinite(value)


# ---------------------------------------------------------------- 位域回归
# 背景：SITL 实测 HEARTBEAT.custom_mode=50593792(0x03040000) = AUTO/LOITER(3)，
# 但工程原先 >>8/>>16 解出 main=0、sub=4。Fake 自编自解一致，所以只有用**真实
# 整数**才能测出这个错。以下值全部来自 PX4 SITL 的实收报文与 px4_custom_mode.h。
def test_px4_custom_mode_bitfield_matches_real_px4_layout():
    from boom_birds_nav.px4_backend import (
        px4_custom_main_mode_name,
        px4_custom_mode,
        px4_custom_sub_mode,
    )

    # SIH 默认悬停：main_mode=4(AUTO), sub_mode=3(LOITER)
    real = 50593792
    assert real == 0x03040000
    assert px4_custom_main_mode_name(real) == "auto"
    assert px4_custom_sub_mode(real) == 3

    # OFFBOARD 由 PX4 定义为 0x00060000（main=6）
    offboard = 0x00060000
    assert px4_custom_main_mode_name(offboard) == "offboard"
    assert px4_custom_sub_mode(offboard) == 0

    # 编码必须与解码互为逆（且和真实 int 一致）
    assert px4_custom_mode(4, 3) == real
    assert px4_custom_mode(6, 0) == offboard
    assert px4_custom_mode(1, 0) == 0x00010000          # MANUAL
    for main in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11):
        for sub in (0, 1, 3, 6):
            assert px4_custom_sub_mode(px4_custom_mode(main, sub)) == sub


def test_vehicle_state_offboard_detected_from_real_custom_mode():
    """真 OFFBOARD 必须被认出来：这是安全相关判定，错了会导致误判未接管。"""
    import inspect

    from boom_birds_nav.px4_backend import VehicleState

    sig = inspect.signature(VehicleState)
    if "custom_main_mode" not in sig.parameters:
        import pytest as _pytest

        _pytest.skip("VehicleState 无 custom_main_mode 字段")

    st = VehicleState(connected=True, custom_main_mode=6, custom_mode=0x00060000)
    assert st.is_offboard is True
    st2 = VehicleState(connected=True, custom_main_mode=4, custom_mode=50593792)
    assert st2.is_offboard is False


# ---------------------------------------------------------------- 诊断不掩盖
def test_mavlink_rejected_source_does_not_mask_last_error():
    """来源拒绝是设计行为：必须只计数/只记最近一次，不得写进 last_error。

    背景（SITL 实测）：PX4 持续用 sysid=0 发 TIMESYNC，若每条都写 last_error，
    真正的错误会被长期掩盖，诊断字段等于失效。
    """
    import inspect

    source = inspect.getsource(pb.MavlinkPx4Backend._handle_message)
    assert "rejected_source" in source
    # 拒绝分支里不得再出现 last_error 赋值
    reject_block = source.split("if not self._accept_source", 1)[1].split("return", 1)[0]
    # 只看可执行赋值（注释里提到 last_error 是允许的）
    code_lines = [ln.strip() for ln in reject_block.splitlines()
                  if ln.strip() and not ln.strip().startswith("#")]
    assert not any(ln.startswith("self.last_error") for ln in code_lines), (
        f"来源拒绝仍写 last_error：{code_lines!r}"
    )

    keys_src = inspect.getsource(pb.MavlinkPx4Backend.link_state)
    assert "rejected_source_last" in keys_src
    assert "sources_rejected" in keys_src
