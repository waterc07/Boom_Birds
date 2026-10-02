"""MAVROS router 已校验报文的只读状态字段；不编码或发送 MAVLink。"""
import struct
from types import SimpleNamespace

# MAVLink common/development 的线上字段顺序。MAVLink 2 允许尾部零截断。
_LAYOUTS = {
    0: ("<IBBBBB", ("custom_mode", "type", "autopilot", "base_mode", "system_status", "mavlink_version")),
    1: ("<III", ("onboard_control_sensors_present", "onboard_control_sensors_enabled", "onboard_control_sensors_health")),
    30: ("<Iffffff", ("time_boot_ms", "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed")),
    32: ("<Iffffff", ("time_boot_ms", "x", "y", "z", "vx", "vy", "vz")),
    77: ("<HB", ("command", "result")),
    245: ("<BB", ("vtol_state", "landed_state")),
    436: ("<IIB", ("custom_mode", "intended_custom_mode", "standard_mode")),
}

def decode_observation(msg):
    if int(msg.framing_status) != 1:  # FRAMING_OK
        return None
    mid = int(msg.msgid)
    if mid not in _LAYOUTS and mid not in (331, 253):
        return None
    data = b"".join(struct.pack("<Q", int(v)) for v in msg.payload64)
    length = int(msg.len)
    if length < 1 or length > len(data) or length > 255:
        raise ValueError("invalid MAVROS payload length")
    data = data[:length]
    if mid == 253:
        fields = {"severity": data[0], "text": data[1:51].split(b"\0", 1)[0].decode("utf-8", errors="replace")}
    elif mid == 331:
        # ODOMETRY: MAVLink 2 extension reset_counter 位于 byte 230。
        # 没有扩展字段时其协议值为 0，不把缺失数据当作消息不存在。
        fields = {"reset_counter": data[230] if len(data) > 230 else 0}
    else:
        fmt, names = _LAYOUTS[mid]
        size = struct.calcsize(fmt)
        fields = dict(zip(names, struct.unpack(fmt, data[:size].ljust(size, b"\0"))))
    return SimpleNamespace(**fields, get_msgId=lambda: mid,
        get_srcSystem=lambda: int(msg.sysid), get_srcComponent=lambda: int(msg.compid),
        get_seq=lambda: int(msg.seq))
