"""会话、序号、轨迹与有效期检查；不含飞控执行许可判断。"""
from boom_birds_control.runtime_config import DEFAULTS
from dataclasses import dataclass
import math
import uuid

EXECUTE, CANCEL, HOLD = 1, 2, 3

@dataclass(frozen=True)
class Command:
    session_id: str
    trajectory_id: int
    sequence: int
    stamp: float
    valid_for: float
    kind: int
    frame: str = DEFAULTS.world_frame
    position: tuple = (0., 0., 0.)
    velocity: tuple = (0., 0., 0.)
    acceleration: tuple = (0., 0., 0.)
    yaw: float = 0.
    yaw_rate: float = 0.
    landing: bool = False
    ground_z_world_m: float = 0.

class ControlIngress:
    def __init__(self, max_ttl=DEFAULTS.command_timeout_s, future_tolerance=DEFAULTS.command_future_tolerance_s):
        if not math.isfinite(max_ttl) or max_ttl <= 0:
            raise ValueError("max_ttl")
        if not math.isfinite(future_tolerance) or future_tolerance <= 0:
            raise ValueError("future_tolerance")
        self.future_tolerance = future_tolerance
        self.max_ttl = max_ttl
        self.session = ""
        self.current = None
        self.sequence = 0
        self.trajectory = 0
        self.retired = 0
        self.deadline = -math.inf
        self.last_ros = None
        self.reason = "no_session"

    def open_session(self):
        self.session = str(uuid.uuid4())
        self.current = None
        self.sequence = self.trajectory = self.retired = 0
        self.deadline = -math.inf
        self.last_ros = None
        self.reason = "no_command"
        return self.session

    def cancel(self, reason="cancelled"):
        self.retired = max(self.retired, self.trajectory)
        self.current = None
        self.deadline = -math.inf
        self.reason = reason

    def close(self, reason):
        self.cancel(reason)
        self.session = ""

    def receive(self, cmd, ros_now, mono_now):
        if not self.session or cmd.session_id != self.session:
            return False, "old_session"
        if type(cmd.sequence) is not int or cmd.sequence <= self.sequence:
            return False, "unordered"
        if type(cmd.trajectory_id) is not int or cmd.trajectory_id <= 0:
            return False, "invalid_trajectory"
        if cmd.kind not in (EXECUTE, CANCEL, HOLD):
            return False, "unknown_command"
        if cmd.kind == CANCEL:
            # 取消无需延续旧 setpoint 的有效期；同一流的高序号取消形成屏障。
            self.sequence = cmd.sequence
            self.retired = max(self.retired, cmd.trajectory_id, self.trajectory)
            self.cancel()
            return True, "cancelled"
        if cmd.trajectory_id <= self.retired or cmd.trajectory_id < self.trajectory:
            return False, "retired_trajectory"
        if type(cmd.landing) is not bool or (cmd.landing and cmd.kind != HOLD):
            return False, "invalid_landing_command"
        scalars = (cmd.ground_z_world_m, cmd.stamp, cmd.valid_for, ros_now, mono_now, cmd.yaw, cmd.yaw_rate)
        vectors = (cmd.position, cmd.velocity, cmd.acceleration)
        if any(len(v) != 3 for v in vectors) or not all(math.isfinite(x) for x in (*scalars, *cmd.position, *cmd.velocity, *cmd.acceleration)):
            return False, "nonfinite"
        if not cmd.frame or not 0 < cmd.valid_for <= self.max_ttl:
            return False, "invalid_validity"
        if not -self.future_tolerance <= ros_now - cmd.stamp < cmd.valid_for:
            return False, "expired"
        if self.last_ros is not None and ros_now < self.last_ros - self.future_tolerance:
            self.close("clock_reset")
            return False, self.reason
        if cmd.trajectory_id > self.trajectory:
            self.retired = max(self.retired, self.trajectory)
        self.trajectory = cmd.trajectory_id
        self.sequence = cmd.sequence
        self.current = cmd
        self.last_ros = ros_now
        self.deadline = mono_now + min(cmd.valid_for, cmd.stamp + cmd.valid_for - ros_now)
        self.reason = "accepted"
        return True, self.reason

    def active(self, ros_now, mono_now):
        if self.last_ros is not None and ros_now < self.last_ros - self.future_tolerance:
            self.close("clock_reset")
        elif self.current is not None and (not math.isfinite(ros_now) or not math.isfinite(mono_now) or mono_now >= self.deadline or ros_now >= self.current.stamp + self.current.valid_for):
            self.cancel("expired")
        self.last_ros = ros_now
        return self.current
