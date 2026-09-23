"""OpenVINS 位姿 → 机体里程计与校正相机位姿的适配节点。

配对模型（按**采集时间**配对，不用"最新位姿 + 大容差"）：
  1. 位姿流（OpenVINS odomimu，header.stamp = 该位姿对应的观测时刻）进入位姿缓冲；
  2. 深度帧只保留**一个待处理观测**（pending），携带其采集时间与到达时间；
  3. 每个 tick 用 pending 深度的采集时间去位姿缓冲里找配对：
     - 两侧都有位姿 → 线性插值（位置线性、姿态四元数 SLERP），仅当两侧间隔
       不超过 pose_interp_max_gap_s；
     - 只有一侧或间隔过大 → 取时间上最近的位姿，且 |Δt| 必须 ≤ pose_depth_tolerance_s；
     - |Δt| 超容差 → 拒绝该深度帧（计入 pair_reject_count），用下一帧继续。
  4. pose_wait_s 只决定"等多久算超时"：pending 深度到达后超过该时长仍未配上就丢弃
     （计入 wait_timeout_count），**不再混入后续地图**；它不参与采集时间差判定。
  5. depth_max_age_s 只决定"结果是否已经太旧"：配对成功时若
     now − 深度到达时刻 > depth_max_age_s，则拒绝（计入 stale_depth_count）。
  6. 队列容量：位姿缓冲上限 pose_queue_size，超出丢最旧；pending 深度最多一个，
     等待超时即淘汰。乱序（更早的深度）允许，但不会把已淘汰的观测重新激活。

输出：
  - /boom_birds/vio/camera_pose : 配对成功时发布，时间戳 = **该观测时刻**（深度采集时间）；
  - /boom_birds/vio/odom_body   : 标准机体里程计（twist 在机体系）；
  - /boom_birds/vio/odom_ego    : EGO 专用（twist 为世界系速度）。

时效状态：vio_valid 与 depth_valid 分开维护。深度失效时只停发相机位姿（地图链失效），
机体里程计与 EGO 里程计继续发布，不把 VIO 判为失效。

重置：~/reset 清空状态并**闭锁**（忽略后续输入），~/rearm 显式重新启用并要求重新配对。
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image

from .config_io import load_extrinsics
from .depth_core import camera_rect_transform, make_processor
from .frames import body_velocity_from_imu, quat_to_rot, rot_to_quat

QOS_RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)


def _slerp(q0, q1, u: float):
    """单位四元数 SLERP（(x,y,z,w) 顺序）；两四元数夹角过小时退化为线性插值再归一化。"""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    d = float(np.dot(q0, q1))
    if d < 0.0:                      # 取短弧
        q1 = -q1
        d = -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    theta0 = np.arccos(np.clip(d, -1.0, 1.0))
    theta = theta0 * u
    q2 = q1 - q0 * d
    q2 = q2 / np.linalg.norm(q2)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


class PoseAdapter(Node):
    def __init__(self) -> None:
        super().__init__("boom_birds_pose_adapter")
        self.declare_parameter("extrinsics_file", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("odom_imu_topic", "/boom_birds/ov/odomimu")
        self.declare_parameter("depth_topic", "/boom_birds/depth/image")
        self.declare_parameter("camera_pose_topic", "/boom_birds/vio/camera_pose")
        self.declare_parameter("odom_body_topic", "/boom_birds/vio/odom_body")
        self.declare_parameter("odom_ego_topic", "/boom_birds/vio/odom_ego")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("frame_world", "global")
        self.declare_parameter("frame_body", "body")
        self.declare_parameter("use_depth_gate", True)

        # ---- 三个时间量，语义互不替代 ----
        # 采集时间配对容差：两条数据表达的时刻允许多大差异
        self.declare_parameter("pose_depth_tolerance_s", 0.03)
        # 等待时长：pending 深度最多等多久（吸收算法输出延迟，不参与采集时间差判定）
        self.declare_parameter("pose_wait_s", 0.5)
        # 数据年龄上限：配对成功时结果是否已经太旧
        self.declare_parameter("depth_max_age_s", 0.5)
        # 位姿自身过期（VIO 停更）
        self.declare_parameter("pose_timeout_s", 0.15)
        # 深度完全缺失（没有任何 pending 观测）超过该值 → 地图链失效
        self.declare_parameter("depth_timeout_s", 1.0)
        # 允许的轻微时钟超前；超过即拒绝（未来时间戳）
        self.declare_parameter("future_stamp_tolerance_s", 0.05)
        # 位姿缓冲容量与插值上限
        self.declare_parameter("pose_queue_size", 200)
        self.declare_parameter("pose_interp_max_gap_s", 0.15)

        extr_path = self.get_parameter("extrinsics_file").value
        calib_path = self.get_parameter("calibration_file").value
        if not extr_path:
            raise RuntimeError("必须显式提供 extrinsics_file（缺外参不允许静默占位）")
        if not calib_path:
            raise RuntimeError("必须显式提供 calibration_file")
        self.extr = load_extrinsics(extr_path)
        self.processor = make_processor(calib_path)
        self.T_i_b = self.extr["T_I_B"]
        self.T_i_c0 = self.extr["T_I_C0"]
        self.p_i_b = self.extr["p_I_B"]
        self.frame_world = self.get_parameter("frame_world").value
        self.frame_body = self.get_parameter("frame_body").value

        self.tol = float(self.get_parameter("pose_depth_tolerance_s").value)
        self.pose_wait = float(self.get_parameter("pose_wait_s").value)
        self.depth_max_age = float(self.get_parameter("depth_max_age_s").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_s").value)
        self.depth_timeout = float(self.get_parameter("depth_timeout_s").value)
        self.future_tol = float(self.get_parameter("future_stamp_tolerance_s").value)
        self.pose_queue_size = int(self.get_parameter("pose_queue_size").value)
        self.interp_max_gap = float(self.get_parameter("pose_interp_max_gap_s").value)
        self.use_depth_gate = bool(self.get_parameter("use_depth_gate").value)
        if self.tol >= self.depth_max_age:
            self.get_logger().error(
                "配置错误：pose_depth_tolerance_s 必须显著小于 depth_max_age_s（前者是采集时间差，后者是数据年龄）"
            )

        # ---- 缓冲与状态 ----
        self.pose_buffer = []          # [(t, received_at, T_w_i, v_i, w_i)]，按 t 递增
        self.pending_depth = None      # (t_capture, received_at)
        self.latest_pose = None        # 最近一次位姿（供里程计使用）
        self.last_published_depth_t = None
        self.last_accepted_pose_t = None
        self.last_camera_pose_t = None
        self._last_pair_wall = 0.0
        self.reset_latched = False
        self.input_cov6 = None

        self.vio_valid = False
        self.depth_valid = False
        self.blocked_by = ""
        self._depth_gate_msg = ""
        self._pair_msg = ""

        # 计数
        self.pub_count = 0
        self.pair_count = 0
        self.interp_count = 0
        self.nearest_count = 0
        self.pair_reject_count = 0
        self.wait_timeout_count = 0
        self.stale_depth_count = 0
        self.gated_pose_count = 0
        self.time_regression_count = 0
        self.pose_drop_count = 0
        self.last_pair_dt = None

        self.pub_camera = self.create_publisher(PoseStamped, self.get_parameter("camera_pose_topic").value, QOS_RELIABLE)
        self.pub_body = self.create_publisher(Odometry, self.get_parameter("odom_body_topic").value, QOS_RELIABLE)
        self.pub_ego = self.create_publisher(Odometry, self.get_parameter("odom_ego_topic").value, QOS_RELIABLE)

        self.create_subscription(Odometry, self.get_parameter("odom_imu_topic").value, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("depth_topic").value, self.on_depth, QOS_RELIABLE)

        from std_srvs.srv import Trigger

        self.reset_srv = self.create_service(Trigger, "~/reset", self.on_reset)
        self.rearm_srv = self.create_service(Trigger, "~/rearm", self.on_rearm)

        rate = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)
        self.report_timer = self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"pose_adapter 启动：外参来源={self.extr['source']}；杆臂 p_I_B={np.round(self.p_i_b, 6).tolist()} m；"
            f"采集时间容差 {self.tol:.3f}s / 等待 {self.pose_wait:.3f}s / 数据年龄上限 {self.depth_max_age:.3f}s / "
            f"插值上限间隔 {self.interp_max_gap:.3f}s"
        )

    # ---------------- 输入 ----------------
    def on_odom(self, msg: Odometry) -> None:
        if self.reset_latched:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_wall = self.get_clock().now().nanoseconds * 1e-9
        if t > now_wall + self.future_tol:
            self.gated_pose_count += 1
            if self.gated_pose_count == 1 or self.gated_pose_count % 50 == 0:
                self.get_logger().warn(
                    "拒绝未来时间戳的位姿：t=%.3f > now=%.3f + %.3f（累计 %d 次）"
                    % (t, now_wall, self.future_tol, self.gated_pose_count)
                )
            return
        if self.last_accepted_pose_t is not None and t < self.last_accepted_pose_t - 1e-9:
            # 乱序/回放回退：拒绝比已接受更早的位姿，避免时间基准错乱
            self.time_regression_count += 1
            if self.time_regression_count == 1 or self.time_regression_count % 50 == 0:
                self.get_logger().warn(
                    "拒绝时间倒退的位姿：t=%.3f < last=%.3f（累计 %d 次）"
                    % (t, self.last_accepted_pose_t, self.time_regression_count)
                )
            return

        q = msg.pose.pose.orientation
        T_w_i = np.eye(4)
        T_w_i[:3, :3] = quat_to_rot((q.x, q.y, q.z, q.w))
        T_w_i[:3, 3] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        v_i = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        w_i = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        self.pose_buffer.append((t, now_wall, T_w_i, v_i, w_i, (q.x, q.y, q.z, q.w)))
        self.pose_buffer.sort(key=lambda e: e[0])
        if len(self.pose_buffer) > self.pose_queue_size:
            self.pose_drop_count += len(self.pose_buffer) - self.pose_queue_size
            self.pose_buffer = self.pose_buffer[-self.pose_queue_size :]
        self.last_accepted_pose_t = t
        self.latest_pose = (t, T_w_i, v_i, w_i)
        cov = np.asarray(msg.pose.covariance, dtype=float)
        self.input_cov6 = cov.reshape(6, 6) if cov.size == 36 and np.any(cov) else None

    def on_depth(self, msg: Image) -> None:
        if self.reset_latched:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_wall = self.get_clock().now().nanoseconds * 1e-9
        if stamp <= 0.0:
            self.get_logger().warn("收到 header.stamp 为 0 的深度帧，已丢弃", once=True)
            return
        if stamp > now_wall + self.future_tol:
            self.gated_pose_count += 1
            return
        if self.pending_depth is not None:
            # 只保留最新待处理观测：上一个还没配上就被替换，计入等待超时
            self.wait_timeout_count += 1
        self.pending_depth = (stamp, now_wall)

    # ---------------- 配对 ----------------
    def _pair_pending(self, now: float):
        """尝试为 pending 深度找配对位姿。

        返回 (T_w_i, v_i, w_i, t_pose, kind) 或 None；kind ∈ {interp, nearest}。
        无论成功与否都会按等待时长淘汰过期 pending。
        """
        if self.pending_depth is None:
            return None
        t_cap, t_recv = self.pending_depth
        # 等待超时：淘汰，不再参与后续配对（不会"后续到达又混入"）
        if (now - t_recv) > self.pose_wait:
            self.pending_depth = None
            self.wait_timeout_count += 1
            self._depth_gate_note("pending 深度等待超时（%.3fs > %.3fs），已淘汰" % (now - t_recv, self.pose_wait))
            return None
        if not self.pose_buffer:
            return None

        # 找时间上夹住 t_cap 的两个位姿
        before = None
        after = None
        for entry in self.pose_buffer:
            if entry[0] <= t_cap:
                before = entry
            elif after is None:
                after = entry
                break
        best = None
        kind = "nearest"
        can_interp = (
            before is not None
            and after is not None
            and (after[0] - before[0]) <= self.interp_max_gap
            and before[0] <= t_cap <= after[0]
        )
        if can_interp:
            kind = "interp"
            best = before
            # 插值时刻落在两端之间，采集时间差定义为到最近一端的距离
            dt = min(t_cap - before[0], after[0] - t_cap)
        else:
            cands = [e for e in (before, after) if e is not None]
            if not cands:
                return None
            best = min(cands, key=lambda e: abs(e[0] - t_cap))
            dt = abs(best[0] - t_cap)
        if dt > self.tol:
            self.pair_reject_count += 1
            self._pair_note(
                "位姿与深度的采集时间差 %.4fs 超出容差 %.4fs：拒绝该深度观测" % (dt, self.tol)
            )
            self.pending_depth = None
            return None
        if (now - t_recv) > self.depth_max_age:
            self.stale_depth_count += 1
            self._pair_note("配对成功但数据已太旧（到达至今 %.3fs > %.3fs）：拒绝" % (now - t_recv, self.depth_max_age))
            self.pending_depth = None
            return None

        if kind == "interp":
            u = 0.0 if after[0] == before[0] else (t_cap - before[0]) / (after[0] - before[0])
            T = np.eye(4)
            q = _slerp(before[5], after[5], u)
            T[:3, :3] = quat_to_rot(q)
            T[:3, 3] = (1 - u) * before[2][:3, 3] + u * after[2][:3, 3]
            v = (1 - u) * before[3] + u * after[3]
            w = (1 - u) * before[4] + u * after[4]
            self.interp_count += 1
        else:
            T, v, w = best[2], best[3], best[4]
            self.nearest_count += 1

        self.last_pair_dt = float(dt)
        self.last_camera_pose_t = float(t_cap)
        self._last_pair_wall = now
        self.pair_count += 1
        self.pending_depth = None
        # 清理已用不到的旧位姿（保留一点余量给插值）
        cutoff = t_cap - max(self.interp_max_gap, self.tol) * 2.0
        self.pose_buffer = [e for e in self.pose_buffer if e[0] >= cutoff]
        return T, v, w, float(t_cap), kind

    # ---------------- 周期输出 ----------------
    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.reset_latched:
            self._blocked("已重置并闭锁：等待 ~/rearm 显式重新启用（上游数据被忽略）")
            return

        # VIO 状态（独立维护）
        if self.latest_pose is None:
            self.vio_valid = False
            self._blocked("尚无 VIO 里程计输入")
            return
        t_pose, T_w_i, v_i, w_i = self.latest_pose
        if (now - t_pose) > self.pose_timeout:
            self.vio_valid = False
            self._blocked(f"VIO 位姿过期 {(now - t_pose):.3f}s > {self.pose_timeout:.3f}s")
            return
        self.vio_valid = True

        # 深度链状态与配对
        paired = self._pair_pending(now)
        if paired is not None:
            self.depth_valid = True
        else:
            self.depth_valid = False
            # 深度链失效条件：距上次成功配对超过 depth_timeout_s（或从未成功过）
            if (now - self._last_pair_wall) > self.depth_timeout:
                self._depth_gate_note(
                    "深度链超过 %.3fs 没有成功配对：停发相机位姿（地图链失效）；"
                    "VIO 与机体里程计仍有效" % self.depth_timeout
                )

        # 机体里程计与 EGO 里程计只依赖 VIO，深度失效时继续发布
        T_w_b = T_w_i @ self.T_i_b
        v_w_i = T_w_i[:3, :3] @ v_i
        v_w_b = body_velocity_from_imu(v_w_i, w_i, self.p_i_b, T_w_i[:3, :3])
        stamp_now = self._stamp_from_seconds(t_pose)

        body = self._odom_msg(T_w_b, v_w_b, w_i, stamp_now)
        v_b = T_w_b[:3, :3].T @ v_w_b
        body.twist.twist.linear.x, body.twist.twist.linear.y, body.twist.twist.linear.z = (
            float(v_b[0]), float(v_b[1]), float(v_b[2]),
        )
        R_b_i = self.T_i_b[:3, :3].T
        w_b = R_b_i @ w_i
        body.twist.twist.angular.x, body.twist.twist.angular.y, body.twist.twist.angular.z = (
            float(w_b[0]), float(w_b[1]), float(w_b[2]),
        )
        self.pub_body.publish(body)
        self.pub_ego.publish(self._odom_msg(T_w_b, v_w_b, w_i, stamp_now))

        # 相机位姿：只在配对成功时发布，时间戳 = 该观测时刻（深度采集时间）
        if paired is not None:
            T_p, v_p, w_p, t_cap, kind = paired
            T_w_crect = camera_rect_transform(self.processor, T_p, self.T_i_c0)
            cam = self._pose_msg(T_w_crect, self._stamp_from_seconds(t_cap))
            cam.header.frame_id = self.frame_world
            self.pub_camera.publish(cam)
            self.pub_count += 1

    def last_pair_wall(self) -> float:
        """最近一次成功配对成功时的到达时刻（用于深度链超时判定）。"""
        return getattr(self, "_last_pair_wall", 0.0)

    def _pair_note(self, reason: str) -> None:
        if reason != self._pair_msg:
            self.get_logger().warn(reason)
            self._pair_msg = reason

    # ---------------- 输出消息 ----------------
    def _pose_msg(self, T: np.ndarray, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_world
        p = T[:3, 3]
        q = rot_to_quat(T[:3, :3])
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(p[0]), float(p[1]), float(p[2])
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = (
            float(q[0]), float(q[1]), float(q[2]), float(q[3]),
        )
        return msg

    def _odom_msg(self, T: np.ndarray, v_w: np.ndarray, w_i: np.ndarray, stamp) -> Odometry:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_world
        msg.child_frame_id = self.frame_body
        p = T[:3, 3]
        q = rot_to_quat(T[:3, :3])
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = float(p[0]), float(p[1]), float(p[2])
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
            float(q[0]), float(q[1]), float(q[2]), float(q[3]),
        )
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = (
            float(v_w[0]), float(v_w[1]), float(v_w[2]),
        )
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = (
            float(w_i[0]), float(w_i[1]), float(w_i[2]),
        )
        if self.input_cov6 is not None:
            from .frames import transform_covariance_6x6

            cov = transform_covariance_6x6(T, self.input_cov6)
            msg.pose.covariance = [float(v) for v in cov.reshape(-1)]
            msg.twist.covariance = [float(v) for v in cov.reshape(-1)]
        return msg

    def _blocked(self, reason: str) -> None:
        if reason != self.blocked_by:
            self.get_logger().warn(f"停止发布：{reason}")
            self.blocked_by = reason

    def _depth_gate_note(self, reason: str) -> None:
        if reason != self._depth_gate_msg:
            self.get_logger().warn(reason)
            self._depth_gate_msg = reason

    def _stamp_from_seconds(self, seconds: float):
        if seconds <= 0.0:
            return self.get_clock().now().to_msg()
        msg = self.get_clock().now().to_msg()
        msg.sec = int(seconds)
        msg.nanosec = int(round((seconds - int(seconds)) * 1e9))
        if msg.nanosec >= 1000000000:
            msg.sec += 1
            msg.nanosec -= 1000000000
        return msg

    def report(self) -> None:
        self.get_logger().info(
            f"pose_adapter 统计：相机位姿={self.pub_count} 配对成功={self.pair_count}"
            f"（插值 {self.interp_count}/最近 {self.nearest_count}）"
            f" 采集时间超差={self.pair_reject_count} 等待超时={self.wait_timeout_count} "
            f"数据过旧={self.stale_depth_count} 位姿缓冲={len(self.pose_buffer)} 丢弃位姿={self.pose_drop_count} "
            f"未来戳拒绝={self.gated_pose_count} 时间倒退={self.time_regression_count} "
            f"最近配对差={('%.4fs' % self.last_pair_dt) if self.last_pair_dt is not None else 'n/a'} "
            f"当前状态={self.blocked_by or '正常'}"
        )

    # ---------------- 重置 ----------------
    def on_reset(self, request, response):
        self.reset_latched = True
        self.pose_buffer = []
        self.pending_depth = None
        self.latest_pose = None
        self.last_accepted_pose_t = None
        self.last_camera_pose_t = None
        self.input_cov6 = None
        self.vio_valid = False
        self.depth_valid = False
        self.blocked_by = ""
        self.get_logger().warn(
            "收到重置请求：已清空位姿/深度缓冲并闭锁；需 ~/rearm 显式重新启用"
        )
        response.success = True
        response.message = "pose_adapter reset and latched"
        return response

    def on_rearm(self, request, response):
        self.reset_latched = False
        self.pose_buffer = []
        self.pending_depth = None
        self.latest_pose = None
        self.last_accepted_pose_t = None
        self.last_camera_pose_t = None
        self.get_logger().warn("收到重新启用请求：已清空缓冲，等待新的配对")
        response.success = True
        response.message = "pose_adapter re-armed"
        return response


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = None
    try:
        node = PoseAdapter()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_nav] pose_adapter 启动失败：{exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
