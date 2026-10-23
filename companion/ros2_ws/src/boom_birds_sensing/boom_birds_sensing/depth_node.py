"""深度与完整 XYZ 的 ROS 2 节点（脱机）。

契约：
- 订阅左右两路原始图（唯一采集源语义），同帧合成后复用 StereoProcessor；
- 发布 32FC1 米制深度与主 XYZ；无效值保持 NaN；
- 16UC1 毫米兼容话题的无效值为整数 0；
- 发布保持 H×W 对应的主点云（is_dense=false）与附加的紧凑点云；
- 发布由同一标定 P1 推导的 CameraInfo，供地图使用，避免两套内参。
"""

from __future__ import annotations

from boom_birds_control.runtime_config import DEFAULTS


from stereo_depth.orientation import SENSOR, orientStitched

import cv2
import message_filters
import numpy as np
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import Header

from boom_birds_sensing.depth_core import (
    annotateValidity,
    makeProcessor,
    processStitched,
    processPair,
    rectifiedCameraInfo,
    xyzToCompact,
)
from boom_birds_sensing.camera_geometry import scaledCameraInfo
from boom_birds_sensing.frame_worker import LatestFrameWorker
from boom_birds_sensing.ros_msg import cloud2FromStructured, cloud2XyzHw

QOS_IMAGE = QoSProfile(
    depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
)


class DepthNode(Node):
    def __init__(self, *, localInput=False) -> None:
        super().__init__("boom_birds_depth")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("input_transport", "raw")
        self.declare_parameter("mjpeg_topic", DEFAULTS.stereo_stitched_topic + "/compressed")
        self.declare_parameter("left_topic", DEFAULTS.stereo_left_topic)
        self.declare_parameter("right_topic", DEFAULTS.stereo_right_topic)
        self.declare_parameter("depth_topic", DEFAULTS.depth_topic)
        self.declare_parameter("xyz_topic", DEFAULTS.depth_xyz_topic)
        self.declare_parameter("xyz_valid_topic", DEFAULTS.depth_xyz_valid_topic)
        self.declare_parameter("camera_info_topic", DEFAULTS.camera_info_topic)
        self.declare_parameter("frame_id", DEFAULTS.camera_frame)
        self.declare_parameter("min_depth_m", 0.2)
        # 量程默认值只能有一个定义点（RuntimeConfig）；这里写死 5.0 就是第二份定义，
        # 改基准字段时它会静默落后（独立核验 F1 实测）。
        self.declare_parameter("max_depth_m", DEFAULTS.depth_max_range_m)
        self.declare_parameter("publish_xyz", True)
        self.declare_parameter("publish_compact_xyz", True)
        # 主话题保持契约要求的 NaN；另发一个 0 值兼容话题给只接受 uint16 毫米流的消费者。
        self.declare_parameter("depth_compat_topic", DEFAULTS.depth_compat_topic)
        self.declare_parameter("publish_depth_compat", True)
        self.declare_parameter("process_latest_only", False)
        self.declare_parameter("max_processing_rate_hz", 0.0)
        self.declare_parameter("opencv_threads", 0)
        self.declare_parameter("image_queue_depth", 10)
        self.declare_parameter("sync_queue_depth", 10)
        self.declare_parameter("output_scale", 1.0)
        self.declare_parameter("publish_color_preview", False)
        self.declare_parameter("publish_aux_without_subscribers", False)
        self.declare_parameter("mjpeg_decode_divisor", 1)
        self.declare_parameter("num_disparities", 96)
        self.declare_parameter("required_min_depth_m", 0.0)
        experimentDefaults = {
            "fixed_maps": False,
            "reuse_buffers": False,
            "postprocess": "numpy",
            "matcher": "sgbm",
            "experimental": False,
            "native_library": "",
            "elas_library": "",
            "rknn_model": "",
            "rknn_manifest": "",
            "matching_size": [320, 240],
            "sgbm_block_size": 5,
        }
        for name, default in experimentDefaults.items():
            self.declare_parameter(name, default)
        self.publish_aux_without_subscribers = bool(
            self.get_parameter("publish_aux_without_subscribers").value
        )
        self.mjpeg_decode_divisor = int(self.get_parameter("mjpeg_decode_divisor").value)
        if self.mjpeg_decode_divisor not in (1, 2, 4):
            raise ValueError("mjpeg_decode_divisor must be 1, 2, or 4")

        calib = self.get_parameter("calibration_file").value
        if not calib:
            raise RuntimeError("必须显式提供 calibration_file（缺标定不允许静默使用占位值）")
        self.processor = makeProcessor(
            calib,
            self.get_parameter("num_disparities").value,
            self.get_parameter("required_min_depth_m").value,
            **{k: self.get_parameter(k).value for k in experimentDefaults},
        )
        self.frame_id = self.get_parameter("frame_id").value
        self.declare_parameter("vio_prior_topic", "")
        self.priorBuffer = None
        priorTopic = self.get_parameter("vio_prior_topic").value
        if priorTopic:
            if not self.processor.config.experimental or self.processor.config.matcher not in (
                "census",
                "coarse_census",
            ):
                raise ValueError("vio_prior_topic requires experimental Census")
            from stereo_depth.experiments import PosePriorBuffer

            self.priorBuffer = PosePriorBuffer()
            self.priorSubscription = self.create_subscription(
                Odometry, priorTopic, self._onPriorPose, 10
            )
        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        self.publish_xyz = bool(self.get_parameter("publish_xyz").value)
        self.publishCompact = bool(self.get_parameter("publish_compact_xyz").value)
        self.output_scale = float(self.get_parameter("output_scale").value)
        if not 0.5 <= self.output_scale <= 1.0:
            raise ValueError("output_scale 必须处于 0.5 到 1.0")

        info = rectifiedCameraInfo(self.processor, self.frame_id)
        self.camera_info = scaledCameraInfo(info, self.output_scale)
        self.get_logger().info(
            "深度内参（与标定 P1 同源）："
            f"fx={self.camera_info['fx']:.4f} fy={self.camera_info['fy']:.4f} "
            f"cx={self.camera_info['cx']:.4f} cy={self.camera_info['cy']:.4f} "
            f"baseline={info['baseline_m']:.6f} m "
            f"size={self.camera_info['width']}x{self.camera_info['height']}"
        )

        self.get_logger().info(
            f"视差范围={self.processor.num_disparities}，理论最近深度>"
            f"{self.processor.nearestDepthM:.4f} m（不包含距离精度验收）"
        )
        cvThreads = int(self.get_parameter("opencv_threads").value)
        if cvThreads < 0:
            raise ValueError("opencv_threads must be nonnegative")
        if cvThreads:
            cv2.setNumThreads(cvThreads)
        imageDepth = int(self.get_parameter("image_queue_depth").value)
        syncDepth = int(self.get_parameter("sync_queue_depth").value)
        if imageDepth < 1 or syncDepth < 1:
            raise ValueError("image_queue_depth/sync_queue_depth 必须为正整数")
        imageQos = QoSProfile(
            depth=imageDepth,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.bridge = CvBridge()
        self.pub_depth = self.create_publisher(
            Image, self.get_parameter("depth_topic").value, imageQos
        )
        self.publish_color_preview = bool(self.get_parameter("publish_color_preview").value)
        self.pub_preview = self.create_publisher(Image, DEFAULTS.depth_preview_topic, imageQos)
        self.pub_xyz = self.create_publisher(
            PointCloud2, self.get_parameter("xyz_topic").value, imageQos
        )
        self.pub_xyz_valid = self.create_publisher(
            PointCloud2, self.get_parameter("xyz_valid_topic").value, imageQos
        )
        self.pub_info = self.create_publisher(
            CameraInfo, self.get_parameter("camera_info_topic").value, imageQos
        )
        self.publish_depth_compat = bool(self.get_parameter("publish_depth_compat").value)
        self.pub_depth_compat = self.create_publisher(
            Image, self.get_parameter("depth_compat_topic").value, imageQos
        )

        self.process_latest_only = bool(self.get_parameter("process_latest_only").value)
        rate = float(self.get_parameter("max_processing_rate_hz").value)
        if not np.isfinite(rate) or rate < 0:
            raise ValueError("max_processing_rate_hz must be finite and nonnegative")
        if rate and not self.process_latest_only:
            raise ValueError("max_processing_rate_hz requires process_latest_only")
        self.processingPeriod = 1.0 / rate if rate else 0.0
        self.worker = None
        transport = str(self.get_parameter("input_transport").value)
        if localInput:
            if not self.process_latest_only:
                raise ValueError("共享解码需要 process_latest_only=true")
            # 输入由采集节点直接交付，不建立图像 ROS 订阅。
        elif transport == "raw":
            self.subLeft = message_filters.Subscriber(
                self, Image, self.get_parameter("left_topic").value, qos_profile=imageQos
            )
            self.subRight = message_filters.Subscriber(
                self, Image, self.get_parameter("right_topic").value, qos_profile=imageQos
            )
            # 左右目来自同一拼接帧，stamp 必须完全相同；近似配对会混入不同曝光的两目。
            self.sync = message_filters.TimeSynchronizer(
                [self.subLeft, self.subRight], queue_size=syncDepth
            )
            self.sync.registerCallback(
                self.queuePair if self.process_latest_only else self.onPair
            )
        elif transport == "mjpeg":
            self.subPacket = self.create_subscription(
                CompressedImage,
                self.get_parameter("mjpeg_topic").value,
                self.queuePacket if self.process_latest_only else self.onPacket,
                imageQos,
            )
        else:
            raise ValueError("input_transport must be raw or mjpeg")

        self.frames = 0
        self.stats = {"valid_ratio_sum": 0.0, "compute_ms_sum": 0.0, "over_range": 0}
        self.reportTimer = self.create_timer(10.0, self.report)
        if self.process_latest_only:
            self.worker = LatestFrameWorker(
                self._processInput,
                processingPeriod=self.processingPeriod,
                fatalErrors=True,
                name="depth_worker",
            )

    @property
    def droppedProcessing(self):
        return self.worker.snapshot()["replaced"] if self.worker else 0

    def _submitInput(self, item):
        self.worker.raiseIfFailed("depth worker failed")
        self.worker.submit(item)

    def queuePair(self, leftMsg: Image, rightMsg: Image) -> None:
        self._submitInput((leftMsg, rightMsg))

    def _processInput(self, pair):
        if len(pair) == 3:
            self.onDecodedPair(*pair)
        elif pair[1] is None:
            self.onPacket(pair[0])
        else:
            self.onPair(*pair)

    def shutdown(self) -> None:
        if self.worker is not None and not self.worker.close(timeout=2.0):
            raise RuntimeError("depth worker did not stop within 2 s")
        self.processor.close()

    def queueDecodedPair(self, left, right, stamp) -> None:
        # 只传引用；采集端交付后不得再改数组。
        self._submitInput((left, right, stamp))

    def _needsXyz(self) -> bool:
        return (self.publish_xyz and self._wantAux(self.pub_xyz)) or (
            self.publishCompact and self._wantAux(self.pub_xyz_valid)
        )

    def _onPriorPose(self, msg):
        # 只接受已转换到本深度校正光学系的位姿；不把 IMU 位姿当作相机位姿。
        if msg.child_frame_id != self.frame_id:
            self.get_logger().warn(
                "VIO prior child_frame_id must equal depth rectified optical frame"
            )
            return
        covariance = np.asarray(msg.pose.covariance).reshape(6, 6)
        if (
            not np.isfinite(covariance).all()
            or np.any(np.diag(covariance) < 0)
            or np.max(np.diag(covariance)) > 0.1
        ):
            return
        from boom_birds_control.frames import makeTransform, quatToRot

        pose = msg.pose.pose
        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if not np.isfinite(quaternion).all() or not np.isclose(
            np.linalg.norm(quaternion), 1, atol=1e-3
        ):
            return
        transform = makeTransform(
            quatToRot(quaternion), [pose.position.x, pose.position.y, pose.position.z]
        )
        try:
            self.priorBuffer.push(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                transform,
                msg.header.frame_id,
            )
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _preparePrior(self, stamp):
        if self.priorBuffer is not None:
            seconds = stamp.sec + stamp.nanosec * 1e-9
            prior = self.priorBuffer.prepare(seconds, self.processor.p1, self.processor.q)
            self.processor.setVioPrior(prior, seconds)

    def onDecodedPair(self, left, right, stamp) -> None:
        self._preparePrior(stamp)
        result = processPair(self.processor, left, right, with_xyz=self._needsXyz())
        self._publishResult(result, stamp)

    def queuePacket(self, msg: CompressedImage) -> None:
        self.queuePair(msg, None)

    def onPacket(self, msg: CompressedImage) -> None:
        if "jpeg" not in msg.format.lower():
            raise ValueError("compressed stereo input must be JPEG")
        if not msg.data:
            raise ValueError("invalid stitched JPEG")
        decodeMode = {
            1: cv2.IMREAD_GRAYSCALE,
            2: cv2.IMREAD_REDUCED_GRAYSCALE_2,
            4: cv2.IMREAD_REDUCED_GRAYSCALE_4,
        }[self.mjpeg_decode_divisor]
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), decodeMode)
        if image is None or image.shape[1] % 2:
            raise ValueError("invalid stitched JPEG")
        orientation = SENSOR
        for part in msg.format.split(";")[1:]:
            if part.strip().startswith("boombirds_orientation="):
                orientation = part.strip().split("=", 1)[1]
        image = orientStitched(image, orientation, getattr(self.processor, "imageOrientation", SENSOR))
        self._preparePrior(msg.header.stamp)
        result = processStitched(self.processor, image, with_xyz=self._needsXyz())
        self._publishResult(result, msg.header.stamp)

    def onPair(self, leftMsg: Image, rightMsg: Image) -> None:
        try:
            left = self.bridge.imgmsg_to_cv2(leftMsg, desired_encoding="mono8")
            right = self.bridge.imgmsg_to_cv2(rightMsg, desired_encoding="mono8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"图像解码失败：{exc}")
            return
        if left.shape[0] != right.shape[0]:
            self.get_logger().error("左右图高度不一致，丢弃该帧")
            return
        stitched = np.hstack([left, right])
        self._preparePrior(leftMsg.header.stamp)
        try:
            result = processStitched(self.processor, stitched, with_xyz=self._needsXyz())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"深度计算失败：{exc}")
            return

        self._publishResult(result, leftMsg.header.stamp)

    def _wantAux(self, publisher) -> bool:
        return self.publish_aux_without_subscribers or publisher.get_subscription_count() > 0

    def _publishResult(self, result, stamp) -> None:
        if self.priorBuffer is not None:
            self.priorBuffer.record(result.depth)
        # 公共契约（任务要求）：主深度话题 32FC1、米制、**无效值保持 NaN**。
        # 另发一个 0 值兼容话题，供只接受 uint16 毫米流的消费者使用。
        depth = result.depth
        xyz = result.xyz
        valid = result.valid
        if self.output_scale != 1.0:
            size = (self.camera_info["width"], self.camera_info["height"])
            depth = cv2.resize(depth, size, interpolation=cv2.INTER_NEAREST)
            if xyz is not None:
                xyz = cv2.resize(xyz, size, interpolation=cv2.INTER_NEAREST)
            valid = cv2.resize(
                valid.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST
            ).astype(bool)
        depthPub, invalid, overRange = annotateValidity(
            depth, valid, self.max_depth, self.min_depth
        )
        # min/max 量程筛查用于毫米兼容流和诊断；主深度保留算法匹配结果及 NaN。
        depthNan = np.asarray(depth, dtype=np.float32).copy()  # 无效位置已是 NaN
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id
        self.pub_depth.publish(
            self.bridge.cv2_to_imgmsg(depthNan, encoding="32FC1", header=header)
        )
        if self.publish_color_preview and self._wantAux(self.pub_preview):
            normalized = np.uint8(
                np.clip(
                    (np.nan_to_num(depthNan, nan=self.min_depth) - self.min_depth)
                    / (self.max_depth - self.min_depth),
                    0,
                    1,
                )
                * 255
            )
            preview = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
            preview[~valid] = 0
            self.pub_preview.publish(
                self.bridge.cv2_to_imgmsg(preview, encoding="bgr8", header=header)
            )
        if self.publish_depth_compat and self._wantAux(self.pub_depth_compat):
            # 兼容话题必须名副其实：16UC1、毫米、uint16、无效值 = 整数 0。
            # 主话题保持 32FC1、米、NaN（公共契约）。
            depthMm = np.where(np.isfinite(depthPub), depthPub * 1000.0, 0.0)
            depthMm = np.clip(np.rint(depthMm), 0.0, 65535.0).astype(np.uint16)
            self.pub_depth_compat.publish(
                self.bridge.cv2_to_imgmsg(depthMm, encoding="16UC1", header=header)
            )

        if xyz is not None and self.publish_xyz and self._wantAux(self.pub_xyz):
            msg = cloud2XyzHw(xyz, valid, header)
            self.pub_xyz.publish(msg)
        if xyz is not None and self.publishCompact and self._wantAux(self.pub_xyz_valid):
            pts, dense = xyzToCompact(xyz, valid)
            self.pub_xyz_valid.publish(cloud2FromStructured(pts, header, dense))

        infoMsg = CameraInfo()
        infoMsg.header = header
        infoMsg.width = self.camera_info["width"]
        infoMsg.height = self.camera_info["height"]
        infoMsg.k = [
            self.camera_info["fx"],
            0.0,
            self.camera_info["cx"],
            0.0,
            self.camera_info["fy"],
            self.camera_info["cy"],
            0.0,
            0.0,
            1.0,
        ]
        infoMsg.p = [
            self.camera_info["fx"],
            0.0,
            self.camera_info["cx"],
            0.0,
            0.0,
            self.camera_info["fy"],
            self.camera_info["cy"],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        self.pub_info.publish(infoMsg)

        self.frames += 1
        self.stats["valid_ratio_sum"] += float(valid.mean())
        self.stats["compute_ms_sum"] += float(result.timings["compute_ms"])
        self.stats["over_range"] += int(overRange.sum())
        nanRatio = float(np.isnan(depthNan).mean())
        self.stats["nan_ratio_sum"] = self.stats.get("nan_ratio_sum", 0.0) + nanRatio

    def report(self) -> None:
        if self.worker is not None:
            self.worker.raiseIfFailed("depth worker failed")
        if self.frames == 0:
            self.get_logger().warn("尚无成功处理的深度帧")
            return
        self.get_logger().info(
            f"深度统计：frames={self.frames} 平均有效率={self.stats['valid_ratio_sum']/self.frames:.3f} "
            f"平均 compute={self.stats['compute_ms_sum']/self.frames:.1f} ms "
            f"超量程像素累计={self.stats['over_range']} "
            f"dropped_processing={self.droppedProcessing} "
            f"主话题 NaN 比例={self.stats.get('nan_ratio_sum', 0.0)/self.frames:.3f}"
        )


def main(argv=None) -> None:
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    oldHandlers = {
        sig: signal.signal(sig, signal.default_int_handler)
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    try:
        node = DepthNode()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"[boom_birds_sensing] depth_node 启动失败：{exc}")
        raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        # 幂等关闭：外部已经 shutdown（例如 Ctrl-C 或父进程信号）时不再重复调用，
        # 否则会在日志里留下 'rcl_shutdown already called' 的噪声，掩盖真正的失败原因。
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in oldHandlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
