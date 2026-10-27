"""ROS adapter for one file-defined Tag flight task."""
from dataclasses import replace
import json
import math
from pathlib import Path
import time
import yaml
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from boom_birds_control.platform_model import BoardObservation
from boom_birds_control.runtime_config import DEFAULTS
from .attitude_lifecycle import AttitudeMissionConfig
from .lifecycle import State
from .lifecycle_node import LifecycleNode
from .tag_flight_task import TagFlightTask, loadTask
from .tag_landing_fix import TagLandingFix


class TagFlightNode(LifecycleNode):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.declare_parameter("flight_task_file", "")
        self.declare_parameter("tag_config_file", "")
        self.declare_parameter("tag_test_only", False)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("start", False)
        self.declare_parameter("sih_pid",0)
        self.declare_parameter("handover_enabled",False)
        from boom_birds_control.sih_guard import verifySihProcess
        simulation = verifySihProcess(int(self.get_parameter("sih_pid").value))
        from ament_index_python.packages import get_package_share_directory
        self.declare_parameter("obstacle_config_file",str(Path(get_package_share_directory("boom_birds_bringup"))/"config/tag_obstacle_guard.yaml"))
        if self.control_mode != "companion_attitude":
            raise ValueError("tag_task_requires_attitude")
        # 交接必须显式开启；开启时平台速度接管必须同时启用，关闭时不得偷偷接管。
        self.handover = bool(self.get_parameter("handover_enabled").value)
        if self.handover:
            if not self.platform_gate.enabled:
                raise ValueError("tag_handover_requires_platform_landing")
            self.platform_gate.handover = True
            self.platform_gate.releaseRequired = True
        elif self.platform_gate.enabled:
            raise ValueError("platform_landing_requires_handover")
        testOnly = self.get_parameter("tag_test_only").value
        if testOnly and not self.get_parameter("dry_run").value and not simulation:
            raise ValueError("tag_test_only_requires_dry_run")
        task = loadTask(str(self.get_parameter("flight_task_file").value))
        profile = yaml.safe_load(Path(str(self.get_parameter("attitude_config_file").value)).read_text())
        mission = AttitudeMissionConfig(**profile.get("mission", {}))
        if task["landing_speed_m_s"] > mission.landing_speed:
            raise ValueError("tag_task_landing_speed_exceeds_profile")
        from boom_birds_control.attitude_control import AttitudeConfig
        limits = AttitudeConfig(**profile["controller"])
        if (max(task["max_speed_m_s"], task["max_landing_xy_m_s"]) > limits.max_velocity
                or task["max_accel_m_s2"] > limits.max_acceleration):
            raise ValueError("tag_task_exceeds_controller_limits")
        from .task_depth_guard import loadObstacleConfig
        obstacleConfig = loadObstacleConfig(str(self.get_parameter("obstacle_config_file").value),
                                           live=not self.get_parameter("dry_run").value and not simulation)
        if obstacleConfig.braking_accel_m_s2 > limits.max_acceleration:
            raise ValueError("obstacle_braking_exceeds_controller")
        self.fsm = TagFlightTask(self.config, mission, task,obstacleConfig,handover=self.handover)
        tagProfile = yaml.safe_load(Path(str(self.get_parameter("tag_config_file").value)).read_text())
        self.tracker = TagLandingFix(tagProfile, testOnly=testOnly)
        from .task_depth_bridge import TaskDepthBridge
        self.depthBridge = TaskDepthBridge(self,obstacleConfig)
        self.tagClockStamp = self.tagMonoStamp = None
        self.declare_parameter("require_landing_inputs",True)
        self.declare_parameter("downward_image_topic","/boom_birds/downward/image")
        self.requireLandingInputs=bool(self.get_parameter("require_landing_inputs").value)
        self.inputGraphAt=-math.inf
        self.inputPublishersReady=False
        self.inputReason="landing_inputs_missing"
        if not self.requireLandingInputs and not self.get_parameter("dry_run").value and not simulation:
            raise ValueError("live_tag_tasks_require_landing_inputs")
        self.declare_parameter("feature_tracking_enabled",True)
        self.useFeatureTracking=bool(self.get_parameter("feature_tracking_enabled").value)
        if not self.useFeatureTracking and not self.handover and not self.get_parameter("dry_run").value and not simulation:
            raise ValueError("live_tag_tasks_require_feature_tracking")
        from .feature_landing_fix import FeatureLandingFix
        self.featureFix=FeatureLandingFix(self.tracker)
        self.create_subscription(String,"/boom_birds/platform/feature_observation",self.onFeature,10)
        self.autoStart = bool(self.get_parameter("start").value)
        self.autoStartRequested = False
        self.create_subscription(Odometry, DEFAULTS.odom_body_topic, self.onPose, 50)
        self.create_subscription(Range, "/boom_birds/platform/range", self.onRange, qos_profile_sensor_data)
        self.create_subscription(String, "/boom_birds/platform/observation", self.onTag, 10)
        self.pubTask = self.create_publisher(String, "/boom_birds/tag_task/status", 10)
        self.releaseClient = self.create_client(__import__("std_srvs.srv", fromlist=["Trigger"]).Trigger,
                                                "/boom_birds/compute/release")

    def sampleTime(self, msg):
        return self.stamp(msg)

    def onPose(self, msg):
        if msg.header.frame_id != self.config.world_frame or msg.child_frame_id != DEFAULTS.body_frame:
            self.tracker.clear()
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        try:
            self.tracker.addPose(self.sampleTime(msg), (p.x,p.y,p.z), (q.x,q.y,q.z,q.w))
        except ValueError:
            self.tracker.clear()

    def onRange(self, msg):
        self.tracker.addRange(self.sampleTime(msg), msg.range, msg.min_range, msg.max_range, msg.header.frame_id)

    def onTag(self, msg):
        try:
            item = json.loads(msg.data)
            observation = BoardObservation(**item)
            if type(observation.stamp) not in (float, int):
                raise ValueError("tag_stamp")
            self.tracker.submit(observation)
        except (TypeError, ValueError, KeyError):
            self.tracker.pending = self.tracker.fix = None

    def onFeature(self,msg):
        from boom_birds_sensing.platform_feature_track import LocalFeatureObservation
        try:
            observation=LocalFeatureObservation(**json.loads(msg.data))
            if type(observation.stamp) not in (int,float) or not math.isfinite(observation.stamp) or type(observation.valid) is not bool:
                raise ValueError("feature_stamp_or_valid")
            self.featureFix.submit(observation)
        except (TypeError,ValueError,KeyError):
            self.featureFix.reset()
            self.featureFix.reason="feature_wire_invalid"

    def _mission(self, request, response):
        if request.action == request.START:
            # 交接任务必须在解锁前证明计算释放服务存在；否则交接后无法确认 VIO 退出。
            if (self.handover and not self.get_parameter("dry_run").value
                    and not self.releaseClient.service_is_ready()):
                response.accepted=False;response.session_id=self.fsm.session or ""
                response.state=self.fsm.state.value;response.reason="compute_release_service_missing"
                return response
            if self.requireLandingInputs and not self.landingInputs(time.monotonic()):
                response.accepted=False;response.session_id=self.fsm.session or ""
                response.state=self.fsm.state.value;response.reason=self.inputReason
                return response
            # The file owns the route; Mission.goal must not change it.
            request.goal.x = request.goal.y = request.goal.z = 0.
            if self.fsm.state in (State.IDLE, State.COMPLETE) and self.startFuture is None:
                self.tracker.clear()
        return super()._mission(request, response)

    def landingInputs(self,now):
        if not self.requireLandingInputs:
            self.inputReason=""
            return True
        featureRequired=self.useFeatureTracking and not self.handover
        if now-self.inputGraphAt>=.5:
            topics=[str(self.get_parameter("downward_image_topic").value),"/boom_birds/platform/range",
                    "/boom_birds/platform/observation"]
            if featureRequired:topics.append("/boom_birds/platform/feature_observation")
            self.inputPublishersReady=all(self.count_publishers(topic)==1 for topic in topics)
            self.inputGraphAt=now
        if not self.inputPublishersReady:
            self.inputReason="landing_input_publishers"
            return False
        ros=self.rosNow()
        image=self.tracker.pending
        feature=self.featureFix.pending
        imageFresh=(image is not None and math.isfinite(image.stamp)
            and 0<=ros-image.stamp<=self.fsm.task["tag_timeout_s"])
        rangeFresh=(bool(self.tracker.ranges) and 0<=ros-self.tracker.ranges[-1][0]<=self.tracker.profile["guidance"]["range_timeout_s"])
        featureFresh=(not featureRequired or (feature is not None
            and math.isfinite(feature.stamp) and 0<=ros-feature.stamp<=self.fsm.task["tag_timeout_s"]))
        self.inputReason="" if imageFresh and rangeFresh and featureFresh else "landing_sensor_link"
        return not self.inputReason

    def monotonicFix(self, fix, now):
        if fix is None:
            return None
        if fix.stamp != self.tagClockStamp:
            self.tagClockStamp = fix.stamp
            self.tagMonoStamp = now-(self.rosNow()-fix.stamp)
        return replace(fix,stamp=self.tagMonoStamp)

    def _tick(self):
        if not hasattr(self, "tracker"):
            return
        now = time.monotonic()
        self.fsm.obstacle = self.depthBridge.evaluate()
        self.fsm.landingInputsReady=self.landingInputs(now)
        status = self.control
        if (status is not None and status.control_pose_known
                and 0 <= now-self.controlReceived <= self.config.pose_timeout_s
                and 0 <= self.rosNow()-self.stamp(status) <= self.config.pose_timeout_s
                and 0 <= status.control_pose_age_s <= self.config.pose_timeout_s
                and math.isfinite(status.control_yaw_world_rad)):
            self.fsm.currentYaw = status.control_yaw_world_rad
        else:
            self.fsm.currentYaw = None
        try:
            fix = self.tracker.evaluate(self.rosNow())
        except (TypeError, ValueError, OverflowError):
            self.tracker.fix = None
            self.tracker.reason = "tag_wire_invalid"
            fix = None
        featureFix=None
        if self.useFeatureTracking:
            try:
                featureFix=self.featureFix.evaluate(self.rosNow())
            except (TypeError,ValueError,KeyError,OverflowError):
                self.featureFix.reset()
                self.featureFix.reason="feature_wire_invalid"
        if self.handover:
            # 平台速度交接只接受完整身份解码结果：局部特征 PnP 的米制解依赖 VIO，
            # VIO 退出后不能作为交接/下降依据，因此只保留诊断，不覆盖解码 fix。
            self.fsm.featureDiagnostic = None if featureFix is None else featureFix.source
            fix = fix
        else:
            fix = featureFix
        self.fsm.fix = self.monotonicFix(fix,now)
        if self.autoStart and not self.autoStartRequested and self.client.service_is_ready():
            from boom_birds_interfaces.srv import Mission
            request = Mission.Request(action=Mission.Request.START)
            response = self._mission(request, Mission.Response())
            self.autoStartRequested = response.accepted
        super()._tick()
        self.pubTask.publish(String(data=json.dumps(dict(task=self.fsm.task["name"],
            state=self.fsm.state.value, phase=self.fsm.phase, stage=self.fsm.stageIndex,
            reason=self.fsm.reason, tag_reason=self.tracker.reason,
            tag_input_reason=None if self.tracker.pending is None else self.tracker.pending.reason,
            route_reached=self.fsm.goal_reached and not self.fsm.obstacleMissionFailed,
            complete=(self.fsm.state == State.COMPLETE and not self.fsm.obstacleMissionFailed
                and not (self.handover and self.platform_gate.failed)
                and (not self.handover or (status is not None and status.landed_known
                     and status.landed_state == 1 and not status.armed))),
            obstacle_state=self.fsm.obstacle.state,obstacle_reason=self.fsm.obstacle.reason,
            nearest_obstacle_m=self.fsm.obstacle.nearest_m,
            landing_velocity_m_s=self.fsm.landingVelocity,
            landing_fix_source=None if self.fsm.fix is None else self.fsm.fix.source,
            landing_inputs_ready=self.fsm.landingInputsReady,
            landing_input_reason=self.inputReason,
            feature_reason=self.featureFix.reason,
            feature_pnp_candidates=self.featureFix.pnpCandidateCount,
            feature_pnp_reprojection_px=self.featureFix.pnpReprojection,
            feature_identity_stamp=None if self.fsm.fix is None else self.fsm.fix.identity_stamp,
            landing_alignment_age_s=None if self.fsm.alignSince is None else now-self.fsm.alignSince,
            current_yaw_rad=self.fsm.currentYaw,yaw_setpoint_rad=self.fsm.yawSetpoint,
            control_pose_age_s=None if status is None else status.control_pose_age_s,
            control_pose_known=False if status is None else status.control_pose_known,
            tag_height_m=None if self.fsm.fix is None else self.fsm.fix.height,
            landing_target_world_m=self.fsm.landingSetpoint,
            avoidance_state=self.fsm.avoidance.state,avoidance_purpose=self.fsm.avoidance.purpose,
            detour_used=self.fsm.avoidance.used,mission_aborted=self.fsm.obstacleMissionFailed or self.fsm.avoidance.aborted,
            returned_and_landed=bool(self.fsm.reachedHome and status is not None
                and 0 <= now-self.controlReceived <= self.config.pose_timeout_s
                and status.landed_known and status.landed_state == 1 and not status.armed),
            handover_enabled=bool(self.handover),
            handover=(self.platform_gate.report() if self.handover else None),
            capture=dict(self.fsm.capture),
            landed_disarmed=bool(status is not None and status.landed_known
                and status.landed_state == 1 and not status.armed),
            session=self.fsm.session))))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TagFlightNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
