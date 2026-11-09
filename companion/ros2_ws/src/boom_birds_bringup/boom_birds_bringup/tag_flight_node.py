"""ROS adapter for one file-defined Tag flight task."""
from boom_birds_control.runtime_config import trialFreshness
from dataclasses import replace
import json
import math
from pathlib import Path
import time
import yaml
import rclpy
import signal
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from boom_birds_control.platform_model import BoardObservation
from boom_birds_control.runtime_config import DEFAULTS
from .lifecycle import State
from .lifecycle_node import LifecycleNode
from .tag_flight_task import taskFromProfile, loadTask
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
        if self.control_mode not in ("companion_attitude", "px4_velocity"):
            raise ValueError("tag_task_requires_attitude")
        # 交接必须显式开启；开启时平台速度接管必须同时启用，关闭时不得偷偷接管。
        self.handover = bool(self.get_parameter("handover_enabled").value)
        if self.handover and self.control_mode == "px4_velocity":
            raise ValueError("velocity_task_has_no_output_handover")
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
        from .task_depth_guard import loadObstacleConfig
        obstacleConfig = loadObstacleConfig(str(self.get_parameter("obstacle_config_file").value),
                                           live=not self.get_parameter("dry_run").value and not simulation)
        from boom_birds_control.flight_trial import flightTrialApproved, validateTrialTask
        validateTrialTask([profile, yaml.safe_load(Path(str(self.get_parameter("tag_config_file").value)).read_text()),
                           yaml.safe_load(Path(str(self.get_parameter("obstacle_config_file").value)).read_text())], task)
        self.flightTrial = self.control_mode == "px4_velocity" and flightTrialApproved(profile)
        self.fsm = taskFromProfile(self.config, profile, task, obstacleConfig, handover=self.handover)
        self.declare_parameter("px4_local_feedback", False)
        self.px4LocalFeedback = bool(self.get_parameter("px4_local_feedback").value)
        if self.px4LocalFeedback and (not self.flightTrial or task["name"] != "tag_takeoff_land" or self.handover):
            raise ValueError("px4_local_feedback_requires_task1_trial")
        self.declare_parameter("airborne_landing", False)
        self.fsm.airborneLanding = bool(self.get_parameter("airborne_landing").value)
        if self.fsm.airborneLanding and not self.px4LocalFeedback:
            raise ValueError("airborne_landing_requires_px4_local_feedback")
        self.fsm.useForwardDepth = not self.px4LocalFeedback
        self.fsm.relaxTrialDepthCoverage = self.flightTrial and task["name"] == "tag_takeoff_land" and not self.px4LocalFeedback
        tagProfile = yaml.safe_load(Path(str(self.get_parameter("tag_config_file").value)).read_text())
        self.tracker = TagLandingFix(tagProfile, testOnly=testOnly)
        from .task_depth_bridge import TaskDepthBridge
        self.depthBridge = (None if self.px4LocalFeedback else
                            TaskDepthBridge(self, obstacleConfig, fixedFloor=not self.fsm.relaxTrialDepthCoverage))
        self.tagClockStamp = self.tagMonoStamp = None
        self._landingSamplesAfter = None
        self.declare_parameter("require_landing_inputs",True)
        self.declare_parameter("downward_image_topic","/boom_birds/downward/image")
        self.requireLandingInputs=bool(self.get_parameter("require_landing_inputs").value)
        self.lastDownwardImageStamp=None
        self.create_subscription(Image,str(self.get_parameter("downward_image_topic").value),
                                 self.onDownwardImage,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.inputGraphAt=-math.inf
        self.inputPublishersReady=False
        self.inputReason="landing_inputs_missing"
        if not self.requireLandingInputs and not self.get_parameter("dry_run").value and not simulation:
            raise ValueError("live_tag_tasks_require_landing_inputs")
        self.declare_parameter("feature_tracking_enabled",True)
        self.useFeatureTracking=bool(self.get_parameter("feature_tracking_enabled").value)
        if (not self.useFeatureTracking and not self.handover and self.control_mode != "px4_velocity"
                and not self.get_parameter("dry_run").value and not simulation):
            raise ValueError("live_tag_tasks_require_feature_tracking")
        from .feature_landing_fix import FeatureLandingFix
        self.featureFix=FeatureLandingFix(self.tracker)
        self.create_subscription(String,"/boom_birds/platform/feature_observation",self.onFeature,10)
        self.autoStart = bool(self.get_parameter("start").value)
        self.autoStartRequested = False
        poseTopic = "/boom_birds/px4/local_pose" if self.px4LocalFeedback else DEFAULTS.odom_body_topic
        self.create_subscription(Odometry, poseTopic, self.onPose, 50)
        self.create_subscription(Range, "/boom_birds/platform/range", self.onRange, qos_profile_sensor_data)
        self.create_subscription(String, "/boom_birds/platform/observation", self.onTag, 10)
        self.pubTask = self.create_publisher(String, "/boom_birds/tag_task/status", 10)
        from boom_birds_control.diagnostic_throttle import DiagnosticThrottle
        self._taskDiagnosticThrottle=DiagnosticThrottle(5.)
        self.releaseClient = self.create_client(__import__("std_srvs.srv", fromlist=["Trigger"]).Trigger,
                                                "/boom_birds/compute/release")

    def _controlStatus(self, msg):
        previous = self.control
        if self.fsm.airborneLanding and previous is not None:
            epoch = (msg.restart_epoch, msg.frame_reset_epoch)
            previousEpoch = (previous.restart_epoch, previous.frame_reset_epoch)
            stamp = self.stamp(msg)
            age = self.rosNow() - stamp
            if (epoch < previousEpoch
                    or not -self.config.command_future_tolerance_s <= age <= self.config.command_timeout_s):
                return
            if epoch != previousEpoch:
                self.tracker.clear()
                self.featureFix.reset()
                self.tagClockStamp = self.tagMonoStamp = self.lastDownwardImageStamp = None
                self.fsm.fix = None
                # 状态与观测走不同话题，只用代次回读之后采集的样本。
                self._landingSamplesAfter = stamp
                # ROS 对时可能回拨，旧代次的时间戳不能拒绝新状态。
                self.control = None
        super()._controlStatus(msg)

    def sampleTime(self, msg):
        return self.stamp(msg)

    def _acceptLandingSample(self, stamp):
        return self._landingSamplesAfter is None or self._landingSamplesAfter < stamp <= self.rosNow()

    def onPose(self, msg):
        stamp = self.sampleTime(msg)
        if not self._acceptLandingSample(stamp):
            return
        if msg.header.frame_id != self.config.world_frame or msg.child_frame_id != DEFAULTS.body_frame:
            self.tracker.clear()
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        try:
            self.tracker.addPose(stamp, (p.x,p.y,p.z), (q.x,q.y,q.z,q.w))
        except ValueError:
            self.tracker.clear()

    def onDownwardImage(self, msg):
        expected=tuple(reversed(self.tracker.profile["camera"]["image_size"]))
        stamp=self.sampleTime(msg)
        if not self._acceptLandingSample(stamp):
            return
        self.lastDownwardImageStamp=(stamp if math.isfinite(stamp) and stamp>0
            and (msg.height,msg.width)==expected and msg.encoding=="mono8"
            and msg.step>=msg.width and len(msg.data)>=msg.step*msg.height else None)

    def onRange(self, msg):
        stamp = self.sampleTime(msg)
        if self._acceptLandingSample(stamp):
            self.tracker.addRange(stamp, msg.range, msg.min_range, msg.max_range, msg.header.frame_id)

    def onTag(self, msg):
        try:
            item = json.loads(msg.data)
            observation = BoardObservation(**item)
            if type(observation.stamp) not in (float, int):
                raise ValueError("tag_stamp")
            if self._acceptLandingSample(observation.stamp):
                self.tracker.submit(observation)
        except (TypeError, ValueError, KeyError):
            self.tracker.pending = self.tracker.fix = None

    def onFeature(self,msg):
        from boom_birds_sensing.platform_feature_track import LocalFeatureObservation
        try:
            observation=LocalFeatureObservation(**json.loads(msg.data))
            if type(observation.stamp) not in (int,float) or not math.isfinite(observation.stamp) or type(observation.valid) is not bool:
                raise ValueError("feature_stamp_or_valid")
            if (self._acceptLandingSample(observation.stamp)
                    and (not observation.valid or self._acceptLandingSample(observation.identity_stamp))):
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
            if (self.fsm.state in (State.IDLE, State.COMPLETE) and self.startFuture is None
                    and not self.fsm.airborneLanding):
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
        imageStamp=self.lastDownwardImageStamp
        feature=self.featureFix.pending
        # 相机心跳不生成降落目标；Tag/特征/测距仍各自检查原观测年龄。
        imageAgeLimit = trialFreshness(1.0 if getattr(self, "flightTrial", False) else self.fsm.task["tag_timeout_s"])
        imageFresh=(imageStamp is not None and math.isfinite(imageStamp)
            and 0<=ros-imageStamp<=imageAgeLimit)
        rangeFresh=(bool(self.tracker.ranges) and 0<=ros-self.tracker.ranges[-1][0]<=trialFreshness(self.tracker.profile["guidance"]["range_timeout_s"]))
        featureFresh=(getattr(self, "px4LocalFeedback", False) or not featureRequired or (feature is not None
            and math.isfinite(feature.stamp) and 0<=ros-feature.stamp<=trialFreshness(self.fsm.task["tag_timeout_s"])))
        # A grounded rangefinder may be below its minimum range; raw images prove the camera link.
        groundRangeOptional = False
        if getattr(self, "flightTrial", False):
            if self.fsm.state in (State.IDLE, State.PRECHECK) and not self.fsm.airborneLanding:
                groundRangeOptional = True
            elif self.fsm.state in (State.HOLD_READY, State.OFFBOARD_PENDING, State.TAKEOFF):
                status = self.control
                groundRangeOptional = (status is not None and status.control_pose_known
                    and 0 <= now-self.controlReceived <= self.config.pose_timeout_s
                    and 0 <= status.control_pose_age_s <= self.config.pose_timeout_s
                    and self.fsm.groundZ is not None
                    and status.control_position_world.z-self.fsm.groundZ <= self.fsm.task["final_land_height_m"])
        self.inputDetail = dict(image_fresh=imageFresh, range_fresh=rangeFresh,
            range_optional_near_ground=groundRangeOptional, feature_fresh=featureFresh,
            image_age_s=None if imageStamp is None else ros-imageStamp,
            image_source="downward_image", image_heartbeat_timeout_s=imageAgeLimit,
            range_age_s=None if not self.tracker.ranges else ros-self.tracker.ranges[-1][0])
        self.inputReason="" if imageFresh and (rangeFresh or groundRangeOptional) and featureFresh else "landing_sensor_link"
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
        if self.depthBridge is not None:
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
        elif self.useFeatureTracking and (not getattr(self, "px4LocalFeedback", False) or fix is None):
            fix = featureFix
        self.fsm.fix = self.monotonicFix(fix,now)
        if self.autoStart and not self.autoStartRequested and self.client.service_is_ready():
            from boom_birds_interfaces.srv import Mission
            request = Mission.Request(action=Mission.Request.START)
            response = self._mission(request, Mission.Response())
            self.autoStartRequested = response.accepted
        super()._tick()
        obstacle = self.fsm.obstacleReport(time.monotonic())
        airborneTagReady = False
        if (self.fsm.airborneLanding and self.fsm.currentYaw is not None
                and status.connected and status.sensors_ready and status.alignment_valid):
            p, v = status.control_position_world, status.control_velocity_world
            airborneTagReady = self.fsm.airborneTagReady(now, (p.x,p.y,p.z), (v.x,v.y,v.z))
        key=(self.fsm.state,self.fsm.phase,self.fsm.stageIndex,self.fsm.reason,
             self.fsm.landingInputsReady,obstacle.state,obstacle.reason,self.tracker.reason,airborneTagReady)
        if not self._taskDiagnosticThrottle.due(now,key):return
        self.pubTask.publish(String(data=json.dumps(dict(task=self.fsm.task["name"],
            state=self.fsm.state.value, phase=self.fsm.phase, stage=self.fsm.stageIndex,
            reason=self.fsm.reason, tag_reason=self.tracker.reason,
            tag_input_reason=None if self.tracker.pending is None else self.tracker.pending.reason,
            route_reached=self.fsm.goal_reached and not self.fsm.obstacleMissionFailed,
            complete=(self.fsm.state == State.COMPLETE and not self.fsm.obstacleMissionFailed
                and not (self.handover and self.platform_gate.failed)
                and (not self.handover or (status is not None and status.landed_known
                     and status.landed_state == 1 and not status.armed))),
            obstacle_state=obstacle.state,obstacle_reason=obstacle.reason,
            nearest_obstacle_m=obstacle.nearest_m,
            obstacle_valid_coverage=obstacle.valid_coverage,
            landing_velocity_m_s=self.fsm.landingVelocity,
            landing_fix_source=None if self.fsm.fix is None else self.fsm.fix.source,
            landing_inputs_ready=self.fsm.landingInputsReady,
            landing_input_detail=getattr(self,"inputDetail",{}),
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
            airborne_tag_ready=airborneTagReady,
            airborne_control_engaged=(self.fsm.airborneLanding and self.fsm.flightStarted is not None),
            handover_enabled=bool(self.handover),
            handover=(self.platform_gate.report() if self.handover else None),
            capture=dict(self.fsm.capture), landing_capture_samples=getattr(self.fsm,"landingCaptureSamples",0),
            landed_disarmed=bool(status is not None and status.landed_known
                and status.landed_state == 1 and not status.armed),
            session=self.fsm.session))))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping=False
    def requestStop(signum,frame):
        nonlocal stopping
        stopping=True
    previousHandlers={number:signal.signal(number,requestStop)
                      for number in (signal.SIGINT,signal.SIGTERM)}
    executor=SingleThreadedExecutor()
    node = None
    try:
        node = TagFlightNode()
        executor.add_node(node)
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        for number,handler in previousHandlers.items():signal.signal(number,handler)
