"""生成 Task 2/3 的 OpenVINS 配置并检查 PX4 高度源切换。"""

import json
import math
import re

import yaml

import run_aircraft_motor_sitl as sim


def writeVioConfig(out, stereo, imuSensor, gravity):
    source = sim.WS / "src/open_vins/config/boombirds_synthetic"
    config = (source / "estimator_config.yaml").read_text()
    for key, value in {
        "gravity_mag": str(gravity),
        "try_zupt": "1",
        "zupt_only_at_beginning": "1",
        "init_dyn_use": "0",
        "zupt_chi2_multipler": "0",
        "zupt_max_disparity": "0.12",
        "num_pts": "170",
        "fast_threshold": "12",
        "num_opencv_threads": "1",
        "max_clones": "11",
        "max_slam": "40",
        "up_msckf_sigma_px": "1.5",
        "up_slam_sigma_px": "1.5",
    }.items():
        config, count = re.subn(rf"^{key}:.*$", f"{key}: {value}", config, flags=re.M)
        if count != 1:
            raise ValueError(f"missing OpenVINS setting {key}")
    target = out / "estimator_config.yaml"
    # The rectified pair still supplies disparity when the vehicle turns in place.
    target.write_text(config + "\nstereo_geometry_check: true\nfi_max_dist: 7.0\n"
                      "fi_max_baseline: 120.0\nfi_max_cond_number: 60000.0\n")

    raw = (source / "kalibr_imu_chain.yaml").read_text()
    imu = yaml.safe_load(raw.removeprefix("%YAML:1.0").lstrip())
    imuRate = float(imuSensor.findtext("update_rate"))
    accelNoise = max(float(imuSensor.findtext(f"imu/linear_acceleration/{axis}/noise/stddev"))
                     for axis in ("x", "y", "z")) / math.sqrt(imuRate)
    gyroNoise = max(float(imuSensor.findtext(f"imu/angular_velocity/{axis}/noise/stddev"))
                    for axis in ("x", "y", "z")) / math.sqrt(imuRate)
    imu["imu0"].update(rostopic="/boom_birds/sim/imu_flu", update_rate=imuRate,
                       accelerometer_noise_density=accelNoise, accelerometer_random_walk=.0002,
                       gyroscope_noise_density=gyroNoise, gyroscope_random_walk=.00001)
    cameras = {}
    for index, (side, (x, y, z)) in enumerate(zip(("left", "right"), sim.stereoPositions(stereo))):
        cameras[f"cam{index}"] = dict(
            T_imu_cam=[[0., 0., 1., x], [-1., 0., 0., y], [0., -1., 0., z], [0., 0., 0., 1.]],
            camera_model="pinhole", distortion_model="radtan", distortion_coeffs=[0.]*4,
            intrinsics=list(stereo["intrinsics"]), resolution=list(stereo["resolution"]),
            rostopic=f"/boom_birds/stereo/{side}_raw", timeshift_cam_imu=0.,
        )
    for basename, data in (("kalibr_imu_chain", imu), ("kalibr_imucam_chain", cameras)):
        lines = ["%YAML:1.0", "---"]
        # OpenCV FileStorage expects row sequences for matrix fields.
        class ChainDumper(yaml.SafeDumper):
            pass

        ChainDumper.add_representer(list, lambda dumper, value:
                                   dumper.represent_sequence("tag:yaml.org,2002:seq", value, flow_style=True))
        lines.append(yaml.dump(data, Dumper=ChainDumper, sort_keys=False, width=1000))
        (out / f"{basename}.yaml").write_text("\n".join(lines))
    return target


class VioHeightAid:
    """Relay only fresh, visually updated OpenVINS odometry, anchored before motion."""

    def __init__(self, px4, node):
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String

        self.px4 = px4
        self.last = None
        self.lastSent = -math.inf
        self.offsetDown = None
        self.positionAnchor = None
        self.inputStamp = None
        self.positionVariance = None
        self.status = {}
        self.subscription = node.create_subscription(Odometry, "/ov_msckf/odomimu", self.receive, 1)
        self.statusSubscription = node.create_subscription(String, "/ov_msckf/status", self.receiveStatus, 1)

    def recordTrajectory(self, node):
        from nav_msgs.msg import Odometry

        self.trajectoryFile = (self.px4.runDirectory / "vio-trajectory.jsonl").open("x", buffering=1)
        self.lastTrajectoryStamp = -math.inf
        self.stateSubscription = node.create_subscription(
            Odometry, "/ov_msckf/state_odomimu", self.receiveTrajectory, 1)

    def receiveTrajectory(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if stamp-self.lastTrajectoryStamp < .06 or not all(math.isfinite(value) for value in values):
            return
        self.lastTrajectoryStamp = stamp
        print(json.dumps(dict(stamp=stamp, position=values[:3], quaternion=values[3:],
                             last_visual_stamp=self.status.get("last_visual_update_stamp_s"))),
              file=self.trajectoryFile)

    def receiveStatus(self, msg):
        self.status = json.loads(msg.data)

    def receive(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.e-9
        positionVariance = msg.pose.covariance[14]
        self.inputStamp = stamp
        self.positionVariance = positionVariance
        if (msg.header.frame_id != "global" or msg.child_frame_id != "imu"
                or not all(math.isfinite(n) for n in (stamp, p.z, v.x, v.y, v.z, positionVariance))
                or not 0. <= positionVariance <= .08**2):
            return
        self.last = stamp, -p.z, (v.x, -v.y, -v.z), positionVariance
        q = msg.pose.pose.orientation
        if all(math.isfinite(value) for value in (p.x, p.y, q.x, q.y, q.z, q.w,
                                                  msg.pose.covariance[0], msg.pose.covariance[7])):
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            quaternion = q.x, q.y, q.z, q.w
            self.horizontal = stamp, p.x, p.y, yaw, quaternion, max(
                msg.pose.covariance[0], msg.pose.covariance[7], .09**2)


    def initialized(self, now):
        stamp = self.status.get("input_stamp_s")
        return (self.status.get("state_initialized") is True and isinstance(stamp, (int, float))
                and -.004 <= now-stamp <= .60)

    def fresh(self, now):
        return self.last is not None and -.004 <= now-self.last[0] <= .25

    def diagnostics(self, now):
        return dict(age_s=None if self.last is None else now-self.last[0],
                    input_age_s=None if self.inputStamp is None else now-self.inputStamp,
                    variance_m2=self.positionVariance, tracking=self.status)

    def anchor(self, now, down):
        if not self.fresh(now):
            return False
        self.offsetDown = down - self.last[1]
        return True

    def anchorHorizontal(self, now, position, yawNed):
        if not self.fresh(now) or not hasattr(self, "horizontal") or self.horizontal[0] != self.last[0]:
            return False
        _, x, y, yaw, _, _ = self.horizontal
        angle = math.pi/2-yawNed-yaw
        self.positionAnchor = x, y, angle, position[0], position[1]
        return True

    def tick(self, now):
        if self.offsetDown is None or not self.fresh(now):
            return
        stamp, down, velocity, variance = self.last
        if stamp - self.lastSent < .039:
            return
        self.lastSent = stamp
        mav = self.px4.mav
        unknown = math.nan
        poseCovariance = [0.]*21
        north = east = 0.
        poseFrame, orientation = mav.MAV_FRAME_LOCAL_NED, [unknown]*4
        poseCovariance[0] = poseCovariance[6] = 1.
        poseCovariance[15] = poseCovariance[18] = poseCovariance[20] = 1.
        if self.positionAnchor is not None:
            sampleTime, x, y, _, quaternion, xyVariance = self.horizontal
            if sampleTime != stamp:
                return
            from boom_birds_control.attitude_control import NED_TO_ENU, rotation, yawRotation
            from boom_birds_control.frames import rotToQuat
            import numpy as np

            x0, y0, angle, n0, e0 = self.positionAnchor
            transform = NED_TO_ENU @ yawRotation(angle)
            north, east = (transform @ [x-x0, y-y0, 0.])[:2] + [n0, e0]
            qx, qy, qz, qw = rotToQuat(transform @ rotation(quaternion) @ np.diag([1., -1., -1.]))
            orientation = [qw, qx, qy, qz]
            poseFrame = mav.MAV_FRAME_LOCAL_FRD
            poseCovariance[0] = poseCovariance[6] = xyVariance
            poseCovariance[15] = poseCovariance[18] = poseCovariance[20] = .03**2
        poseCovariance[11] = max(variance, .015**2)
        twistCovariance = [0.]*21
        for diagonal in (0, 6, 11, 15, 18, 20):
            twistCovariance[diagonal] = .05**2
        # Task 3 supplies local FRD pose for horizontal aiding without compass yaw alignment.
        self.px4.wire.mav.odometry_send(
            round(stamp*1.e6), poseFrame, mav.MAV_FRAME_BODY_FRD,
            north, east, down+self.offsetDown, orientation, *velocity, unknown, unknown, unknown,
            poseCovariance, twistCovariance, 0, mav.MAV_ESTIMATOR_TYPE_VIO, 100,
        )
