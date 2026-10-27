"""Read-only, unqualified MAVLink EKF reference candidate.

ODOMETRY carries position, velocity and reset counter from one PX4 sample.
ESTIMATOR_STATUS supplies horizontal validity and an accuracy estimate. This
reader never requests streams, sends commands or authorizes control.
"""
import math
import struct
from .landing_ekf_reference import EkfReferenceSample
from .mavros_observation import decodeObservation


class MavrosEkfReference:
    def __init__(self, node, backend, *, max_age_s=.2, max_pair_skew_s=.15):
        from mavros_msgs.msg import Mavlink
        from rclpy.qos import qos_profile_sensor_data
        self.node, self.backend = node, backend
        self.maxAge, self.maxSkew = max_age_s, max_pair_skew_s
        self.samples, self.lastBoot = {}, {}
        self.epoch = None
        self.reason = "missing_ekf_messages"
        self.subscription = node.create_subscription(Mavlink,
            f"/uas{backend.target_system}/mavlink_source", self.onRaw, qos_profile_sensor_data)

    def onRaw(self, raw):
        b = self.backend
        mid = int(raw.msgid)
        if (not b._endpointVerified or raw.sysid != b.target_system
                or raw.compid != b.target_component or mid not in (230, 331)):
            return
        try:
            now = b._clock()
            rosNow = self.node.get_clock().now().nanoseconds*1e-9
            receipt = raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9
            if not math.isfinite(receipt) or not 0 <= rosNow-receipt <= self.maxAge:
                self.samples.pop(mid, None)
                return
            state = b.readVehicleState()
            epoch = (state.restart_epoch << 32) | state.frame_reset_epoch
            if self.epoch != epoch:
                self.samples.clear()
                self.lastBoot.clear()
                self.epoch = epoch
            if mid == 230:
                msg = decodeObservation(raw)
                if msg is None:
                    self.samples.pop(mid, None)
                    return
                boot = float(msg.time_usec)*1e-6
            else:
                if int(raw.framing_status) != 1:
                    self.samples.pop(mid, None)
                    return
                payload = b"".join(struct.pack("<Q", int(v)) for v in raw.payload64)
                length = int(raw.len)
                # estimator_type (extension byte 231) must explicitly identify PX4.
                if length < 232 or length > len(payload) or length > 255:
                    self.samples.pop(mid, None)
                    return
                payload = payload[:length]
                boot = struct.unpack_from("<Q", payload, 0)[0]*1e-6
                x, y, z = struct.unpack_from("<fff", payload, 8)
                vx, vy, vz = struct.unpack_from("<fff", payload, 36)
                covX = struct.unpack_from("<f", payload, 60)[0]
                covY = struct.unpack_from("<f", payload, 84)[0]
                covZ = struct.unpack_from("<f", payload, 104)[0]
                if (payload[228] != 1 or payload[229] != 1 or payload[231] != 8
                        or not all(math.isfinite(v) for v in (x, y, vx, vy, covX, covY))
                        or covX < 0 or covY < 0):
                    self.samples.pop(mid, None)
                    return
                msg = (x, y, vx, vy, covX, covY, payload[230], z, vz, covZ)
            if not math.isfinite(boot) or boot <= 0 or boot <= self.lastBoot.get(mid, -math.inf):
                return
            if b._sihReceiptTime:
                observed = now-(rosNow-receipt)
            else:
                gate = b._timeGate
                if (gate.lastSync is None or gate.count < gate.converge_samples
                        or not 0 <= now-gate.lastSync <= gate.sync_timeout_s):
                    self.samples.pop(mid, None)
                    return
                age = rosNow-(boot+gate.offsetNs*1e-9)
                if not math.isfinite(age) or not 0 <= age <= self.maxAge:
                    self.samples.pop(mid, None)
                    return
                observed = now-age
            self.lastBoot[mid] = boot
            self.samples[mid] = (observed, msg, epoch)
        except (ValueError, TypeError, AttributeError, OverflowError, struct.error):
            self.samples.pop(mid, None)

    def snapshot(self):
        now = self.backend._clock()
        state = self.backend.readVehicleState()
        epoch = (state.restart_epoch << 32) | state.frame_reset_epoch
        odom, status = self.samples.get(331), self.samples.get(230)
        if (not state.connected or odom is None or status is None
                or odom[2] != epoch or status[2] != epoch
                or not 0 <= now-odom[0] <= self.maxAge
                or not 0 <= now-status[0] <= self.maxAge
                or abs(odom[0]-status[0]) > self.maxSkew):
            self.reason = "ekf_evidence_missing_or_stale"
            return None
        flags = int(status[1].flags)
        accuracy = float(status[1].pos_horiz_accuracy)
        x, y, vx, vy, covX, covY, counter, z, vz, covZ = odom[1]
        if (flags & 11 != 11 or flags & 128
                or not math.isfinite(accuracy) or accuracy <= 0):
            self.reason = "ekf_horizontal_invalid"
            return None
        accuracy = max(accuracy, math.sqrt(max(covX, covY)))
        verticalAccuracy = float(status[1].pos_vert_accuracy)
        verticalValid = bool(flags & 36 == 36 and all(math.isfinite(v) for v in
            (z, vz, covZ, verticalAccuracy)) and covZ >= 0 and verticalAccuracy > 0)
        verticalAccuracy = max(verticalAccuracy, math.sqrt(covZ)) if verticalValid else None
        self.reason = "observed_unqualified"
        return (EkfReferenceSample(odom[0], (x, y), True, False, accuracy, epoch, counter,
                    z if verticalValid else None, vz if verticalValid else None,
                    verticalValid, verticalAccuracy), (vx, vy))

    def close(self):
        self.node.destroy_subscription(self.subscription)
        self.samples.clear()
