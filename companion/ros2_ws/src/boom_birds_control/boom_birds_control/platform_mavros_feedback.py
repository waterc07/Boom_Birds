"""MAVROS raw target/estimator evidence candidate; hardware handoff remains gated."""
import math
from .mavros_observation import decodeObservation
from .platform_landing import HandoffFeedback


class MavrosVelocityFeedback:
    """Read FCU target output and EKF flags without a PX4 shell or uORB reader.

    POSITION_TARGET_LOCAL_NED is the controller output, not the original
    trajectory input. This candidate requires SIH and hardware comparison
    before it may authorize real compute release.
    """
    source = "PX4_MAVLINK_VELOCITY_TARGET"

    def __init__(self, node, backend, core):
        from mavros_msgs.msg import Mavlink
        from rclpy.qos import qos_profile_sensor_data
        self.node, self.backend, self.core = node, backend, core
        self.samples = {}
        self.lastBoot = {}
        self.epoch = None
        self.diagnostic = {}
        backend.platform_feedback_required = True
        self.subscription = node.create_subscription(Mavlink,
            f"/uas{backend.target_system}/mavlink_source", self.onRaw, qos_profile_sensor_data)

    def currentEpoch(self, state):
        return (state.restart_epoch << 32) | state.frame_reset_epoch

    def onRaw(self, raw):
        b = self.backend
        if (not b._endpointVerified or raw.sysid != b.target_system
                or raw.compid != b.target_component or raw.msgid not in (85, 230)):
            return
        try:
            msg = decodeObservation(raw)
            if msg is None:
                return
            now = b._clock()
            rosNow = self.node.get_clock().now().nanoseconds*1e-9
            receipt = raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9
            age = rosNow-receipt
            if not math.isfinite(age) or not 0 <= age <= .2:
                return
            state = b.readVehicleState()
            epoch = self.currentEpoch(state)
            if self.epoch != epoch:
                self.samples.clear(); self.lastBoot.clear(); self.epoch = epoch
            boot = float(msg.time_boot_ms)*1e-3 if raw.msgid == 85 else float(msg.time_usec)*1e-6
            if not math.isfinite(boot) or boot <= self.lastBoot.get(raw.msgid, -math.inf):
                return
            if b._sihReceiptTime:
                # SIH boot clock is lockstep; this evidence remains marked test-only.
                observed = now-age
            else:
                gate = b._timeGate
                if (gate.lastSync is None or gate.count < gate.converge_samples
                        or not 0 <= now-gate.lastSync <= gate.sync_timeout_s):
                    return
                sampleAge = rosNow-(boot+gate.offsetNs*1e-9)
                if not math.isfinite(sampleAge) or not 0 <= sampleAge <= .2:
                    return
                observed = now-sampleAge
            self.lastBoot[raw.msgid] = boot
            self.samples[raw.msgid] = (observed, msg, epoch)
        except (ValueError, TypeError, AttributeError, OverflowError):
            return

    def snapshot(self):
        now = self.backend._clock()
        state = self.backend.readVehicleState()
        epoch = self.currentEpoch(state)
        estimate = self.samples.get(230)
        valid = bool(state.connected and estimate is not None and estimate[2] == epoch
            and 0 <= now-estimate[0] <= .2 and int(estimate[1].flags) & 7 == 7)
        target = self.samples.get(85)
        self.diagnostic = dict(stamp=now, estimator_valid=valid, source=self.source,
            target_age_s=None if target is None else now-target[0],
            estimator_age_s=None if estimate is None else now-estimate[0])
        if (not valid or target is None or target[2] != epoch
                or not 0 <= now-target[0] <= .2 or not state.is_offboard
                or self.backend.control_mode != "px4_position"
                or getattr(self.backend, "_attitudePub", None) is not None
                or getattr(self.backend, "_platformToken", None) != self.core.token):
            return None, valid
        observed, msg, _ = target
        velocity = (msg.vx, msg.vy, msg.vz)
        if (msg.coordinate_frame != 1 or msg.type_mask & 7 != 7 or msg.type_mask & 56
                or not all(math.isfinite(v) for v in velocity)):
            return None, valid
        matches = [seq for seq, (sent, value) in list(self.core.sent_history.items())
                   if 0 <= observed-sent <= .3 and value is not None and len(value) == 3
                   and all(abs(a-b) <= .01 for a,b in zip(velocity, value))]
        if not matches:
            return None, valid
        return HandoffFeedback(observed,self.core.token,max(matches),epoch,
            True,False,velocity,self.source), valid

    def close(self):
        self.node.destroy_subscription(self.subscription)
        self.samples.clear()
