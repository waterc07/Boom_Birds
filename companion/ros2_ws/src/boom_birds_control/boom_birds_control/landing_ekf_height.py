"""TEST-ONLY range-to-EKF height reference; no contact detection."""
import math
from .landing_ekf_reference import EkfReferenceSample


class LandingEkfHeight:
    def __init__(self, *, max_age_s, max_pair_skew_s, max_accuracy_m,
                 max_duration_s, max_speed_m_s, max_height_m):
        limits = (max_age_s,max_pair_skew_s,max_accuracy_m,max_duration_s,max_speed_m_s,max_height_m)
        if any(type(v) not in (int,float) or not math.isfinite(v) or v <= 0 for v in limits):
            raise ValueError("height_reference_limits")
        self.maxAge,self.maxSkew,self.maxAccuracy,self.maxDuration,self.maxSpeed,self.maxHeight = limits
        self.reset()

    def reset(self):
        self.anchor = self.previous = None
        self.mode, self.reason = "range", "unseeded"

    def step(self, now, sample, range_stamp=None, range_height=None):
        if self.mode == "failed":
            return None
        valid = (isinstance(sample,EkfReferenceSample)
            and type(sample.vertical_position_valid) is bool and sample.vertical_position_valid
            and all(type(v) in (int,float) and math.isfinite(v) for v in
                    (now,sample.stamp,sample.position_ned_z,sample.velocity_ned_z,sample.vertical_accuracy_m))
            and 0 <= now-sample.stamp <= self.maxAge
            and 0 < sample.vertical_accuracy_m <= self.maxAccuracy
            and abs(sample.velocity_ned_z) <= self.maxSpeed)
        if not valid:
            self.mode,self.reason,self.anchor = "failed","ekf_vertical_invalid",None
            return None
        if self.anchor is not None:
            _,_,_,epoch,counter = self.anchor
            if (sample.epoch != epoch or sample.reset_counter != counter
                    or sample.stamp < self.previous.stamp):
                self.mode,self.reason,self.anchor = "failed","ekf_vertical_reset",None
                return None
            dt = sample.stamp-self.previous.stamp
            dz = abs(sample.position_ned_z-self.previous.position_ned_z)
            if (dt == 0 and dz > 0) or (dt > 0 and dz/dt > self.maxSpeed):
                self.mode,self.reason,self.anchor = "failed","ekf_vertical_jump",None
                return None
        if self.mode == "range" and range_height is not None:
            validRange = (type(range_height) in (int,float) and math.isfinite(range_height)
                and 0 < range_height <= self.maxHeight
                and type(range_stamp) in (int,float) and math.isfinite(range_stamp)
                and 0 <= now-range_stamp <= self.maxAge
                and abs(range_stamp-sample.stamp) <= self.maxSkew)
            if validRange:
                self.anchor=(sample.stamp,sample.position_ned_z,range_height,sample.epoch,sample.reset_counter)
                self.previous=sample
                self.reason="range"
                return range_height
            # A bad pair is not treated as a new anchor.
        if self.mode == "range":
            self.mode = "ekf"
        if self.anchor is None:
            self.mode,self.reason="failed","height_anchor_missing"
            return None
        stamp,z,height,_,_ = self.anchor
        if now-stamp > self.maxDuration:
            self.mode,self.reason,self.anchor="failed","height_anchor_expired",None
            return None
        height = height-(sample.position_ned_z-z)
        if not 0 < height <= self.maxHeight:
            self.mode,self.reason,self.anchor="failed","height_out_of_bounds",None
            return None
        self.previous=sample
        self.reason="ekf"
        return height
