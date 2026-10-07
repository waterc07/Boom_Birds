"""Single-owner output arbiter and compute-release callbacks."""
from .platform_landing import PlatformLanding, VELOCITY_YAW_RATE_MASK


class PlatformExecutor:
    def __init__(self, backend, profile, *, test_only=False, revoke_navigation,
                 release_compute, native_land, disarm):
        self.core = PlatformLanding(profile, test_only=test_only)
        self.backend = backend
        self.revoke_navigation = revoke_navigation
        self.release_compute = release_compute
        self.native_land = native_land
        self.disarm = disarm
        self.owner = "navigation"
        self.revoked = False
        self.release_requested = False
        self.release_verified = False
        self.land_requested = False
        self.disarm_requested = False
        self.release_error = ""
        self.history = []

    def navigation_attitude(self, setpoint):
        if self.owner != "navigation":
            return False
        return self.backend.send_attitude_setpoint(setpoint)

    def navigation_position(self, setpoint, mask):
        if self.owner != "navigation":
            return False
        return self.backend.send_setpoint(setpoint, mask)

    def request(self, intent="land"):
        self.core.request(intent)

    def tick(self, now, observation, range_sample, flight, feedback=None, *, navigation_ready=False):
        # Acquire valid distinct samples while navigation still owns the output.
        if self.core.state == "ACQUIRE" and navigation_ready and self.core.good >= self.core.cfg.acquire_samples:
            if not self.revoked:
                self.revoked = self.revoke_navigation() is True
                if not self.revoked:
                    self.core.cancel("navigation_revoke_failed")
        out = self.core.step(now, observation, range_sample, flight, feedback,
                             navigation_revoked=self.revoked, navigation_ready=navigation_ready, release_ack=self.release_verified)
        if out.state == "PREPARE" and self.owner == "navigation":
            self.owner = "platform"
            switch = getattr(self.backend, "activate_platform_velocity", None)
            if switch is not None and not switch(out.token):
                self.core.cancel("backend_handoff_failed")
                out = self.core.step(now)
        if out.velocity_ned is not None:
            sent = self.backend.send_setpoint(out.setpoint(), VELOCITY_YAW_RATE_MASK)
            self.core.note_sent(out, now, sent)
            if not sent:
                out = self.core.step(now)
        if out.release_compute and not self.release_requested:
            self.release_requested = True
            try:
                # Callback requests shutdown; verification must come from actual process/resource state.
                self.release_verified = self.release_compute(out.token) is True
                if not self.release_verified:
                    self.release_error = "compute_release_not_confirmed"
            except Exception as exc:
                self.release_error = str(exc)
        if out.request_native_land and not self.land_requested:
            self.owner = "fallback"
            self.land_requested = self.native_land(out.reason) is True
        if out.request_disarm and not self.disarm_requested:
            self.disarm_requested = self.disarm() is True
        if out.state == "COMPLETE":
            self.owner = "none"
        self.history.append(dict(now=now, owner=self.owner, output=out,
            release_requested=self.release_requested, release_verified=self.release_verified,
            release_error=self.release_error))
        return out

    def cancel(self, reason="cancelled"):
        self.owner = "fallback"
        self.core.cancel(reason)
