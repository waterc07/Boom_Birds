"""Read-only failsafe observations from the authorized local SIH instance."""
import re
import subprocess
import threading
import time
from pathlib import Path
from .sih_guard import verify_sih_process

# These missing optional links can be present before the SIH mission starts.
OPTIONAL_LINKS = {"manual_control_signal_lost", "gcs_connection_lost"}
REQUIRED = {'parachute_unhealthy', 'global_position_invalid_relaxed', 'offboard_control_signal_lost', 'battery_low_remaining_time', 'vtol_fixed_wing_system_failure', 'local_position_invalid', 'fd_critical_failure', 'fd_motor_failure', 'local_velocity_invalid', 'local_position_invalid_relaxed', 'angular_velocity_invalid', 'position_accuracy_low', 'fd_alt_loss', 'navigator_failure', 'gcs_connection_lost', 'attitude_invalid', 'home_position_invalid', 'fd_esc_arming_failure', 'global_position_invalid', 'geofence_breached', 'battery_unhealthy', 'gnss_lost', 'wind_limit_exceeded', 'flight_time_limit_exceeded', 'auto_mission_missing', 'remote_id_unhealthy', 'manual_control_signal_lost', 'local_altitude_invalid', 'fd_imbalanced_prop', 'traffic_avoidance_unhealthy', 'battery_warning', 'mission_failure'}


def parse_flags(text):
    fields = dict(re.findall(r"^\s*(\w+):\s*(True|False|\d+)\s*$", text, re.MULTILINE))
    if not REQUIRED <= fields.keys():
        raise ValueError("incomplete failsafe_flags")
    age = re.search(r"timestamp:.*?\(([\d.]+) seconds ago\)", text)
    if age is None:
        raise ValueError("missing PX4 observation age")
    values = {k: (v == "True" if v in ("True", "False") else int(v)) for k, v in fields.items()}
    return values, float(age.group(1))


def classify_cause(flags, exempt_links, remote_id_required, offboard, previous):
    failures = {k for k, v in flags.items() if v is True}
    failures -= {"auto_mission_missing", "offboard_control_signal_lost"}
    failures -= exempt_links & OPTIONAL_LINKS
    if remote_id_required is False:
        failures.discard("remote_id_unhealthy")
    if failures or flags["battery_warning"] != 0:
        return "unknown"
    if flags["offboard_control_signal_lost"]:
        return "offboard_link"
    return "" if offboard else previous


class SihSafetyReader:
    def __init__(self, pid, vehicle_state):
        self.pid, self.vehicle_state = pid, vehicle_state
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.received = None
        self.mode = self.cause = ""
        self.wire_age = 0.
        self.exempt_links = None
        self.remote_id_required = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            age = float("inf") if self.received is None else time.monotonic() - self.received + self.wire_age
            return self.mode, self.cause, age

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)

    def _run(self):
        while not self.stop.is_set():
            try:
                if not verify_sih_process(self.pid):
                    raise ValueError("SIH instance not verified")
                binary = (Path("/proc") / str(self.pid) / "exe").resolve(strict=True)
                parameter = subprocess.run([str(binary.with_name("px4-param")), "show", "COM_ARM_ODID"],
                    cwd=binary.parent.parent / "rootfs/0", capture_output=True, text=True, timeout=.4)
                match = re.search(r"COM_ARM_ODID\s+\[[^]]+\]\s*:\s*(\d+)\s*$", parameter.stdout, re.MULTILINE)
                if parameter.returncode or match is None:
                    raise ValueError("Remote ID requirement unavailable")
                self.remote_id_required = int(match.group(1)) != 0
                before = self.vehicle_state()
                result = subprocess.run([str(binary.with_name("px4-listener")), "failsafe_flags", "-n", "1"],
                    cwd=binary.parent.parent / "rootfs/0", capture_output=True, text=True, timeout=.4)
                flags, wire_age = parse_flags(result.stdout)
                after = self.vehicle_state()
                if result.returncode or (before.restart_epoch, before.mode_detail) != (after.restart_epoch, after.mode_detail):
                    raise ValueError("changed observation epoch or mode")
                if self.exempt_links is None and after.armed and after.is_offboard:
                    self.exempt_links = {k for k in OPTIONAL_LINKS if flags.get(k) is True}
                if self.exempt_links is not None:
                    self.exempt_links &= {k for k in OPTIONAL_LINKS if flags.get(k) is True}
                cause = classify_cause(flags, self.exempt_links or set(), self.remote_id_required,
                                       after.is_offboard, self.cause)
                with self.lock:
                    self.mode, self.cause = after.mode_detail or "unknown", cause
                    self.received, self.wire_age = time.monotonic(), wire_age
            except (OSError, ValueError, subprocess.TimeoutExpired):
                with self.lock:
                    self.received = None
            self.stop.wait(.5)
