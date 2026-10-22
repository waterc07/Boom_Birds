import threading
import time

from tools.vio_calibration.telemetry_journal import TelemetryJournal


def test_slow_sync_does_not_block_serial_forwarding(tmp_path, monkeypatch):
    from pymavlink.dialects.v20 import common as mav
    from tools.vio_calibration.vio_calibration_inputs import TelemetryLink
    entered = threading.Event()
    release = threading.Event()
    original = TelemetryJournal._sync

    def slow_sync(self):
        entered.set()
        assert release.wait(3)
        original(self)

    monkeypatch.setattr(TelemetryJournal, "_sync", slow_sync)
    journal = TelemetryJournal(tmp_path, sync_interval=0)
    link = object.__new__(TelemetryLink)
    link.journal = journal
    link.parser = mav.MAVLink(None)
    link.parser.robust_parsing = True
    link.armed = False
    encoder = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    packet = encoder.heartbeat_encode(2, 12, 0, 0, 0).pack(encoder)
    forwarded = []
    try:
        assert entered.wait(1)
        started = time.monotonic()
        for _ in range(50):
            link.consume(packet, forwarded.append)
        assert time.monotonic() - started < .25
        assert len(forwarded) == 50 and link.hb is not None
        assert not release.is_set()
    finally:
        release.set()
        result = journal.close()
    assert result["complete"] and result["pending"] == 0
    assert (tmp_path / "serial-rx.bin").read_bytes() == packet * 50


def test_queue_overflow_fails_capture_instead_of_dropping_silently(tmp_path, monkeypatch):
    from tools.vio_calibration.vio_calibration_inputs import TelemetryLink
    import pytest
    entered = threading.Event()
    release = threading.Event()
    original = TelemetryJournal._sync

    def slow_sync(self):
        entered.set()
        assert release.wait(3)
        original(self)

    monkeypatch.setattr(TelemetryJournal, "_sync", slow_sync)
    journal = TelemetryJournal(tmp_path, capacity=1, sync_interval=0)
    try:
        assert entered.wait(1)
        journal.write("rx", b"first")
        journal.write("rx", b"overflow")
        link = object.__new__(TelemetryLink)
        link.journal = journal
        with pytest.raises(RuntimeError, match="queue full"):
            link.check()
    finally:
        release.set()
        result = journal.close()
    assert not result["complete"]
    assert (tmp_path / "serial-rx.bin").read_bytes() == b"first"


def test_sync_failure_is_reported(tmp_path, monkeypatch):
    def failed_sync(self):
        raise OSError("injected disk error")
    monkeypatch.setattr(TelemetryJournal, "_sync", failed_sync)
    journal = TelemetryJournal(tmp_path)
    journal.write("tx", b"command")
    result = journal.close()
    assert not result["complete"] and "injected disk error" in result["error"]
