import threading
import time
from boom_birds_sensing.platform_worker import LatestFrameWorker


def test_latest_pending_frame_replaces_backlog_and_failure_does_not_kill_worker():
    entered, release, done = threading.Event(), threading.Event(), threading.Event()
    seen = []
    def process(item):
        seen.append(item)
        if item == 0:
            entered.set()
            assert release.wait(2.)
        elif item == 99:
            done.set()
            raise ValueError("injected")
    worker = LatestFrameWorker(process)
    try:
        assert worker.submit(0) and entered.wait(2.)
        for i in range(1, 100):
            assert worker.submit(i)
        release.set()
        assert done.wait(2.)
        deadline = time.monotonic()+2.
        while worker.snapshot()["errors"] == 0 and time.monotonic() < deadline:
            time.sleep(.001)
        stats = worker.snapshot()
        assert seen == [0, 99]
        assert stats["received"] == 100 and stats["replaced"] == 98 and stats["errors"] == 1
    finally:
        release.set()
        assert worker.close()
    assert not worker.submit(100)
