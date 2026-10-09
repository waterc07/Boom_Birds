import threading
import time
from boom_birds_sensing.platform_worker import LatestFrameWorker


def test_latest_pending_frame_replaces_backlog_and_failure_does_not_kill_worker():
    entered, release, done = threading.Event(), threading.Event(), threading.Event()
    continued = threading.Event()
    seen = []
    def process(item):
        seen.append(item)
        if item == 0:
            entered.set()
            assert release.wait(2.)
        elif item == 99:
            done.set()
            raise ValueError("injected")
        elif item == 100:
            continued.set()
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
        assert worker.submit(100) and continued.wait(2.)
        assert seen == [0, 99, 100]
    finally:
        release.set()
        assert worker.close()
    assert not worker.submit(100)


def test_fatal_failure_discards_backlog_and_is_chained_to_submit():
    import pytest
    entered, release = threading.Event(), threading.Event()
    failure = ValueError("processor failed")
    seen = []
    def process(item):
        seen.append(item)
        entered.set()
        assert release.wait(2.)
        raise failure
    worker = LatestFrameWorker(process, fatal_errors=True)
    try:
        worker.submit(1)
        assert entered.wait(2.)
        worker.submit(2)
        release.set()
        worker.thread.join(2.)
        assert not worker.thread.is_alive()
        assert seen == [1] and not worker.snapshot()["pending"]
        with pytest.raises(RuntimeError) as error:
            worker.submit(3)
        assert error.value.__cause__ is failure
    finally:
        release.set()
        assert worker.close()


def test_close_reports_a_still_running_processor():
    entered, release = threading.Event(), threading.Event()
    def process(item):
        entered.set()
        release.wait(2.)
    worker = LatestFrameWorker(process)
    try:
        worker.submit(1)
        assert entered.wait(2.)
        assert not worker.close(timeout=.01)
        assert not worker.submit(2)
    finally:
        release.set()
        assert worker.close()
