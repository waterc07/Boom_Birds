# ******************************************************************************
# @file           : test_sih_ulog.py
# @brief          : 验证 SIH 只归档 PX4 本轮声明且新增的日志。
# ******************************************************************************
# @author         : water_c
# ******************************************************************************

from hashlib import sha256
from pathlib import Path


def test_archive_only_new_log_announced_by_this_run(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[3]))
    from tools.sih_env import copyRunUlog, snapshotUlogs

    log = tmp_path / "rootfs" / "log"
    log.mkdir(parents=True)
    previousLog = log / "previous.ulg"
    previousLog.write_bytes(b"previous")
    previous = snapshotUlogs(log.parent)

    currentLog = log / "current.ulg"
    currentLog.write_bytes(b"current")
    previousLog.write_bytes(b"modified previous")
    (log / "unannounced.ulg").write_bytes(b"unannounced")
    outsideLog = tmp_path / "outside.ulg"
    outsideLog.write_bytes(b"outside")

    target = tmp_path / "px4.ulg"
    console = tmp_path / "px4.log"
    console.write_text(
        "INFO  [logger] Opened full log file: ./log/previous.ulg\n"
        "INFO  [logger] Opened full log file: ../outside.ulg\n"
    )
    assert copyRunUlog(log.parent, console, previous, target) is None
    assert not target.exists()

    with console.open("a") as stream:
        stream.write("INFO  [logger] Opened full log file: ./log/current.ulg\n")
    assert copyRunUlog(log.parent, console, previous, target) == sha256(b"current").hexdigest()
    assert target.read_bytes() == b"current"
