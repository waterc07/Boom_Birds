"""恢复 2026-09-14 为标定性能测试暂停的 VS Code 后台进程。

读取同目录 optimization_20260914/paused_backgrounds.json 中的 {pid, needle, starttime}，
只有三者同时匹配（/proc/<pid>/stat 的 starttime 字段与命令行包含 needle）才发送 SIGCONT，
避免误恢复已复用 PID 的其它进程。仅在 Linux 上有意义；暂停记录由设备端生成，不随仓库分发。
用法：python3 resume_dev_backgrounds.py
"""
from pathlib import Path
import json
import os
import signal

RECORD = Path(__file__).resolve().parent / "optimization_20260914" / "paused_backgrounds.json"


def main():
    if not RECORD.is_file():
        raise SystemExit(f"没有暂停记录：{RECORD}；该文件在设备端生成且被 .gitignore 忽略")
    resumed = 0
    for item in json.loads(RECORD.read_text()):
        proc = Path("/proc") / str(item["pid"])
        try:
            stat = proc.joinpath("stat").read_text().split()
            if stat[21] != item["starttime"]:
                continue
            cmdline = proc.joinpath("cmdline").read_bytes().replace(b"\0", b" ").decode()
            if item["needle"] not in cmdline:
                continue
            os.kill(item["pid"], signal.SIGCONT)
            resumed += 1
            print("Resumed", item["pid"])
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            pass
    print(f"恢复 {resumed} 个进程")


if __name__ == "__main__":
    main()
