"""证据指纹：Git 快照与文件 SHA256，供 check_offline 与 sih_manifest 共用。"""
import hashlib
import os
from pathlib import Path
import subprocess


def git(directory, *args):
    return subprocess.check_output(["git", *args], cwd=directory)


def gitSnapshot(directory):
    """源码树快照：HEAD、工作树状态、二进制 diff 哈希与逐文件 SHA256。"""
    directory = Path(directory)
    manifest = {}
    for name in git(directory, "ls-files", "--cached", "--others", "--exclude-standard", "-z").split(b"\0"):
        if not name:
            continue
        path = directory / os.fsdecode(name)
        if path.is_file():
            manifest[os.fsdecode(name)] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(sha=git(directory, "rev-parse", "HEAD").decode().strip(),
                status=git(directory, "status", "--short").decode(),
                diff_sha256=hashlib.sha256(git(directory, "diff", "--binary", "HEAD")).hexdigest(),
                files=manifest)


def fingerprintFiles(paths):
    """{str(path): sha256}；调用方保留自己的文件清单，字段名与顺序不变。"""
    return {str(path): hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in paths}
