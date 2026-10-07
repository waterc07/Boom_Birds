"""Source/config fingerprints for offline tools; excludes data and credentials."""
import hashlib
from pathlib import Path


def fingerprints(config_file):
    workspace = Path(__file__).resolve().parents[1]
    files = {Path(config_file).resolve()}
    for package in ("boom_birds_control", "boom_birds_sensing", "boom_birds_bringup"):
        files.update((workspace/"src"/package/package).glob("platform_*.py"))
    files.update(workspace.joinpath("tools").glob("*platform*.py"))
    files.add(workspace/"src/boom_birds_bringup/boom_birds_bringup/compute_release_node.py")
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
