#!/usr/bin/env python3
"""Offline regression for Tag tasks and the affected sensing/control packages."""
import os
from pathlib import Path
import sys
import subprocess


def main():
    root=Path(__file__).resolve().parents[1]
    repo=root.parents[1]
    for package in ("stereo_depth","boom_birds_bringup","boom_birds_control","boom_birds_sensing","boom_birds_sim","boom_birds_nav"):
        sys.path.insert(0,str(root/"src"/package))
    sys.path.insert(0,str(repo))
    os.environ["PYTHONPATH"]=os.pathsep.join(sys.path)
    arguments=sys.argv[1:]
    if not arguments:
        results=os.environ.get("BB_TAG_TEST_RESULTS")
        if results:Path(results).mkdir(parents=True,exist_ok=True)
        failed=False
        for package in ("boom_birds_bringup","boom_birds_control","boom_birds_sensing"):
            command=[sys.executable,str(Path(__file__).resolve()),str(root/"src"/package/"test"),"-q","--tb=short"]
            if results:command.append("--junitxml="+str(Path(results)/(package+".xml")))
            failed=subprocess.run(command).returncode!=0 or failed
        return int(failed)
    import pytest
    return pytest.main(arguments)


if __name__=="__main__":
    raise SystemExit(main())
