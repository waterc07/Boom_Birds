from pathlib import Path
import sys

WS = Path(__file__).resolve().parents[3]
for path in (WS, WS / "src/stereo_depth"):
    sys.path.insert(0, str(path))
