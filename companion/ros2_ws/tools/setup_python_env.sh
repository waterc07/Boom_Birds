#!/usr/bin/env bash
# 不隐式读取用户 site-packages；--install 显式安装锁定的脱机测试依赖。
set -euo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${BOOM_BIRDS_VENV:-${WS_ROOT}/.venv}"
python3 -m venv --without-pip "$VENV"
SITE="$("$VENV/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
printf '%s\n' /usr/lib/python3/dist-packages > "$SITE/zz_system_dist_packages.pth"
# 仅删除旧脚本创建的这一份注入文件。
rm -f "$SITE/zz_user_local_packages.pth"
if [[ "${1:-}" == "--install" ]]; then
  PYTHONNOUSERSITE=1 /usr/bin/python3 -m pip --python "$VENV/bin/python" install -r "$WS_ROOT/requirements-mavlink.txt"
elif [[ "$#" != 0 ]]; then
  echo "用法：setup_python_env.sh [--install]" >&2; exit 2
fi
BB_CHECK_TEST_DEPS="${1:-}" PYTHONNOUSERSITE=1 "$VENV/bin/python" - <<'PY'
import sys
import os
import numpy, cv2
modules = [numpy, cv2]
if os.environ.get("BB_CHECK_TEST_DEPS") == "--install":
    import pymavlink, serial
    modules.extend((pymavlink, serial))
from pathlib import Path
for module in modules:
    path = Path(module.__file__).resolve()
    if Path.home() / '.local' in path.parents:
        raise SystemExit(f'拒绝用户目录依赖：{path}')
    print(module.__name__, path)
PY
