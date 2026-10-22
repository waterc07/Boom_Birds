#!/usr/bin/env bash
# 参数：解释器、源码目录、安装前缀、CMake binary 目录、包名。
set -euo pipefail
PY="${1:?python}"; SRC="${2:?source}"; PREFIX="${3:?prefix}"
BUILD_DIR="${4:?build directory}"; PKG="${5:?package}"
[[ "$PKG" == boom_birds_* || "$PKG" == stereo_depth ]] || exit 2
SRC="$(realpath "$SRC")"; PREFIX="$(realpath -m "$PREFIX")"
BUILD_DIR="$(realpath -m "$BUILD_DIR")"
[[ "$PREFIX" != / && "$PREFIX" != "$SRC" && "$BUILD_DIR" != "$SRC" ]] || exit 2
cd "$SRC"
# setup.py install 只增不删；同时清理 build/lib，避免迁出的模块重新安装。
rm -rf "${BUILD_DIR}/python" "${BUILD_DIR}/${PKG}.egg-info"
for installed in "${PREFIX}"/lib/python*/site-packages/"${PKG}"; do
  [[ ! -e "$installed" && ! -L "$installed" ]] || rm -rf "$installed"
done
BOOM_BIRDS_COLCON_BUILD=1 "$PY" setup.py egg_info --egg-base="$BUILD_DIR" build --build-base="${BUILD_DIR}/python" install \
  --prefix="$PREFIX" --single-version-externally-managed --record="${BUILD_DIR}/install_manifest.txt"
mapfile -t NAMES < <(grep -oE '"[A-Za-z0-9_.-]+[[:space:]]*=[[:space:]]*'"${PKG}"'\.' setup.py | sed -E 's/^"([A-Za-z0-9_.-]+).*/\1/' | sort -u)
LIBEXEC="${PREFIX}/lib/${PKG}"
mkdir -p "$LIBEXEC"
for name in "${NAMES[@]}"; do
  if [[ -f "${PREFIX}/bin/${name}" ]]; then
    mv -f "${PREFIX}/bin/${name}" "${LIBEXEC}/${name}"
    chmod +x "${LIBEXEC}/${name}"
  fi
done
for installed in "$LIBEXEC"/*; do
  [[ -f "$installed" || -L "$installed" ]] || continue
  # 原生 .so 由 CMake 管理。
  [[ "$installed" == *.so ]] && continue
  keep=0
  for name in "${NAMES[@]}"; do [[ "${installed##*/}" != "$name" ]] || keep=1; done
  [[ "$keep" == 1 ]] || rm -f "$installed"
done
# config/launch 中被迁走的文件不能残留在增量安装树。
for directory in config launch; do
  target="${PREFIX}/share/${PKG}/${directory}"
  if [[ -d "$target" && ! -L "$target" ]]; then
    while IFS= read -r -d '' installed; do
      relative="${installed#"$target"/}"
      [[ -e "${SRC}/${directory}/${relative}" ]] || rm -f "$installed"
    done < <(find "$target" \( -type f -o -type l \) -print0)
    find "$target" -mindepth 1 -type d -empty -delete
  fi
done
for name in "${NAMES[@]}"; do
  [[ -x "${LIBEXEC}/${name}" ]] || { echo "缺少入口：${PKG}/${name}" >&2; exit 1; }
done
echo "[${PKG}] libexec: ${NAMES[*]:-(none)}"
