#!/usr/bin/env bash
# 参数：解释器、源码目录、安装前缀、CMake binary 目录、包名。
set -euo pipefail
py="${1:?python}"; src="${2:?source}"; prefix="${3:?prefix}"
buildDir="${4:?build directory}"; pkg="${5:?package}"
[[ "$pkg" == boom_birds_* || "$pkg" == stereo_depth ]] || exit 2
src="$(realpath "$src")"; prefix="$(realpath -m "$prefix")"
buildDir="$(realpath -m "$buildDir")"
[[ "$prefix" != / && "$prefix" != "$src" && "$buildDir" != "$src" ]] || exit 2
cd "$src"
# setup.py install 只增不删；同时清理 build/lib，避免迁出的模块重新安装。
rm -rf "${buildDir}/python" "${buildDir}/${pkg}.egg-info"
for installed in "${prefix}"/lib/python*/site-packages/"${pkg}"; do
  [[ ! -e "$installed" && ! -L "$installed" ]] || rm -rf "$installed"
done
BOOM_BIRDS_COLCON_BUILD=1 "$py" setup.py egg_info --egg-base="$buildDir" build --build-base="${buildDir}/python" install \
  --prefix="$prefix" --single-version-externally-managed --record="${buildDir}/install_manifest.txt"
mapfile -t names < <(grep -oE '"[A-Za-z0-9_.-]+[[:space:]]*=[[:space:]]*'"${pkg}"'\.' setup.py | sed -E 's/^"([A-Za-z0-9_.-]+).*/\1/' | sort -u)
libexec="${prefix}/lib/${pkg}"
mkdir -p "$libexec"
for name in "${names[@]}"; do
  if [[ -f "${prefix}/bin/${name}" ]]; then
    mv -f "${prefix}/bin/${name}" "${libexec}/${name}"
    chmod +x "${libexec}/${name}"
  fi
done
for installed in "$libexec"/*; do
  [[ -f "$installed" || -L "$installed" ]] || continue
  # 原生 .so 由 CMake 管理。
  [[ "$installed" == *.so ]] && continue
  keep=0
  for name in "${names[@]}"; do [[ "${installed##*/}" != "$name" ]] || keep=1; done
  [[ "$keep" == 1 ]] || rm -f "$installed"
done
# config/launch 中被迁走的文件不能残留在增量安装树。
for directory in config launch; do
  target="${prefix}/share/${pkg}/${directory}"
  if [[ -d "$target" && ! -L "$target" ]]; then
    while IFS= read -r -d '' installed; do
      relative="${installed#"$target"/}"
      [[ -e "${src}/${directory}/${relative}" ]] || rm -f "$installed"
    done < <(find "$target" \( -type f -o -type l \) -print0)
    find "$target" -mindepth 1 -type d -empty -delete
  fi
done
for name in "${names[@]}"; do
  [[ -x "${libexec}/${name}" ]] || { echo "缺少入口：${pkg}/${name}" >&2; exit 1; }
done
echo "[${pkg}] libexec: ${names[*]:-(none)}"
