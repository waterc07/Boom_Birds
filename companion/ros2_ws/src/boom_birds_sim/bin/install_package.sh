#!/usr/bin/env bash
# 由 CMakeLists.txt 在安装阶段调用：安装 Python 包并布置 libexec。
#
# ROS 2 的 ros2 run/launch 约定：可执行文件必须位于 <prefix>/lib/<pkg>/。
# 本仓库用 setup.py install 安装（不走 ament_python 的 build_py），因此安装后
# 需要把 console_scripts 从 <prefix>/bin 移到 <prefix>/lib/<pkg>。
# 参数：$1=python 解释器 $2=源码目录 $3=安装前缀
#
# 为什么入口点清单从 setup.py 推导（而不是安装记录）：
#   重建时 `setup.py install` 会跳过"已是最新"的脚本，于是 --record 写出的清单
#   可能**不完整**。曾经用记录做白名单来清理陈旧入口，结果把仍然声明着的
#   `lifecycle_node` 从 libexec 删掉，构建却是成功的（拆包第二批实测）。
#   声明文件才是唯一真值：先按它搬运，再按它清理，最后按它核对。
set -euo pipefail
PY="${1:?python}"; SRC="${2:?source}"; PREFIX="${3:?prefix}"
PKG=boom_birds_sim

cd "${SRC}"
# 强制重新生成脚本：`setup.py install` 会按 egg-info 判断"已是最新"而**跳过**脚本，
# 于是被清理掉（或从未生成）的入口点无法恢复，最后的"声明即必须有"核对就会失败。
rm -rf "${SRC}/${PKG}.egg-info"
# 清空本包已安装的 site-packages 目录再装：`setup.py install` 只增不删，源里删掉的
# 模块会一直留在安装树里，于是"源里已迁走"和"import 还能拿到旧实现"同时成立
# （拆包第四批实测：bringup 里残留 runtime_config/frames/control_protocol）。
for _sp in "${PREFIX}"/lib/python*/site-packages/"${PKG}"; do
  [[ -d "$_sp" ]] && rm -rf "$_sp" && echo "[${PKG}] 清理已安装模块目录：${_sp}"
done
# 还要清掉 build/lib：install_lib 是从那里复制过去的，源里删掉的模块会先留在
# build/lib，再被重新拷回 site-packages（只清 site-packages 不够，实测如此）。
rm -rf "${SRC}/build"
"${PY}" setup.py install \
  --prefix="${PREFIX}" --single-version-externally-managed --record=install_manifest.txt

mapfile -t NAMES < <(
  grep -oE '"[A-Za-z0-9_.-]+[[:space:]]*=[[:space:]]*'"${PKG}"'\.' "${SRC}/setup.py" \
    | sed -E 's/^"([A-Za-z0-9_.-]+).*/\1/' | sort -u
)
# 声明 0 个入口点是**合法**的（例如拆包后的 boom_birds_nav 只是兼容转发层），
# 此时 libexec 会被清空；但一定要把这件事说出来，避免"悄悄没有可执行文件"。
if [[ ${#NAMES[@]} -eq 0 ]]; then
  echo "[${PKG}] setup.py 未声明 console_scripts：本包不提供可执行文件，libexec 将被清空"
fi

LIBEXEC="${PREFIX}/lib/${PKG}"
mkdir -p "${LIBEXEC}"
for name in "${NAMES[@]}"; do
  if [[ -f "${PREFIX}/bin/${name}" ]]; then
    mv -f "${PREFIX}/bin/${name}" "${LIBEXEC}/${name}"
    chmod +x "${LIBEXEC}/${name}"
  fi
done

# 清理陈旧入口：不在声明清单里的 libexec 文件必须删掉，否则改名/迁走后旧可执行
# 文件会一直挂着，`ros2 pkg executables` 仍列得出来。
for existing in "${LIBEXEC}"/*; do
  [[ -e "$existing" ]] || continue
  name="$(basename "$existing")"
  keep=0
  for wanted in "${NAMES[@]}"; do [[ "$name" == "$wanted" ]] && keep=1 && break; done
  if [[ "$keep" -eq 0 ]]; then
    rm -f "$existing"
    echo "[${PKG}] 清理陈旧入口：${name}"
  fi
done

# 清理陈旧配置：安装只增不删，源里删掉的配置文件会一直留在 share/<pkg>/config 里。
# runtime.yaml 从 nav 迁到 control 后，nav 的 share 里仍留着旧副本——它与真实配置
# 同名同结构，却属于「看起来合法」的过期副本，正是 config_path() 最怕的东西
# （拆包第五批实测）。这里按「源码里还有没有这个相对路径」逐个删除。
SHARE_CONFIG="${PREFIX}/share/${PKG}/config"
if [[ -d "${SRC}/config" && -d "${SHARE_CONFIG}" ]]; then
  while IFS= read -r -d '' installed; do
    rel="${installed#"${SHARE_CONFIG}"/}"
    if [[ ! -e "${SRC}/config/${rel}" ]]; then
      rm -f "$installed"
      echo "[${PKG}] 清理陈旧配置：config/${rel}"
    fi
  done < <(find "$SHARE_CONFIG" -type f -print0)
  find "$SHARE_CONFIG" -mindepth 1 -type d -empty -delete 2>/dev/null || true
fi

# 收尾核对：每个**声明**的入口点都必须确实在 libexec 里，缺一个就不算装好。
missing=0
for name in "${NAMES[@]}"; do
  if [[ ! -x "${LIBEXEC}/${name}" ]]; then
    echo "[${PKG}] 错误：声明了 ${name} 但 ${LIBEXEC} 里没有它" >&2
    missing=1
  fi
done
[[ "$missing" -eq 0 ]] || exit 1
echo "[${PKG}] libexec(${#NAMES[@]}): $(cd "${LIBEXEC}" && ls | tr '\n' ' ')"
