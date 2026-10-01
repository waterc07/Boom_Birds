#!/usr/bin/env bash
# SIH 故障/反向矩阵驱动：按行跑 run_sih_mission.sh，并把每个场景的关键结果
# 汇总成 JSONL（失败与未执行原因分开记录，不挑选通过结果）。
#
# 用法：bash companion/ros2_ws/tools/run_sih_matrix.sh <batch 名> <spec 文件>
# spec 每行：scenario|scene|gx|gy|gz|fault|obl_action|timeout|fault_at|forest_seed|inject_alt_below
#          |when_ready(1/空)|stable_s|transient_relaunch_s|inject_mode_detail|planner_suspend_s|forest_profile
#   fault 取 none/规划取消/深度断流/里程计断流/setpoint中断/模式确认失败/人工取消/
#            飞控重启/飞控重启恢复/编排器重启/相机断流
#   fault_at 取 EXECUTING/TAKEOFF/...；inject_alt_below 只在需要"高度证明"时给
#   以 # 开头的行与空行忽略
#
# 串行护栏（硬约束）：每个场景开始前都 flock 等构建锁（与 build_all.sh 同一把），
# 并确认没有**别的** PX4/ROS/SIH 进程、UDP 14580 未被占用；有残留即拒绝启动。
# 本脚本自己的进程树（含 run_sih_mission.sh 子进程）不算残留。
set -o pipefail
# 自己的进程 id 用 BASHPID **在顶层**取一次：`$$` 在子 shell 里仍是调用者的 pid，
# 而 `bash script.sh` 实际跑的进程可能是它的子 shell（实测两者不等），只认 $$ 会把
# 本脚本自己当成残留进程从而拒绝启动。
SELF_PID="$BASHPID"
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"
BATCH="${1:?缺少 batch 名}"
SPECS="${2:?缺少 spec 文件}"
[[ -f "$SPECS" ]] || { echo "spec 不存在：$SPECS" >&2; exit 2; }

export BOOM_BIRDS_VENV="${BOOM_BIRDS_VENV:-/home/waterc/bb_build/architecture/venv}"
ROOT_EVID="${BB_SIH_MATRIX_EVID:-/home/waterc/bb_build/architecture/evidence/deepseek-08/sih-matrix}"
SUMMARY="$ROOT_EVID/$BATCH.jsonl"
[[ ! -e "$ROOT_EVID/$BATCH" && ! -e "$SUMMARY" ]] || { echo "批次证据已存在，拒绝覆盖：$BATCH" >&2; exit 2; }
mkdir -p "$ROOT_EVID/$BATCH"
: > "$SUMMARY"
cp -- "$SPECS" "$ROOT_EVID/$BATCH/cases.spec"
GUARD_LOG="$ROOT_EVID/$BATCH/GUARD.txt"
: > "$GUARD_LOG"
# SIH 运行标记：别的成员（Lead 的脱机验收）据此判断"当前有没有 SIH 在跑"，
# 避免他们的检查与 SIH 抢同一套 ROS 话题导致互相污染。batch 期间存在，退出即删。
RUNNING_FLAG="$ROOT_EVID/RUNNING"
mark_running() {
  { echo "pid=$SELF_PID started=$(date -Is) batch=$BATCH";
    echo "specs=$SPECS";
    echo "scenario=$1";
  } > "$RUNNING_FLAG"
}
clear_running() { rm -f "$RUNNING_FLAG"; }
trap clear_running EXIT INT TERM
echo "$(date -Is) batch=$BATCH specs=$SPECS pid=$$" >> "$GUARD_LOG"

# ---- 串行护栏 ---------------------------------------------------------------
# 只允许一个 SIH：① 每个场景前 flock 等构建锁；② 任何**不属于本脚本进程树**的
# SIH 专属进程，或已被占用的 UDP 14580，都视为"别的 SIH 还在跑"，fail-closed 退出
# （不抢占、不清理别人的进程）。
#
# 两个口径分开（两者都逐次写进 GUARD.txt 留证）：
#   * 广口径 = 硬约束要求的 `ps -eo pid,cmd | grep -E 'px4|ros2|sih_record|run_sih|
#     ego_planner|traj_server|lifecycle'`；它也会命中**别的成员跑脱机检查**时
#     cmdline 里的 `ros2_ws` 路径，那和"别的 SIH 在跑"不是一回事，因此只留证不阻塞。
#   * 阻塞口径 = SIH 专属签名（本机 SIH 的 px4 可执行文件、SIH 脚本、安装树里的
#     ROS 可执行文件、ros2 launch/topic/service/run）。命中即拒绝启动。
# 判据用"从候选进程向上走到本脚本 pid 为止"：上一版从自己往下做闭包，会因为祖先
# 一路走到 pid 1 而把被 init 收养的**上一次运行残留**误判成自己的进程树成员（实测漏报）。
BROAD_RE='px4|ros2|sih_record|run_sih|ego_planner|traj_server|lifecycle'
SIH_RE='/PX4-Autopilot/build/px4_sitl_default/bin/px4|tools/sih_record\.py|tools/run_sih_mission\.sh|tools/run_sih_matrix\.sh|px4_sih_mission\.launch\.py|/(depth_node|lifecycle_node|px4_interface_node|stereo_source|synthetic_stereo_source|sitl_truth_source|random_forest_node|mockamap_node|traj_server|ego_planner_node)( |$)|ros2 (launch|topic|service|run|node|param)'
proc_ppid() { sed 's/.*) //' "/proc/$1/stat" 2>/dev/null | awk '{print $2}'; }
in_own_tree() {
  # 判定"候选进程是否属于本脚本自己这一摊"，三种情况都要认，少一种就会误报：
  #   ⓪ 进程已消失 —— ps 快照与判定之间存在竞态，脚本自己的 $( )/管道子 shell 很短命，
  #      刚在快照里出现、再读 /proc 就没了。已消失的进程不可能还在跑 SIH，跳过。
  #   ① 候选就是自己 / 调用本脚本的 shell。
  #   ② 从候选向上走能走到自己或调用者 —— 覆盖自己的后代与管道里的兄弟（如 tee）。
  #   ③ 从自己向上走能走到候选 —— 候选是自己的**祖先**（外层 bash -lc 的 cmdline 里
  #      可能就带着被 grep 的字样）。
  local target="$1" p
  [[ -r "/proc/$target/stat" ]] || return 0
  [[ "$target" == "$SELF_PID" || "$target" == "$$" ]] && return 0
  p="$target"
  for _ in $(seq 1 30); do
    case "$p" in ""|0|1) break;; esac
    p=$(proc_ppid "$p"); [[ -z "$p" ]] && break
    [[ "$p" == "$SELF_PID" || "$p" == "$$" ]] && return 0
  done
  p="$SELF_PID"
  for _ in $(seq 1 30); do
    case "$p" in ""|0|1) break;; esac
    p=$(proc_ppid "$p"); [[ -z "$p" ]] && break
    [[ "$p" == "$target" ]] && return 0
  done
  return 1
}
sih_guard() {
  local stage="$1" blockers="" broad="" pid cmd port n_broad n_block
  broad=$(ps -eo pid=,ppid=,cmd= | grep -E "$BROAD_RE" | grep -v grep || true)
  while read -r pid ppid cmd; do
    [[ -z "${pid// }" ]] && continue
    in_own_tree "$pid" && continue
    blockers="$blockers  pid=$pid ppid=$ppid $cmd"$'\n'
  done < <(printf '%s\n' "$broad" | grep -E "$SIH_RE" || true)
  port=$(ss -lunp 2>/dev/null | grep '14580' || true)
  n_broad=0; [[ -n "$broad" ]] && n_broad=$(printf '%s\n' "$broad" | grep -c .)
  n_block=0; [[ -n "$blockers" ]] && n_block=$(printf '%s\n' "$blockers" | grep -c .)
  {
    echo "$(date -Is) GUARD [$stage] broad_matches=$n_broad sih_blockers=$n_block udp14580=$([[ -n "$port" ]] && echo occupied || echo free)"
    if [[ -n "$broad" ]]; then echo "  broad (仅供参考，可能是别的成员的脱机检查):"; printf '%s\n' "$broad" | sed 's/^/    /'; fi
    if [[ -n "$blockers" ]]; then echo "  sih blockers:"; printf '%s\n' "$blockers" | sed 's/^/    /'; fi
  } >> "$GUARD_LOG"
  if [[ -n "$blockers" || -n "$port" ]]; then
    echo "SIH 串行护栏失败（$stage）：拒绝启动（详见 $GUARD_LOG）" >&2
    [[ -n "$blockers" ]] && printf '%s' "$blockers" >&2
    [[ -n "$port" ]] && echo "$port" >&2
    exit 4
  fi
}

while IFS='|' read -r scenario scene gx gy gz fault obl timeout fault_at forest_seed inject_alt when_ready stable_s transient_s mode_detail planner_suspend_s forest_profile; do
  [[ -z "${scenario// }" || "${scenario:0:1}" == "#" ]] && continue
  # 每次运行前：等构建锁（build_all.sh 自带 flock，用同一把锁），再查残留。
  flock -w 3600 /home/waterc/bb_build/build.lock true || {
    echo "构建锁 3600s 内未取得：拒绝启动 $scenario" >&2; exit 3; }
  sih_guard "before:$scenario"
  mark_running "$scenario"
  echo "=== [$BATCH] $scenario (fault=${fault} obl=${obl} inject_alt=${inject_alt:-none}) ==="
  BB_SIH_EVID="$ROOT_EVID/$BATCH/$scenario" \
  timeout $(( timeout + 300 )) bash companion/ros2_ws/tools/run_sih_mission.sh \
    --scenario "$scenario" --scene "$scene" --goal "$gx" "$gy" "$gz" \
    --fault "$fault" --obl-action "$obl" ${fault_at:+--fault-at "$fault_at"} ${forest_seed:+--forest-seed "$forest_seed"} ${inject_alt:+--inject-alt-below "$inject_alt"} ${when_ready:+--inject-when-ready} \
    ${stable_s:+--inject-stable-s "$stable_s"} ${transient_s:+--transient-relaunch-s "$transient_s"} \
    ${mode_detail:+--inject-mode-detail "$mode_detail"} ${planner_suspend_s:+--planner-suspend-s "$planner_suspend_s"} ${forest_profile:+--forest-profile "$forest_profile"} --timeout "$timeout" \
    > "$ROOT_EVID/$BATCH/$scenario.run.log" 2>&1
  RC=$?
  echo "$RC" > "$ROOT_EVID/$BATCH/$scenario/run_exit.txt"
  echo "  run exit=$RC"
  RANGE=$("$BOOM_BIRDS_VENV/bin/python" -c "
import sys; sys.path.insert(0, 'companion/ros2_ws/src/boom_birds_control')
from boom_birds_control.runtime_config import SCENES
print(SCENES['$scene'].depth_max_range_m)" 2>/dev/null || true)
  "$BOOM_BIRDS_VENV/bin/python" companion/ros2_ws/tools/sih_matrix_row.py \
    "$ROOT_EVID/$BATCH/$scenario" "$scenario" "$fault" "$obl" "$RANGE" >> "$SUMMARY" 2>>"$ROOT_EVID/$BATCH/summary.err" \
    || echo "{\"scenario\": \"$scenario\", \"summary_error\": true}" >> "$SUMMARY"
done < "$SPECS"

clear_running
sih_guard "after-batch"
echo "=== 汇总（$SUMMARY）==="
cat "$SUMMARY"
