#!/usr/bin/env bash
# 本机 PX4 SIH 完整任务入口（TEST-ONLY）。只启动本机 SIH，不连任何硬件。
#
# 用法：
#   bash companion/ros2_ws/tools/run_sih_mission.sh --scenario <名字> --scene <local|forest_30m> \
#        --goal X Y Z [--timeout S] [--fault <规划取消|深度断流|里程计断流|setpoint中断>]
#
# 行为：
#   1) 生成合成标定，后台启动 PX4 SIH（setsid，脱离父 shell）；
#   2) 用 px4_sih_mission.launch.py 启动合成双目→深度→EGO→px4_interface→lifecycle；
#   3) 调 Mission.START 发起任务，周期记录 mission/status 与 PX4 回读；
#   4) 结束条件：COMPLETE / FAULT_LATCHED / 超时；超时则请求 LAND；
#   5) 无论如何都核对最终 Disarmed，并 kill 全部进程。
#
# 证据目录：$BB_SIH_EVID（默认 architecture/evidence/deepseek-01/sih/<scenario>）
# 注意：**不能**用 `set -u`。ROS 2 的 setup.bash 与 nounset 不兼容（见
# tools/activate_python_env.sh 的说明），开了 nounset 会在 source 阶段直接中止。
set -o pipefail
# flock 父进程持锁，关闭传给任务的锁描述符，避免后台 sleep 继承锁。
if [[ "${1:-}" == "--lock-child" && "$(cat /proc/$PPID/comm 2>/dev/null)" == "flock" ]]; then
  shift
else
  mkdir -p "$HOME/bb_build"
  exec flock -n -E 4 --close "$HOME/bb_build/sih.lock" bash "${BASH_SOURCE[0]}" --lock-child "$@"
fi
# 自己的进程 id 用 BASHPID 在顶层取一次：`$$` 在子 shell 里仍是调用者的 pid，
# cleanup 要按"父进程是我"精确收子 shell，用错 id 会误杀调用方。
SELF_PID="$BASHPID"
BB_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$BB_PROJECT_ROOT"

SCENARIO=""; SCENE="local"; GOAL=(); TIMEOUT=180; FAULT=""; OBL_ACTION="land"; FAULT_AT="EXECUTING"; FOREST_SEED="1"; INJECT_ALT_BELOW=""
INJECT_WHEN_READY="0"; INJECT_STABLE_S="0.4"; INJECT_MODE_DETAIL="auto:mission"
MODE_INJECT_WINDOW_S="8"; TRANSIENT_RELAUNCH_S="2"; PLANNER_SUSPEND_S="2"
FOREST_PROFILE="reference_30m"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario) SCENARIO="$2"; shift 2;;
    --scene)    SCENE="$2"; shift 2;;
    --goal)     GOAL=("$2" "$3" "$4"); shift 4;;
    --timeout)  TIMEOUT="$2"; shift 2;;
    --fault)    FAULT="$2"; shift 2;;
    --obl-action) OBL_ACTION="$2"; shift 2;;
    --fault-at)  FAULT_AT="$2"; shift 2;;
    --forest-seed) FOREST_SEED="$2"; shift 2;;
    --forest-profile) FOREST_PROFILE="$2"; shift 2;;
    --inject-alt-below) INJECT_ALT_BELOW="$2"; shift 2;;
    --inject-when-ready) INJECT_WHEN_READY="1"; shift;;
    --inject-stable-s) INJECT_STABLE_S="$2"; shift 2;;
    --inject-mode-detail) INJECT_MODE_DETAIL="$2"; shift 2;;
    --mode-inject-window-s) MODE_INJECT_WINDOW_S="$2"; shift 2;;
    --transient-relaunch-s) TRANSIENT_RELAUNCH_S="$2"; shift 2;;
    --planner-suspend-s) PLANNER_SUSPEND_S="$2"; shift 2;;
    *) echo "未知参数：$1" >&2; exit 2;;
  esac
done
[[ "$OBL_ACTION" == "land" || "$OBL_ACTION" == "rtl" ]] || { echo "--obl-action 只接受 land/rtl" >&2; exit 2; }
case "$FAULT" in
  ""|none|规划取消|深度断流|里程计断流|setpoint中断|模式确认失败|setpoint中断瞬态|里程计断流瞬态|模式码注入|Offboard中断瞬态|恢复模式拒绝|深度挂起|里程计挂起|规划器挂起|规划器重启|人工取消|飞控重启|飞控重启恢复|编排器重启|相机断流) ;;
  *) echo "未知故障：$FAULT" >&2; exit 2;;
esac
[[ "$FAULT" == "none" ]] && FAULT=""
[[ "$TIMEOUT" =~ ^[1-9][0-9]*$ ]] || { echo "--timeout 必须是正整数" >&2; exit 2; }
[[ -n "$SCENARIO" ]] || { echo "缺少 --scenario" >&2; exit 2; }
[[ ${#GOAL[@]} -eq 3 ]] || { echo "缺少 --goal X Y Z" >&2; exit 2; }

export BOOM_BIRDS_VENV="${BOOM_BIRDS_VENV:-/home/waterc/bb_build/architecture/venv}"
export BUILD_BASE="${BUILD_BASE:-/home/waterc/bb_build/architecture/build}"
export INSTALL_BASE="${INSTALL_BASE:-/home/waterc/bb_build/architecture/install}"
export LOG_BASE="${LOG_BASE:-/home/waterc/bb_build/architecture/log}"
export OV_INSTALL="${OV_INSTALL:-/home/waterc/bb_build/ov/install}"
EVID="${BB_SIH_EVID:-/home/waterc/bb_build/architecture/evidence/deepseek-01/sih/$SCENARIO}"
[[ ! -e "$EVID" ]] || { echo "证据目录已存在，拒绝覆盖：$EVID" >&2; exit 2; }
mkdir -p "$EVID"
echo "$SCENARIO scene=$SCENE goal=${GOAL[*]} timeout=$TIMEOUT fault=${FAULT:-none} obl_action=$OBL_ACTION fault_at=$FAULT_AT forest_seed=$FOREST_SEED forest_profile=$FOREST_PROFILE inject_alt_below=${INJECT_ALT_BELOW:-none} inject_when_ready=$INJECT_WHEN_READY inject_stable_s=$INJECT_STABLE_S transient_relaunch_s=$TRANSIENT_RELAUNCH_S planner_suspend_s=$PLANNER_SUSPEND_S inject_mode_detail=$INJECT_MODE_DETAIL" > "$EVID/run.args"
STAGE="$EVID/stage.log"
stage() { echo "$(date +%H:%M:%S) $*" >> "$STAGE"; }
stage "start"

python3 - <<'GUARD' > "$EVID/preflight.txt" 2>&1 || exit 4
from pathlib import Path
import os, sys
names = {"px4", "ego_planner_node", "traj_server", "depth_node", "lifecycle_node",
         "px4_interface_node", "sitl_truth_source", "sitl_hold_relay", "stereo_source", "synthetic_stereo_source", "sih_record.py"}
blockers = []
for path in Path("/proc").iterdir():
    if not path.name.isdigit(): continue
    try:
        argv = path.joinpath("cmdline").read_bytes().split(b"\0")
        if any(os.path.basename(os.fsdecode(a)) in names for a in argv[:2] if a):
            blockers.append((path.name, [os.fsdecode(a) for a in argv if a]))
    except (OSError, ProcessLookupError): pass
print("existing_simulation_processes:", blockers)
if blockers: sys.exit("拒绝启动，不清理其他进程")
GUARD
if ss -lunp 2>/dev/null | grep -q ':14580 '; then
  echo "UDP 14580 已占用，拒绝启动" >> "$EVID/preflight.txt"; exit 4
fi
stage "activate"; source companion/ros2_ws/tools/activate_python_env.sh > "$EVID/env.log" 2>&1 || exit 2
stage "source-install"; source "${INSTALL_BASE}/setup.bash" >> "$EVID/env.log" 2>&1 || exit 2
if [[ -f "${OV_INSTALL}/local_setup.bash" ]]; then source "${OV_INSTALL}/local_setup.bash" >> "$EVID/env.log" 2>&1; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-194}"
export ROS_LOCALHOST_ONLY=1
export MAVLINK20=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

PX4_SOURCE="${PX4_SOURCE:-$HOME/PX4-Autopilot}"
PX4_BUILD="${PX4_BUILD:-$PX4_SOURCE/build/px4_sitl_default}"
PX4_BIN="$PX4_BUILD/bin"
PY="$BOOM_BIRDS_VENV/bin/python"
"$PY" companion/ros2_ws/tools/sih_manifest.py "$EVID/source-environment.json" --scene "$SCENE" --forest-profile "$FOREST_PROFILE" >> "$EVID/env.log" 2>&1 || exit 2

LAUNCH_PID=""; PX4_PID=""; REC_PID=""
# 全局看门狗：任何内部环节卡住都不能让本脚本无限期挂着。
WATCHDOG_S="${BB_SIH_WATCHDOG_S:-$((TIMEOUT + 300))}"
# 看门狗放在独立会话里：它不能成为本脚本退出的阻碍，也不能把 SIGTERM 打到本进程组。
setsid bash -c "sleep ${WATCHDOG_S}; echo '看门狗触发（${WATCHDOG_S}s）：强制结束' >&2; kill -TERM ${SELF_PID}" \
  > /dev/null 2>&1 < /dev/null &
WATCHDOG_PID=$!
REC_NODE_PID=""
cleanup() {
  # 先摘掉 EXIT trap：cleanup 内部会 kill/pkill，若再触发一次 EXIT 会重入。
  trap - EXIT INT TERM
  kill -TERM "-$WATCHDOG_PID" 2>/dev/null
  for p in "$REC_NODE_PID" "$REC_PID" "$LAUNCH_PID"; do
    [[ -n "$p" ]] && kill -TERM "-$p" 2>/dev/null
  done
  sleep 3
  for p in "$REC_PID" "$LAUNCH_PID"; do
    [[ -n "$p" ]] && kill -KILL "-$p" 2>/dev/null
  done
  if [[ -n "$PX4_PID" ]]; then
    kill -TERM "$PX4_PID" 2>/dev/null
    for _ in $(seq 1 15); do kill -0 "$PX4_PID" 2>/dev/null || break; sleep 1; done
    kill -KILL "$PX4_PID" 2>/dev/null
  fi
  for p in "$REC_NODE_PID" "$REC_PID" "$CAMINFO_PID" "$PLANNER_WATCH_PID"; do
    if [[ -n "$p" ]]; then
      pkill -TERM -P "$p" 2>/dev/null
      kill -TERM "$p" 2>/dev/null
    fi
  done
  # 本脚本 fork 出来的子 shell（记录器轮询循环 `( while true ... ) &`）cmdline 与本
  # 脚本相同、又未必是进程组组长，`kill -- -PID` 对它们无效 ⇒ 会残留到下一次运行
  # （第 17 轮实测：残留的子 shell 让下一次运行的串行护栏正确地拒绝启动）。
  # 这里按"父进程是本脚本"精确收尾：SELF_PID 而不是 $$。
  pkill -P "$SELF_PID" 2>/dev/null
  sleep 1
  pkill -9 -P "$SELF_PID" 2>/dev/null
  # cleanup 之后的自检留证：把广口径 ps 与 UDP 端口写成证据（排除本脚本自身那一行）。
  {
    echo "# cleanup 后残留检查（广口径；本脚本自身 pid=$SELF_PID 已排除）"
    ps -eo pid=,ppid=,cmd= | grep -E 'px4|ros2|sih_record|run_sih|ego_planner|traj_server|lifecycle' \
      | grep -v grep | grep -v "^ *$SELF_PID " || echo NONE
    ss -lunp 2>/dev/null | grep '14580' || echo 'udp14580 free'
  } > "$EVID/leftover_after_cleanup.txt" 2>&1
}
trap cleanup EXIT
abort_run() {
  trap - INT TERM
  echo "signal_abort" > "$EVID/abort.txt"
  if [[ -n "$LAUNCH_PID" && -n "$PX4_PID" ]] && kill -0 "$PX4_PID" 2>/dev/null; then
    timeout 15 "$PY" -m boom_birds_bringup.lifecycle_cli land > "$EVID/abort-land.json" 2>&1 || true
    for _ in $(seq 1 30); do
      ( cd "$PX4_BUILD/rootfs/0" && timeout 2 "$PX4_BIN/px4-commander" status ) > "$EVID/abort-status.txt" 2>&1
      if grep -q 'Disarmed' "$EVID/abort-status.txt"; then
        echo "abort_disarmed=yes" >> "$EVID/abort.txt"; break
      fi
      sleep 1
    done
  fi
  exit 130
}
trap abort_run INT TERM

stage "calibration"
# ---- 1) 合成标定 ----
mkdir -p /tmp/boom_birds_synth
"$PY" -m boom_birds_sim.synthetic --write-calibration /tmp/boom_birds_synth/synthetic_candidate.npz \
  > "$EVID/calibration.log" 2>&1

# ---- 2) PX4 SIH ----
cd "$PX4_SOURCE"
setsid env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim PX4_SYS_AUTOSTART=10040 \
  "$PX4_BIN/px4" -d -i 0 > "$EVID/px4.log" 2>&1 < /dev/null &
PX4_PID=$!
echo "$PX4_PID" > "$EVID/px4.pid"
for _ in $(seq 1 30); do
  ss -lunp 2>/dev/null | grep -q '14580' && break
  sleep 1
done
sleep 5
"$PY" - "$PX4_PID" > "$EVID/sih_guard.txt" 2>&1 <<'PYEOF'
import sys
sys.path.insert(0, "/home/waterc/workspace/Boom_Birds/companion/ros2_ws/src/boom_birds_control")
from boom_birds_control.sih_guard import verify_sih_process
pid = int(sys.argv[1])
print("pid", pid, "verify_sih_process", verify_sih_process(pid))
PYEOF
grep -q 'verify_sih_process True' "$EVID/sih_guard.txt" || { echo "SIH 进程守卫未通过" >&2; cat "$EVID/sih_guard.txt" >&2; exit 1; }

stage "chain-launch"
# 下发并记录 PX4 参数基线：数值只有一个定义点（RuntimeConfig / sih_params），
# 本脚本不写任何参数数值。rootfs 的 parameters.bson 会跨运行残留，所以每次都显式
# 下发 + 回读留证；`COM_OF_LOSS_T` 固定为本地版本缺省，**不放大**。
{
  echo "# 本次将下发的 PX4 参数（来源：RuntimeConfig / sih_params）"
  ( cd "$PX4_BUILD/rootfs/0" && "$PY" -m boom_birds_bringup.sih_params --scene "$SCENE" --format json --obl-action "$OBL_ACTION" ) 2>&1 || exit 2
  echo "# 下发"
  ( cd "$PX4_BUILD/rootfs/0" && "$PY" -m boom_birds_bringup.sih_params --scene "$SCENE" --obl-action "$OBL_ACTION" --apply \
      --param-bin "$PX4_BIN/px4-param" ) 2>&1 || exit 2
  echo "# 回读"
  ( cd "$PX4_BUILD/rootfs/0" && "$PY" -m boom_birds_bringup.sih_params --scene "$SCENE" --obl-action "$OBL_ACTION" --readback \
      --param-bin "$PX4_BIN/px4-param" ) 2>&1 || exit 2
} > "$EVID/px4_params.txt" 2>&1

# ---- 3) 任务链 ----
cd "$BB_PROJECT_ROOT"
setsid ros2 launch boom_birds_nav px4_sih_mission.launch.py \
  "scene:=$SCENE" "sih_pid:=$PX4_PID" "forest_seed:=$FOREST_SEED" "forest_profile:=$FOREST_PROFILE" "goal_z:=${GOAL[2]}" \
  > "$EVID/launch.log" 2>&1 < /dev/null &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$EVID/launch.pid"

# ---- 3b) CameraInfo 监看器 ----
# 为什么需要：EGO 的 [BB-A3] 相机几何闭锁只报"内参或尺寸变化"，而 grid_map 的
# sameCameraGeometry() 只比较 width/height/frame_id/P。要回答"到底是哪个字段变、
# 何时变、谁发布"，必须把**每一条** CameraInfo 原文与发布者 GID 落盘。
# 只订阅、不发布，5 Hz 量级旁路观测，不写入被测链路。
setsid "$PY" - "$EVID" > "$EVID/caminfo_watch.log" 2>&1 < /dev/null <<'PYEOF' &
import json, sys, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo

out = Path(sys.argv[1]) / "caminfo_watch.jsonl"
rclpy.init()
node = Node("bb_caminfo_watch")
qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST)


def on_info(msg):
    try:
        pubs = node.get_publishers_info_by_topic("/boom_birds/depth/camera_info")
        rows = sorted(str(x.node_name) + "#" + bytes(x.endpoint_gid).hex()[:12] for x in pubs)
    except Exception as exc:
        rows = ["err:" + str(exc)]
    rec = {"t": time.time(), "width": int(msg.width), "height": int(msg.height),
           "frame_id": str(msg.header.frame_id), "p": [round(float(v), 6) for v in msg.p],
           "k": [round(float(v), 6) for v in msg.k],
           "distortion_model": str(msg.distortion_model),
           "n_publishers": len(rows), "publishers": rows}
    with open(out, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(rec, ensure_ascii=False) + "\n")


node.create_subscription(CameraInfo, "/boom_birds/depth/camera_info", on_info, qos)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
PYEOF
CAMINFO_PID=$!
echo "$CAMINFO_PID" >> "$EVID/recorder.pid"

# ---- 3c) planner 观测器（第 1 项量测的唯一时钟源） ----
# 只订阅，不发布。逐条记录 /boom_birds/planner/{request,status}，并对
# /boom_birds/planner/command 与 /boom_birds/control/execution_status 只在
# (session, trajectory_id[, sending/mode]) **变化时**记一条 —— 既拿到"新 trajectory id
# 首次出现"的到达时刻，又不给 100 Hz 的命令流加无谓写盘。另每 5 s 一条心跳计数，
# 用来回答"规划器还在不在发布"。
# 到达时刻用本进程 time.time()，四路同一时钟；与 recorder 的 mission/execution 时间线
# 交叉核对，EGO 侧用 launch.log 的 traj/plan 行核对。
setsid "$PY" - "$EVID" > "$EVID/planner_watch.log" 2>&1 < /dev/null <<'PYEOF' &
import json, sys, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from boom_birds_interfaces.msg import ControlCommand, ExecutionStatus, PlannerRequest, PlannerStatus

out = Path(sys.argv[1]) / "planner_watch.jsonl"
rclpy.init()
node = Node("bb_planner_watch")
qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
fh = open(out, "a", encoding="utf-8")
last = {"command": None, "execution": None}
counts = {"command": 0, "execution": 0, "request": 0, "planner_status": 0}
last_cmd_t = [None]


def emit(kind, **kw):
    kw.update(t=time.time(), kind=kind)
    counts[kind] = counts.get(kind, 0) + 1
    fh.write(json.dumps(kw, ensure_ascii=False) + "\n")
    fh.flush()


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def on_request(m):
    emit("request", session=m.session_id, sequence=int(m.sequence), enabled=bool(m.enabled),
         goal=[m.goal.x, m.goal.y, m.goal.z], stamp=stamp(m))


def on_planner_status(m):
    emit("planner_status", session=m.session_id, map_ready=bool(m.map_ready),
         geometry_fault=bool(m.geometry_fault), goal_active=bool(m.goal_active), stamp=stamp(m))


def on_command(m):
    counts["command"] += 1
    last_cmd_t[0] = time.time()
    key = (m.session_id, int(m.trajectory_id))
    if key == last["command"]:
        return
    last["command"] = key
    emit("command", session=m.session_id, trajectory_id=int(m.trajectory_id), sequence=int(m.sequence),
         command_type=int(m.command_type), position=[m.position.x, m.position.y, m.position.z],
         velocity=[m.velocity.x, m.velocity.y, m.velocity.z], stamp=stamp(m))


def on_exec(m):
    counts["execution"] += 1
    key = (m.session_id, int(m.trajectory_id), bool(m.sending), str(m.mode_detail), bool(m.accepted))
    if key == last["execution"]:
        return
    last["execution"] = key
    emit("execution", session=m.session_id, trajectory_id=int(m.trajectory_id), sequence=int(m.sequence),
         sending=bool(m.sending), accepted=bool(m.accepted), mode_detail=str(m.mode_detail),
         offboard_confirmed=bool(m.offboard_confirmed), armed=bool(m.armed),
         position_ned=[m.position_ned.x, m.position_ned.y, m.position_ned.z],
         velocity_ned=[m.velocity_ned.x, m.velocity_ned.y, m.velocity_ned.z],
         reasons=list(m.reasons), stamp=stamp(m))


def heartbeat():
    now = time.time()
    emit("heartbeat", counts=dict(counts), last_command_age_s=(None if last_cmd_t[0] is None else round(now - last_cmd_t[0], 3)),
         last_command_trajectory=(None if last["command"] is None else last["command"][1]))


node.create_subscription(PlannerRequest, "/boom_birds/planner/request", on_request, qos)
node.create_subscription(PlannerStatus, "/boom_birds/planner/status", on_planner_status, qos)
node.create_subscription(ControlCommand, "/boom_birds/planner/command", on_command, qos)
node.create_subscription(ExecutionStatus, "/boom_birds/control/execution_status", on_exec, qos)
node.create_timer(5.0, heartbeat)
rclpy.spin(node)
PYEOF
PLANNER_WATCH_PID=$!
echo "$PLANNER_WATCH_PID" >> "$EVID/recorder.pid"

# ---- 4) 记录器：完整 JSONL（mission/control/execution）+ PX4 回读 ----
setsid "$PY" "$BB_PROJECT_ROOT/companion/ros2_ws/tools/sih_record.py" "$EVID" \
  > "$EVID/recorder_node.log" 2>&1 < /dev/null &
REC_NODE_PID=$!
# 降载：早先这里每秒 spawn 3 个 px4-listener，进程创建抖动会打断 ROS 侧
# 位姿/命令流（px4_failsafe 的 VIO 超时 0.15 s、迟滞 5 次），反而把被测系统
# 打出假的"链路断流"。时间线证据改由 ExecutionStatus 记录器提供（它已含
# 位置/速度/模式/子模式/落地），这里只做**低频独立回读**，用于交叉核对。
(
  cd "$PX4_BUILD/rootfs/0"
  while true; do
    TS=$(date +%s.%N)
    POS=$(timeout 8 "$PX4_BIN/px4-listener" vehicle_local_position -n 1 2>/dev/null | tr '\n' ' ')
    printf '%s\t%s\n' "$TS" "$POS" >> "$EVID/recorder_px4.jsonl"
    SP=$(timeout 8 "$PX4_BIN/px4-listener" trajectory_setpoint -n 1 2>/dev/null | tr '\n' ' ')
    printf '%s\t%s\n' "$TS" "$SP" >> "$EVID/recorder_px4_setpoint.txt"
    sleep 3
  done
) > /dev/null 2>&1 < /dev/null &
REC_PID=$!
echo "$REC_PID $REC_NODE_PID" > "$EVID/recorder.pid"

# 等任务服务
for _ in $(seq 1 60); do
  timeout 10 ros2 service list 2>/dev/null | grep -q '/boom_birds/mission$' && break
  sleep 1
done

stage "graph-diag"
# 任务开始前一次性采样 ROS 图：上一轮 forest 运行出现 EGO 相机几何闭锁
# （"内参或尺寸变化"），需要知道 CameraInfo 到底有几个发布者。只采一次、在任务开始前，
# 不按周期重复，避免测量本身扰动被测系统。
{
  echo "# ros2 topic info -v /boom_birds/depth/camera_info"
  timeout 20 ros2 topic info -v /boom_birds/depth/camera_info 2>&1
  echo "# ros2 topic info -v /boom_birds/depth/image"
  timeout 20 ros2 topic info -v /boom_birds/depth/image 2>&1
  echo "# ros2 node list"
  timeout 20 ros2 node list 2>&1
} > "$EVID/ros_graph.txt" 2>&1
if [[ "$FAULT" == "模式确认失败" ]]; then
  timeout 10 ros2 param set /boom_birds_px4_interface sih_reject_offboard true > "$EVID/fault.txt" 2>&1 || exit 1
  date +%s.%N > "$EVID/fault_injected_at.txt"
fi
if [[ "$SCENE" == "forest_30m" ]]; then
  stage "scene-reachability"
  timeout 20 ros2 param dump /drone_0_ego_planner_node > "$EVID/planner_params.yaml" 2> "$EVID/planner_params.err" || exit 2
  for _ in $(seq 1 30); do
    [[ -f "$EVID/scene_cloud.npy" ]] && break
    sleep 1
  done
  "$PY" -m boom_birds_sim.scene_reachability \
    --cloud "$EVID/scene_cloud.npy" --planner-parameters "$EVID/planner_params.yaml" \
    --scene "$SCENE" --goal "${GOAL[@]}" --out "$EVID/scene_reachability.json" \
    > "$EVID/scene_reachability.log" 2>&1
  ORACLE_RC=$?
  if [[ "$ORACLE_RC" != 0 ]]; then
    # 未发 Mission.START，拒绝在非法场景中解锁；仍独立核对 PX4 最终状态。
    ( cd "$PX4_BUILD/rootfs/0" && timeout 5 "$PX4_BIN/px4-commander" status ) > "$EVID/final_commander_status.txt" 2>&1
    grep -q 'Disarmed' "$EVID/final_commander_status.txt" || exit 1
    printf 'scene_preflight_rejected disarmed=yes\n' > "$EVID/final_state.txt"
    stage "scene-rejected rc=$ORACLE_RC"
    cat "$EVID/scene_reachability.log"
    exit "$ORACLE_RC"
  fi
fi
stage "mission-start"
echo "--- Mission.START goal=${GOAL[*]} ---"
timeout 30 "$PY" -m boom_birds_bringup.lifecycle_cli start --goal "${GOAL[@]}" > "$EVID/start.json" 2>&1
cat "$EVID/start.json"

# ---- 5) 等 EXECUTING，再等结束 ----
WAIT_STATE="${FAULT_AT:-EXECUTING}"
[[ "$FAULT" == "模式确认失败" ]] && WAIT_STATE="FAULT_LATCHED"
timeout 120 "$PY" -m boom_birds_bringup.lifecycle_cli status --wait-state "$WAIT_STATE" --timeout 90 \
  > "$EVID/wait_state.json" 2>&1
echo "WAIT(${WAIT_STATE}): $(cat "$EVID/wait_state.json")"
if [[ "$WAIT_STATE" != "EXECUTING" && "$WAIT_STATE" != "COMPLETE" && -z "$INJECT_ALT_BELOW" && "$FAULT" != "模式确认失败" ]]; then
  # 反向用例要在更早阶段注入（例如低高度），这里再等一小段让状态稳定。
  timeout 90 "$PY" -m boom_birds_bringup.lifecycle_cli status --wait-state EXECUTING --timeout 60 \
    >> "$EVID/wait_state.json" 2>&1 || true
fi

# ---- 低空门限：注入时机必须由**遥测高度**证明 ----
# `--inject-alt-below X` 时轮询记录器里的 ExecutionStatus（PX4 回读位置，NED），
# 以任务开始前的地面读数为 AGL 基准，等到 AGL < X 的那一帧才注入；注入瞬间的
# 原始样本写入 fault_telemetry.json。始终没等到就**不注入**，并在 fault.txt 里
# 记 NOT RUN——不允许用"等状态"碰运气代替高度证据。
INJECT_GATE_OK=""
NEED_GATE=0
[[ -n "$INJECT_ALT_BELOW" ]] && NEED_GATE=1
[[ "$INJECT_WHEN_READY" == "1" ]] && NEED_GATE=1
if [[ -n "$FAULT" && "$NEED_GATE" == "1" ]]; then
  INJECT_GATE_OK=$("$PY" - "$EVID" "${INJECT_ALT_BELOW:-}" "$INJECT_WHEN_READY" "$INJECT_STABLE_S" <<'PYEOF'
import json, math, sys, time
from pathlib import Path
sys.path.insert(0, "/home/waterc/workspace/Boom_Birds/companion/ros2_ws/src/boom_birds_control")
from boom_birds_control.runtime_config import RuntimeConfig

evid = Path(sys.argv[1])
alt_limit = float(sys.argv[2]) if sys.argv[2].strip() else None
require_ready = sys.argv[3] == "1"
stable_s = float(sys.argv[4])
cfg = RuntimeConfig()
max_speed = cfg.recovery_max_speed_m_s
deadline = time.time() + 90.0
ground = None
ok_since = None
best = None
samples = []
history = []
while time.time() < deadline:
    ex = []
    f = evid / "recorder_execution.jsonl"
    if f.is_file():
        for line in f.read_text(errors="replace").splitlines():
            try:
                ex.append(json.loads(line))
            except Exception:
                pass
    mission = {}
    fm = evid / "recorder_mission.jsonl"
    if fm.is_file():
        for line in fm.read_text(errors="replace").splitlines():
            try:
                mission = json.loads(line).get("data") or {}
            except Exception:
                pass
    if not ex:
        time.sleep(0.15)
        continue
    e = ex[-1].get("data") or {}
    if not e.get("position_known"):
        time.sleep(0.15)
        continue
    z = e["position_ned"][2]
    if ground is None:
        # AGL 基准必须是**起飞前**的地面读数：门是在到达 EXECUTING 之后才开始轮询的，
        # 那时飞行器已经在 1.5 m 空中；拿"第一条看到的样本"当地面，会把 1.5 m 读成 AGL≈0
        # （第 1 版实测踩到：注入时 snapshot 里 ground_z_ned=-1.486、agl_m=0.0）。
        for r0 in ex:
            d0 = r0.get("data") or {}
            if d0.get("position_known") and not d0.get("armed"):
                ground = d0["position_ned"][2]
                break
    agl = ground - z if ground is not None else float("nan")
    v = e.get("velocity_ned") or [0.0, 0.0, 0.0]
    speed = math.hypot(*v)
    sample = {"t": ex[-1].get("t"), "mode_detail": e.get("mode_detail"), "speed_m_s": round(speed, 4),
              "agl_m": round(agl, 3), "state": mission.get("state"), "reason": mission.get("reason"),
              "sending": e.get("sending"), "armed": e.get("armed"),
              "trajectory_id": e.get("trajectory_id")}
    samples.append(sample)
    samples = samples[-600:]
    why = []
    if require_ready:
        if e.get("mode_detail") != "offboard":
            why.append("mode_detail!=offboard")
        if speed >= max_speed:
            why.append("speed>=%.3f" % max_speed)
        if mission.get("state") != "EXECUTING":
            why.append("state!=EXECUTING")
        if mission.get("reason"):
            why.append("latch:" + str(mission.get("reason")))
    if alt_limit is not None and agl >= alt_limit:
        why.append("agl>=%.3f" % alt_limit)
    if not why:
        if ok_since is None:
            ok_since = time.time()
        held = time.time() - ok_since
        if held >= stable_s:
            best = dict(sample)
            best["held_s"] = round(held, 3)
            break
    else:
        ok_since = None
    history.append({"why": why, "t": sample["t"], "speed_m_s": sample["speed_m_s"],
                    "agl_m": sample["agl_m"], "mode_detail": sample["mode_detail"],
                    "state": sample["state"]})
    history = history[-40:]
    time.sleep(0.1)
out = {"require_ready": require_ready, "handoff_max_speed_m_s": max_speed,
       "alt_limit_m": alt_limit, "stable_required_s": stable_s,
       "ground_z_ned": ground, "ground_source": "首次 position_known 且未 armed 的样本（起飞前地面）",
       "trigger": best, "n_samples": len(samples),
       "history_tail": history[-8:], "samples_tail": samples[-8:]}
(evid / "trigger_snapshot.json").write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n")
print("yes" if best else "no")
PYEOF
)
  echo "INJECT_GATE(ready=$INJECT_WHEN_READY alt=${INJECT_ALT_BELOW:-none} stable=${INJECT_STABLE_S}s) -> $INJECT_GATE_OK"
fi

if [[ -n "$FAULT" && "$NEED_GATE" == "1" && "$INJECT_GATE_OK" != "yes" ]]; then
  echo "NOT RUN：注入前置条件（offboard/速度<交接门/无 latch/高度门限）180s 内未持续成立，未注入故障" > "$EVID/fault.txt"
  cat "$EVID/fault.txt"
  FAULT=""
fi

if [[ -n "$FAULT" ]]; then
  echo "--- 故障注入：$FAULT ---"
  # 只有"固定时刻注入"才需要这 8 s 稳定等待；状态条件触发已经在门里证明了条件成立，
  # 再等 8 s 会让条件过期（第 1 批 t2/t3 就是这样在注入前先踩到 handoff 闭锁）。
  [[ "$NEED_GATE" == "1" || "$FAULT" == "模式确认失败" ]] || sleep 8
  [[ -e "$EVID/fault_injected_at.txt" ]] || date +%s.%N > "$EVID/fault_injected_at.txt"
# 注入"杀进程"类故障的统一入口：**先证明匹配到了进程再杀**，并把命中的 pattern
# 与 pid 写进 fault.txt。第 17 轮实测：拆包后深度节点已不在 boom_birds_nav 下，
# 旧 pattern 匹配不到任何进程 -> pkill 静默失败 -> fault.txt 根本没写，故障其实
# 从未注入，而运行照样"结束得很干净"。没有这层证据就会把"没注入"读成"注入无效"。
owned_pids() {
  local candidate p parent
  while read -r candidate; do
    [[ -n "$candidate" && "$candidate" != "$SELF_PID" ]] || continue
    p="$candidate"
    for _ in $(seq 1 30); do
      [[ -r "/proc/$p/stat" ]] || break
      parent=$(sed 's/.*) //' "/proc/$p/stat" 2>/dev/null | awk '{print $2}')
      if [[ "$parent" == "$LAUNCH_PID" || "$parent" == "$SELF_PID" ]]; then
        echo "$candidate"; break
      fi
      [[ -n "$parent" && "$parent" != 0 && "$parent" != 1 ]] || break
      p="$parent"
    done
  done < <(pgrep -f "$1" 2>/dev/null)
}
kill_label() {
  local label="$1"; shift
  local pat pids left
  for pat in "$@"; do
    pids=$(owned_pids "$pat" | tr '\n' ' ')
    if [[ -n "${pids// /}" ]]; then
      kill -TERM $pids 2>/dev/null
      sleep 1
      left=$(owned_pids "$pat" | tr '\n' ' ')
      if [[ -n "${left// /}" ]]; then kill -KILL $left 2>/dev/null; fi
      printf '%s: pattern=%s killed_pids=%s\n' "$label" "$pat" "$pids" > "$EVID/fault.txt"
      return 0
    fi
  done
  printf '%s: NOT INJECTED — 以下 pattern 都没匹配到进程：%s\n' "$label" "$*" > "$EVID/fault.txt"
  return 1
}
# 瞬态故障：杀 → 等 N 秒 → 用**同一份 argv/cwd**重新拉起。
# 为什么必须瞬态：B 的闭环要求"活着的生产者产出新轨迹"。上一轮杀 traj_server 后不再
# 拉起，于是再使能也不会有新轨迹，30 s 静默后 planner_timeout —— 那是"生产者没了"，
# 不是"预算不够"（`planning_timeout_s` 只是 planner 输出的新鲜度窗口）。
kill_label_transient() {
  local label="$1" delay="$2"; shift 2
  local pat pids left pid argv=() cwd newpid alive
  for pat in "$@"; do
    pids=$(owned_pids "$pat" | tr '\n' ' ')
    [[ -z "${pids// /}" ]] && continue
    pid="${pids%% *}"
    mapfile -d '' -t argv < "/proc/$pid/cmdline" 2>/dev/null || true
    cwd=$(readlink -f "/proc/$pid/cwd" 2>/dev/null)
    if [[ ${#argv[@]} -eq 0 ]]; then
      printf '%s: NOT INJECTED — argv 读取为空（pattern=%s pid=%s）\n' "$label" "$pat" "$pid" > "$EVID/fault.txt"
      return 1
    fi
    kill -TERM $pids 2>/dev/null
    sleep 1
    left=$(owned_pids "$pat" | tr '\n' ' ')
    [[ -n "${left// /}" ]] && kill -KILL $left 2>/dev/null
    sleep "$delay"
    cd "${cwd:-$BB_PROJECT_ROOT}"
    setsid "${argv[@]}" > "$EVID/relaunch.log" 2>&1 < /dev/null &
    newpid=$!
    cd "$BB_PROJECT_ROOT"
    sleep 3
    alive=no; kill -0 "$newpid" 2>/dev/null && alive=yes
    "$PY" - "$label" "$pat" "$pids" "$cwd" "$delay" "$newpid" "$alive" "${argv[@]}" > "$EVID/fault_relaunch.json" <<'PYEOF'
import json, sys
label, pat, pids, cwd, delay, newpid, alive = sys.argv[1:8]
print(json.dumps({"label": label, "pattern": pat, "killed_pids": pids.split(), "cwd": cwd,
                  "relaunch_delay_s": float(delay), "new_pid": int(newpid),
                  "alive_after_3s": alive == "yes", "argv": sys.argv[8:]},
                 ensure_ascii=False, indent=1))
PYEOF
    printf '%s: pattern=%s killed_pids=%s relaunch_delay_s=%s new_pid=%s alive_after_3s=%s\n' \
      "$label" "$pat" "$pids" "$delay" "$newpid" "$alive" > "$EVID/fault.txt"
    return 0
  done
  printf '%s: NOT INJECTED — 以下 pattern 都没匹配到进程：%s\n' "$label" "$*" > "$EVID/fault.txt"
  return 1
}

# 未登记模式码注入：**第二个发布者**改写 /boom_birds/control/execution_status 的
# mode_detail，窗口结束即退出（双发布者只是注入手段，不是生产契约）。
inject_mode_detail() {
  local value="$1" window="$2"
  setsid "$PY" - "$EVID" "$value" "$window" > "$EVID/mode_inject.log" 2>&1 < /dev/null <<'PYEOF' &
import json, sys, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from boom_birds_interfaces.msg import ExecutionStatus

evid, value, window = Path(sys.argv[1]), sys.argv[2], float(sys.argv[3])
rclpy.init()
node = Node("bb_mode_detail_inject")
qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
pub = node.create_publisher(ExecutionStatus, "/boom_birds/control/execution_status", qos)
stat = {"started": None, "republished": 0, "seen": {}, "done": False}


def on_status(msg):
    now = time.time()
    if stat["started"] is None:
        stat["started"] = now
        node.get_logger().warn(
            "INJECTION: 本节点是 /boom_birds/control/execution_status 的第二个发布者，"
            "只用于把 mode_detail 改成 %s；窗口 %.1fs 后退出（注入手段，非生产契约）" % (value, window))
    stat["seen"][str(msg.mode_detail)] = stat["seen"].get(str(msg.mode_detail), 0) + 1
    if now - stat["started"] > window:
        return
    out = ExecutionStatus()
    for name in msg.get_fields_and_field_types():
        if name != "mode_detail":
            setattr(out, name, getattr(msg, name))
    out.mode_detail = value
    pub.publish(out)
    stat["republished"] += 1


def finish():
    if stat["done"] or stat["started"] is None or time.time() - stat["started"] <= window:
        return
    stat["done"] = True
    (evid / "mode_inject.json").write_text(json.dumps(
        {"injected_mode_detail": value, "window_s": window, "republished": stat["republished"],
         "observed_mode_details": stat["seen"],
         "note": "第二发布者=注入手段；超窗后不再改写，节点随即退出"},
        ensure_ascii=False, indent=1) + "\n")
    node.destroy_node()
    rclpy.shutdown()


node.create_subscription(ExecutionStatus, "/boom_birds/control/execution_status", on_status, qos)
node.create_timer(0.2, finish)
rclpy.spin(node)
PYEOF
  echo "$!" > "$EVID/mode_inject.pid"
  sleep 2
  printf '模式码注入: 第二发布者把 mode_detail 改写为 %s（窗口 %ss；双发布者=注入手段，非生产契约）\n' \
    "$value" "$window" > "$EVID/fault.txt"
}


# 规划器挂起：SIGSTOP → 等 N 秒 → SIGCONT（**不 kill、不重启**）。
# 为什么必须挂起而不是 kill+重启：重启会让规划器/轨迹生产者的内部 trajectory_id 从 1
# 重新计数，而 ControlIngress.retired 是**单调水位**（`cmd.trajectory_id <= retired`
# ⇒ `retired_trajectory`，control_protocol.py:68-72），复用旧 id 被永久拒绝；重开会话
# 又会被 lifecycle.py:587-588 判 `session_changed`（在 REVOKING_FAULTS 里）而撤销恢复
# 资格。两条路都通向闭锁降落 ⇒ 本任务内"重启轨迹生产者"不存在合法恢复路径。
# 挂起/恢复保持进程、会话与内部计数不变；故障码是 planning_link（RECOVERABLE_FAULTS），
# 且执行许可（sending/sensors）不受影响 ⇒ PX4 不会因 offboard 断流进入 auto:land。
planner_suspend() {
  local label="$1" seconds="$2"; shift 2
  local pat pids pid st stopped="" resumed="" alive=no
  for pat in "$@"; do
    pids=$(owned_pids "$pat" | tr '\n' ' ')
    [[ -z "${pids// /}" ]] && continue
    kill -STOP $pids 2>/dev/null
    sleep 0.3
    for pid in $pids; do
      st=$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null)
      stopped="$stopped $pid:${st:-?}"
    done
    sleep "$seconds"
    kill -CONT $pids 2>/dev/null
    sleep 0.3
    for pid in $pids; do
      st=$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null)
      resumed="$resumed $pid:${st:-?}"
      kill -0 "$pid" 2>/dev/null && alive=yes
    done
    printf '%s: pattern=%s pids=%s stopped_state=%s suspend_s=%s resumed_state=%s same_pids_alive=%s\n' \
      "$label" "$pat" "$pids" "$stopped" "$seconds" "$resumed" "$alive" > "$EVID/fault.txt"
    printf '{"label": "%s", "pattern": "%s", "pids": "%s", "suspend_s": %s, "stopped_state": "%s", "resumed_state": "%s", "same_pids_alive_after": "%s"}\n' \
      "$label" "$pat" "$pids" "$seconds" "$stopped" "$resumed" "$alive" > "$EVID/fault_suspend.json"
    return 0
  done
  printf '%s: NOT INJECTED — 以下 pattern 都没匹配到进程：%s\n' "$label" "$*" > "$EVID/fault.txt"
  return 1
}

  case "$FAULT" in
    深度断流) kill_label "深度断流" 'boom_birds_nav/depth_node' 'lib/boom_birds_sensing/depth_node' || true;;
    里程计断流) kill_label "里程计断流" 'boom_birds_sitl_truth' 'lib/boom_birds_sim/sitl_truth_source' || true;;
    规划取消) timeout 10 "$PY" companion/ros2_ws/tools/sih_inject_planner_cancel.py --sih-pid "$PX4_PID" > "$EVID/fault.txt" 2>&1;;
    setpoint中断) kill_label "setpoint中断" 'ego_planner/traj_server' 'lib/traj_utils/traj_server' || true;;
    模式确认失败) echo "SIH Offboard rejection enabled before Mission.START" >> "$EVID/fault.txt";;
    setpoint中断瞬态) kill_label_transient "setpoint中断瞬态" "$TRANSIENT_RELAUNCH_S" 'lib/ego_planner/traj_server' 'ego_planner/traj_server' || true;;
    里程计断流瞬态) kill_label_transient "里程计断流瞬态" "$TRANSIENT_RELAUNCH_S" 'lib/boom_birds_sim/sitl_truth_source' 'boom_birds_sitl_truth' || true;;
    模式码注入) inject_mode_detail "$INJECT_MODE_DETAIL" "$MODE_INJECT_WINDOW_S";;
    Offboard中断瞬态)
      timeout 25 "$PY" companion/ros2_ws/tools/sih_inject_setpoint_pause.py --sih-pid "$PX4_PID" --seconds "$TRANSIENT_RELAUNCH_S" > "$EVID/fault.txt" 2>&1
      ;;
    恢复模式拒绝)
      timeout 10 ros2 param set /boom_birds_px4_interface sih_reject_offboard true > "$EVID/fault.txt" 2>&1
      timeout 25 "$PY" companion/ros2_ws/tools/sih_inject_setpoint_pause.py --sih-pid "$PX4_PID" --seconds "$TRANSIENT_RELAUNCH_S" >> "$EVID/fault.txt" 2>&1
      ;;
    深度挂起) planner_suspend "深度挂起" "$PLANNER_SUSPEND_S" 'lib/boom_birds_sensing/depth_node' || true;;
    里程计挂起) planner_suspend "里程计挂起" "$PLANNER_SUSPEND_S" 'lib/boom_birds_sim/sitl_truth_source' || true;;
    规划器重启) kill_label_transient "规划器重启" "$TRANSIENT_RELAUNCH_S" 'lib/ego_planner/ego_planner_node' 'ego_planner_node' || true;;
    规划器挂起) planner_suspend "规划器挂起" "$PLANNER_SUSPEND_S" 'lib/ego_planner/ego_planner_node' 'ego_planner_node' || true;;
    人工取消)   timeout 20 "$PY" -m boom_birds_bringup.lifecycle_cli cancel > "$EVID/fault.txt" 2>&1;;
    飞控重启)   kill -KILL "$PX4_PID" 2>/dev/null; echo "killed PX4 pid=$PX4_PID" > "$EVID/fault.txt";;
    飞控重启恢复)
      # 反向用例：在**同一次运行内**重启 PX4 进程。上一轮直接 kill 后不再拉起，
      # 结果"最终 Disarmed"读不回来（NOT OBSERVABLE）。这里 kill 旧实例、等端口释放，
      # 再用完全相同的 env/实例号拉起新实例，新旧 pid 都留证。
      OLD_PX4_PID="$PX4_PID"
      kill -KILL "$OLD_PX4_PID" 2>/dev/null
      for _ in $(seq 1 20); do kill -0 "$OLD_PX4_PID" 2>/dev/null || break; sleep 0.5; done
      for _ in $(seq 1 30); do ss -lunp 2>/dev/null | grep -q '14580' || break; sleep 1; done
      cd "$PX4_SOURCE"
      setsid env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim PX4_SYS_AUTOSTART=10040 \
        "$PX4_BIN/px4" -d -i 0 > "$EVID/px4_restart.log" 2>&1 < /dev/null &
      NEW_PX4_PID=$!
      PX4_PID="$NEW_PX4_PID"
      echo "$NEW_PX4_PID" > "$EVID/px4_restart.pid"
      for _ in $(seq 1 30); do ss -lunp 2>/dev/null | grep -q '14580' && break; sleep 1; done
      sleep 5
      cd "$BB_PROJECT_ROOT"
      printf 'in-run restart: killed PX4 pid=%s, restarted as pid=%s (same env, -i 0)\n' \
        "$OLD_PX4_PID" "$NEW_PX4_PID" > "$EVID/fault.txt"
      ;;

    编排器重启) kill_label "编排器重启" 'lib/boom_birds_bringup/lifecycle_node' 'lifecycle_node' || true;;
    相机断流)   kill_label "相机断流" 'lib/boom_birds_sim/synthetic_stereo_source' 'lib/boom_birds_sensing/stereo_source' 'stereo_source' || true;;
    *) echo "未知故障 $FAULT" >&2;;
  esac
  cat "$EVID/fault.txt" 2>/dev/null
fi

# 注入/闭锁之后再采一次 ROS 图，用于对照"闭锁前后发布者是否变化"
{
  echo "# post-injection ros2 topic info -v /boom_birds/depth/camera_info ($(date -Is))"
  timeout 20 ros2 topic info -v /boom_birds/depth/camera_info 2>&1
} > "$EVID/ros_graph_after.txt" 2>&1

DEADLINE=$(( $(date +%s) + TIMEOUT ))
FINAL_STATE=""
LAST_ST=""
while [[ $(date +%s) -lt $DEADLINE ]]; do
  ST=$(timeout 10 "$PY" -m boom_birds_bringup.lifecycle_cli status --timeout 5 2>/dev/null | tail -1)
  FINAL_STATE="$ST"
  echo "$(date +%s) $ST" >> "$EVID/state_timeline.log"
  # 状态每次变化都留一份"轨迹生产者是否还活着"的快照：planner_timeout 到底是
  # "重发没生效"还是"没人生产"，只能靠这个分辨。
  CUR=$(printf '%s' "$ST" | sed -n 's/.*"state": "\([A-Z_]*\)".*/\1/p' | head -1)
  if [[ "$CUR" != "$LAST_ST" ]]; then
    LAST_ST="$CUR"
    {
      echo "[$(date -Is)] state=$CUR detail=$(printf '%s' "$ST" | sed -n 's/.*"detail": "\([^"]*\)".*/\1/p' | tail -1)"
      pgrep -af 'traj_server|ego_planner_node|sitl_truth_source|depth_node|lifecycle_node' || echo "  (无生产者进程)"
    } >> "$EVID/producer_timeline.txt"
  fi
  case "$ST" in
    *'"state": "COMPLETE"'*|*'"state": "FAULT_LATCHED"'*) break;;
  esac
  sleep 3
done
echo "FINAL_STATE=$FINAL_STATE" > "$EVID/final_state.txt"

stage "finalize"
{
  echo "[$(date -Is)] final"
  pgrep -af 'traj_server|ego_planner_node|sitl_truth_source|depth_node|lifecycle_node' || echo "  (无生产者进程)"
} >> "$EVID/producer_timeline.txt"
# ---- 6) 收尾：请求降落并核对 Disarmed ----
if [[ "$FINAL_STATE" != *'"state": "COMPLETE"'* ]]; then
  timeout 20 "$PY" -m boom_birds_bringup.lifecycle_cli land > "$EVID/land.json" 2>&1 || true
fi
DISARMED=no
for _ in $(seq 1 60); do
  ( cd "$PX4_BUILD/rootfs/0" && timeout 8 "$PX4_BIN/px4-commander" status 2>/dev/null ) > "$EVID/final_status.txt"
  if grep -qa 'Disarmed' "$EVID/final_status.txt"; then DISARMED=yes; break; fi
  sleep 2
done
echo "disarmed=$DISARMED" >> "$EVID/final_state.txt"
( cd "$PX4_BUILD/rootfs/0" && timeout 8 "$PX4_BIN/px4-listener" vehicle_land_detected -n 1 2>/dev/null ) >> "$EVID/final_state.txt"
cp "$EVID/final_status.txt" "$EVID/final_commander_status.txt" 2>/dev/null
timeout 10 "$PY" -m boom_birds_bringup.lifecycle_cli status --timeout 5 > "$EVID/final_mission.json" 2>&1 || true
stage "done"
echo "SIH_SCENARIO_DONE scenario=$SCENARIO state=$FINAL_STATE disarmed=$DISARMED"
# 显式退出：cleanup 已在 EXIT trap 中执行，这里不再等待任何后台子进程。
[[ "$DISARMED" == "yes" ]] || exit 5
exit 0
