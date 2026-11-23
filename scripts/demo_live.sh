#!/usr/bin/env bash
# demo_live.sh —— 一键复现「自瞄打靶」演示（视频里那种效果）
#
# 做的事：
#   1. 安全地打开仿真器 preview（只改 [preview] 段，不动 [livox_ros] / [projectile.aerodynamics]）
#   2. 启动 Daedalus（X11 后端，可录屏/可用键盘摆场景）
#   3. 编译并启动我们自己的四段链路：
#        detector_traditional(S1) → armor_solver(S2) → armor_tracker(S3) → trajectory_planner(S4)
#   4. 等目标被捕获后自动开火；实时打印瞄准角和仿真命中统计
#   5. Ctrl+C 退出时杀掉节点、恢复 config.toml
#
# 用法：
#   bash scripts/demo_live.sh                  # 连发模式（默认）
#   bash scripts/demo_live.sh --fire single    # 每 0.5 s 一发
#   bash scripts/demo_live.sh --fire off       # 只瞄准不开火
#   bash scripts/demo_live.sh --keep-sim       # 复用已经在跑的仿真器
#   bash scripts/demo_live.sh --preview-off    # 不打开画面（黑屏，只测逻辑）
set -uo pipefail

SIM_DIR="${SIM_DIR:-$HOME/projects/bevy_robomaster_simulator}"
CONFIG="$SIM_DIR/config.toml"
BACKUP="$SIM_DIR/config.toml.demo_bak"
WAIT_IMAGE="${WAIT_IMAGE:-60}"
DEV="${DEV:-/tmp/ws_demo}"
LOG_DIR="${LOG_DIR:-/tmp/demo_live}"

FIRE=auto
KEEP_SIM=0
PREVIEW=1
while [ $# -gt 0 ]; do
  case "$1" in
    --fire) FIRE="${2:-auto}"; shift 2 ;;
    --fire=*) FIRE="${1#*=}"; shift ;;
    --keep-sim) KEEP_SIM=1; shift ;;
    --preview-off) PREVIEW=0; shift ;;
    *) echo "[demo] 未知参数: $1"; exit 2 ;;
  esac
done

ok()   { printf '\033[0;32m[ok]\033[0m %s\n' "$*"; }
say()  { printf '\033[0;36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[!]\033[0m %s\n' "$*"; }
die()  { printf '\033[0;31m[X]\033[0m %s\n' "$*" >&2; exit 1; }

mkdir -p "$LOG_DIR"
cleanup() {
  say "收尾：停止节点"
  python3 - <<'PY' 2>/dev/null || true
import os, signal
subs = ['lib/rm_vision_probe', 'ros2 run rm_vision_probe']
me = os.getpid()
for pid in [int(p) for p in os.listdir('/proc') if p.isdigit()]:
    if pid == me:
        continue
    try:
        cmd = open(f'/proc/{pid}/cmdline', 'rb').read().decode('utf8', 'replace').replace('\x00', ' ')
    except Exception:
        continue
    if cmd.startswith('/usr/bin/python3') and any(s in cmd for s in subs):
        try:
            os.kill(pid, signal.SIGKILL)
        except Exception:
            pass
PY
  if [ "$PREVIEW" = "1" ] && [ -f "$BACKUP" ]; then
    cp "$BACKUP" "$CONFIG" && ok "config.toml 已恢复（preview 关）"
  fi
  if [ "$KEEP_SIM" != "1" ]; then
    pkill -x daedalus 2>/dev/null && ok "仿真器已停止"
  fi
  xset r on 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ---------- 0. 检查 ----------
[ -f "$CONFIG" ] || die "找不到 $CONFIG（仿真器没装？）"
[ -x "$SIM_DIR/target/release/daedalus" ] || die "仿真器还没编译：cd $SIM_DIR && cargo build --release --features ros2"
set +u
source /opt/ros/humble/setup.bash || die "ROS2 Humble 没装"
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || warn "~/ros2_ws（rm_interfaces）没找到"
set -u

# ---------- 1. 安全打开 preview ----------
if [ "$PREVIEW" = "1" ]; then
  cp -n "$CONFIG" "$BACKUP" 2>/dev/null || true
  python3 - "$CONFIG" <<'PY'
import os, sys, tempfile
p = sys.argv[1]
lines = open(p, encoding='utf-8').read().splitlines()
sec, changed = '', False
for i, ln in enumerate(lines):
    s = ln.strip()
    if s.startswith('[') and s.endswith(']'):
        sec = s
        continue
    if sec == '[preview]' and s.replace(' ', '') == 'enabled=false':
        lines[i] = 'enabled = true'
        changed = True
print('  [preview] enabled → true' if changed else '  [preview] 已经打开')
fd, tmp = tempfile.mkstemp(dir=os.path.dirname(p), suffix='.tmp')
with os.fdopen(fd, 'w', encoding='utf-8') as fh:
    fh.write('\n'.join(lines) + '\n')
os.replace(tmp, p)   # 原子替换，避免仿真器热重载读到半截文件
PY
else
  warn "跳过 preview（画面会是黑的）"
fi

# ---------- 2. 启动仿真器 ----------
if pgrep -x daedalus >/dev/null; then
  if [ "$PREVIEW" = "1" ] || [ "$KEEP_SIM" != "1" ]; then
    warn "仿真器在跑：切换 preview 会打断 /image_raw，先重启一个干净的"
    pkill -x daedalus 2>/dev/null
    sleep 3
  fi
fi
if pgrep -x daedalus >/dev/null; then
  warn "仿真器已在运行，直接复用"
  KEEP_SIM=1
  SIM_LOG="$(ls -t /tmp/sim*.log 2>/dev/null | head -1)"
else
  say "启动 Daedalus（X11 后端）..."
  SIM_LOG="$LOG_DIR/sim.log"
  ( cd "$HOME" && DAEDALUS_BACKEND=x11 setsid ~/run_daedalus.sh > "$SIM_LOG" 2>&1 < /dev/null & )
  sleep 3
fi
say "等待 /image_raw（最多 ${WAIT_IMAGE}s）..."
t0=$(date +%s)
while true; do
  h=$(timeout 4 ros2 topic echo /image_raw --once --field height 2>/dev/null | head -1 | tr -dc '0-9')
  [ -n "${h:-}" ] && [ "$h" -gt 100 ] 2>/dev/null && { ok "/image_raw 就绪（${h}p）"; break; }
  [ $(( $(date +%s) - t0 )) -gt "$WAIT_IMAGE" ] && die "等不到 /image_raw（看看 $SIM_LOG）"
  sleep 2
done

# ---------- 3. 编译并启动四段链路 ----------
say "编译 rm_vision_probe（tmp workspace，避免 NTFS 编译问题）..."
rm -rf "$DEV"; mkdir -p "$DEV/src"
cp -r "$(dirname "$0")/../vision_ws/src/rm_vision_probe" "$DEV/src/"
( cd "$DEV" && colcon build --packages-select rm_vision_probe > "$LOG_DIR/build.log" 2>&1 ) \
  || die "colcon build 失败，看 $LOG_DIR/build.log"
set +u; source "$DEV/install/setup.bash"; set -u

launch() {  # launch <名字> <包内可执行> [args...]
  local name="$1"; shift
  setsid nohup ros2 run rm_vision_probe "$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
  sleep 1.5
}
say "启动 S1 检测 / S2 PnP / S3 EKF / S4 规划 ..."
launch S1 detector_traditional --ros-args -p publish_hz:=30.0 -p debug_image:=false
launch S2 armor_solver
launch S3 armor_tracker
say "搜索目标（云台缓慢扫视，最多 60s）..."
timeout 70 python3 "$(dirname "$0")/aim_search.py" --span 180 >> "$LOG_DIR/search.log" 2>&1 \
  || warn "搜索超时（看 $LOG_DIR/search.log），S4 可能没目标"
grep -E '\[搜索\]' "$LOG_DIR/search.log" 2>/dev/null | tail -3
launch S4 trajectory_planner --ros-args -p fire_mode:="$FIRE"

# ---------- 4. 等目标捕获 + 打印状态 ----------
say "等待 S3 捕获目标（最多 25s，sim 日志：$SIM_LOG）..."
t0=$(date +%s)
while true; do
  tgt=$(timeout 5 ros2 topic echo /tracker/target --once 2>/dev/null | grep -oE '"lost": (true|false)' | head -1)
  aim=$(timeout 5 ros2 topic echo /planner/aim --once 2>/dev/null | tr -d '\n' | head -c 400)
  if [ -n "${aim:-}" ]; then
    echo "$aim" | python3 -c "
import json,sys
try: d=json.loads(sys.stdin.read())
except Exception: print('  S4 已输出但解析失败'); raise SystemExit
print(f\"  瞄准：yaw {d['yaw_deg']:+.2f}°  pitch {d['pitch_deg']:+.2f}°  距离 {d['horizontal_distance_m']:.2f} m  飞行 {d['t_flight_ms']:.0f} ms  fire={d['fire_advice']}\")
" 2>/dev/null
    ok "自瞄已工作（fire_mode=$FIRE）"
    break
  fi
  [ $(( $(date +%s) - t0 )) -gt 25 ] && { warn "25s 内没拿到 S4 输出，看 $LOG_DIR/S4.log"; break; }
  sleep 2
done

# ---------- 5. 实时打印 ----------
SIM_BASE=$(grep -oE 'shots=[0-9]+ hits=[0-9]+' "$SIM_LOG" 2>/dev/null | tail -1)
say "运行中：按 Ctrl+C 结束（命中基线：${SIM_BASE:-未知}）"
last_shots=""
for _ in $(seq 1 600); do
  sleep 2
  if [ -n "${SIM_LOG:-}" ] && [ -f "$SIM_LOG" ]; then
    stats=$(grep -oE 'shots=[0-9]+ hits=[0-9]+ pct=[0-9.]+' "$SIM_LOG" 2>/dev/null | tail -1)
    if [ -n "${stats:-}" ] && [ "$stats" != "$last_shots" ]; then
      echo "  [仿真命中统计] $stats"
      last_shots="$stats"
    fi
  fi
  det=$(grep -oE '检出率=[0-9.]+%' "$LOG_DIR/S1.log" 2>/dev/null | tail -1)
  [ -n "${det:-}" ] && echo "  [S1] $det" || true
done
