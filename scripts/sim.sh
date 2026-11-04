#!/usr/bin/env bash
# =============================================================================
#  sim.sh —— Daedalus 一键启动/停止/状态（替代手敲 run_daedalus.sh + 改 config）
# -----------------------------------------------------------------------------
#  用法：
#    sim.sh                          # 正常启动（后台，ROS 外部控制模式，窗口黑）
#    sim.sh --preview                # 启动 + 开预览窗口（能看到画面）
#    sim.sh --preview --manual       # 启动 + 预览 + 手动模式（方向键控云台）
#    sim.sh --fg ...                 # 前台运行（Ctrl+C 停）
#    sim.sh --stop                   # 停止
#    sim.sh --status                 # 看当前状态
#    sim.sh --keep --preview         # 已经在跑就复用，不重启
#
#  可用开关：
#    --preview / --no-preview   预览窗口开关（会改 config.toml 的 [preview] enabled）
#    --manual / --auto          手动云台 / ROS 外部控制（默认 auto）
#    --pid-debug                打印云台 PID 内部状态（DAEDALUS_PID_DEBUG=1）
#    --fg / --bg                前台 / 后台（默认后台）
#    --keep                     若已在运行则复用，不重启
#
#  日志：/tmp/daedalus.log
# =============================================================================
set -uo pipefail
REPO="${REPO:-$HOME/projects/bevy_robomaster_simulator}"
LAUNCHER="$HOME/run_daedalus.sh"
CFG="$REPO/config.toml"
LOG="/tmp/daedalus.log"

PREVIEW=""; MANUAL=0; PIDDBG=0; FG=0; KEEP=0; ACTION="start"

for a in "$@"; do
  case "$a" in
    --preview) PREVIEW=1 ;;
    --no-preview) PREVIEW=0 ;;
    --manual) MANUAL=1 ;;
    --auto) MANUAL=0 ;;
    --pid-debug) PIDDBG=1 ;;
    --fg) FG=1 ;;
    --bg) FG=0 ;;
    --keep) KEEP=1 ;;
    --stop) ACTION=stop ;;
    --status) ACTION=status ;;
    -h|--help) sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "[X] 未知参数: $a"; exit 1 ;;
  esac
done

ros_source() {
  local f
  for f in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$HOME/vision_ws/install/setup.bash"; do
    [ -f "$f" ] || continue
    set +u; . "$f" >/dev/null 2>&1; set -u
  done
}
ros_source

running() { pgrep -x daedalus >/dev/null 2>&1; }

show_status() {
  if running; then
    local pid; pid="$(pgrep -x daedalus | head -1)"
    local mode="auto(ROS)"
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q '^DAEDALUS_MANUAL_AIM=1$'; then mode="manual(方向键)"; fi
    local prev="?"; grep -A4 '^\[preview\]' "$CFG" 2>/dev/null | grep -q 'enabled = true' && prev="on" || prev="off"
    echo "仿真器运行中：PID=$pid  模式=$mode  preview=$prev"
    local w; w="$(timeout 5 ros2 topic echo /image_raw --once --field width 2>/dev/null | head -1)"
    [ -n "$w" ] && echo "/image_raw: width=$w" || echo "/image_raw: ⚠️ 没数据"
  else
    echo "仿真器没在运行"
  fi
}

case "$ACTION" in
  status) show_status; exit 0 ;;
  stop)
    if running; then pkill -x daedalus; sleep 2; running && { pkill -9 -x daedalus; sleep 1; }; echo "[ok] 已停止"; else echo "本来就没在跑"; fi
    exit 0 ;;
esac

# ---------- 启动 ----------
[ -x "$LAUNCHER" ] || { echo "[X] 找不到 $LAUNCHER"; exit 1; }
[ -f "$CFG" ] || { echo "[X] 找不到 $CFG"; exit 1; }

if running && [ "$KEEP" = "1" ]; then
  echo "[ok] 已在运行（--keep），复用"; show_status; exit 0
fi
if running; then
  echo "==> 关闭旧实例"; pkill -x daedalus; sleep 2
  running && { pkill -9 -x daedalus; sleep 1; }
fi

# 1) 按需改 preview
if [ -n "$PREVIEW" ]; then
  python3 - "$CFG" "$PREVIEW" <<'PY'
import re, sys
cfg, val = sys.argv[1], "true" if sys.argv[2] == "1" else "false"
s = open(cfg, encoding="utf-8").read()
s2, done = re.subn(r'(\[preview\][^\[]*?enabled = )(true|false)', lambda m: m.group(1) + val, s, count=1)
if done:
    open(cfg, "w", encoding="utf-8").write(s2)
    print(f"[ok] config.toml [preview] enabled = {val}")
else:
    print("[!] config.toml 里没找到 [preview] enabled，跳过")
PY
fi

# 2) 组环境变量
ENVS=()
[ "$MANUAL" = "1" ] && ENVS+=("DAEDALUS_MANUAL_AIM=1")
[ "$PIDDBG" = "1" ] && ENVS+=("DAEDALUS_PID_DEBUG=1")
ENVS+=("DAEDALUS_BACKEND=x11")

PREV_STATE="?"; grep -A4 '^\[preview\]' "$CFG" | grep -q 'enabled = true' && PREV_STATE="on" || PREV_STATE="off"
echo "==> 启动 Daedalus：模式=$([ "$MANUAL" = 1 ] && echo manual || echo auto)  preview=$PREV_STATE"

if [ "$FG" = "1" ]; then
  cd "$REPO" || exit 1
  exec env "${ENVS[@]}" "$LAUNCHER"
fi

cd "$REPO" || exit 1
: > "$LOG"
setsid env "${ENVS[@]}" "$LAUNCHER" >>"$LOG" 2>&1 < /dev/null &
echo "==> 后台启动中，日志：$LOG"

# 3) 等 /image_raw
for i in $(seq 1 60); do
  sleep 1
  if ! running; then echo "[X] 进程退出了，看日志："; tail -20 "$LOG"; exit 1; fi
  w="$(timeout 3 ros2 topic echo /image_raw --once --field width 2>/dev/null | head -1)"
  if [ "$w" = "1440" ]; then break; fi
done
if [ "${w:-}" != "1440" ]; then echo "[!] 60 秒内 /image_raw 没数据，看日志：$LOG"; tail -10 "$LOG"; exit 1; fi

echo "==> 就绪"
show_status
echo
echo "提示：停止用  sim.sh --stop ；看日志  tail -f $LOG"
