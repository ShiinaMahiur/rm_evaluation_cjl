#!/usr/bin/env bash
# record_demo.sh —— 自动录「自瞄演示 + A~E 场景回放」到 media/
#
# 用系统自带 GNOME 录屏（D-Bus org.gnome.Shell.Screencast），不需要 OBS / ffmpeg。
# 只录「仿真器窗口」区域，不会把桌面其它窗口录进去。
#
# 用法：
#   bash scripts/record_demo.sh              # 录自瞄演示 30s + A~E 回放
#   bash scripts/record_demo.sh --aim-only   # 只录自瞄演示
#   bash scripts/record_demo.sh --replay-only# 只录 A~E 回放
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
MEDIA="$REPO/media"
BAGS="${BAGS:-/media/chen/RJ/rm_vision/bags}"
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/1000/bus}"
mkdir -p "$MEDIA"

ok()   { printf '\033[0;32m[ok]\033[0m %s\n' "$*"; }
say()  { printf '\033[0;36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[!]\033[0m %s\n' "$*"; }

MODE="${1:-all}"

# ---- 录屏控制 ----
rec_start() {  # rec_start <输出文件>
  local out="$1"
  rm -f "$out"
  # 仿真器窗口位置（取 daedalus 窗口）
  local wid x y w h
  wid=$(xdotool search --name daedalus 2>/dev/null | head -1)
  if [ -z "$wid" ]; then warn "找不到 daedalus 窗口，整屏录制"; x=0; y=0; w=2560; h=1600; fi
  if [ -n "${wid:-}" ]; then
    eval "$(xdotool getwindowgeometry --shell "$wid" 2>/dev/null)"
    [ -z "${WIDTH:-}" ] && { x=0; y=0; w=2560; h=1600; } || { x=$X; y=$Y; w=$WIDTH; h=$HEIGHT; }
  fi
  say "录制区域 ${w}x${h} @ (${x},${y}) → $(basename "$out")"
  gdbus call --session --dest org.gnome.Shell.Screencast \
    --object-path /org/gnome/Shell/Screencast \
    --method org.gnome.Shell.Screencast.ScreencastArea \
    "$x" "$y" "$w" "$h" "$out" "{'framerate': <30>}" >/dev/null 2>&1
}
rec_stop() {
  gdbus call --session --dest org.gnome.Shell.Screencast \
    --object-path /org/gnome/Shell/Screencast \
    --method org.gnome.Shell.Screencast.StopScreencast >/dev/null 2>&1
  sleep 2
}

TMP_AIM="$MEDIA/demo_aim.mp4"
TMP_SCENES="$MEDIA/demo_A-E.mp4"

# ---- 1) 自瞄演示 ----
if [ "$MODE" != "--replay-only" ]; then
  say "启动自瞄演示（含仿真器 + 检测/瞄准窗口）..."
  ( cd "$REPO" && nohup timeout 240 bash scripts/demo_live.sh --fire auto --debug-view > /tmp/demo_record.log 2>&1 & )
  # 等开火（S4 日志出现 fire_rate 即可）
  for _ in $(seq 1 40); do
    sleep 3
    grep -q 'fire_rate' /tmp/demo_live/S4.log 2>/dev/null && break
  done
  ok "自瞄已开火，开始录制 30 s"
  xdotool search --name daedalus 2>/dev/null | head -1 | xargs -r xdotool windowactivate --sync 2>/dev/null
  rec_start "$TMP_AIM"
  sleep 30
  rec_stop
  [ -s "$TMP_AIM" ] && ok "自瞄演示录完：$TMP_AIM ($(du -h "$TMP_AIM" | cut -f1))" || warn "自瞄录制为空"
fi

# ---- 2) A~E 回放 ----
if [ "$MODE" != "--aim-only" ]; then
  say "停止演示，准备录 A~E 回放"
  python3 - <<'PY'
import os, signal
for pid in [int(p) for p in os.listdir('/proc') if p.isdigit()]:
    try:
        cmd = open(f'/proc/{pid}/cmdline','rb').read().decode('utf8','replace').replace('\x00',' ')
    except Exception:
        continue
    if cmd.startswith('/usr/bin/python3') and any(s in cmd for s in ['lib/rm_vision_probe','ros2 run rm_vision','view_debug.py']):
        try: os.kill(pid, 9)
        except Exception: pass
PY
  pkill -x daedalus 2>/dev/null; sleep 2
  rec_start "$TMP_SCENES"
  for s in A B C D E; do
    say "回放 scene$s（约 15 s）"
    xdotool search --name daedalus 2>/dev/null | head -1 | xargs -r xdotool windowactivate --sync 2>/dev/null
    if [ -f "$HOME/replay_bag.sh" ]; then
      timeout 30 bash "$HOME/replay_bag.sh" "$BAGS/scene$s" > "/tmp/replay_$s.log" 2>&1 || true
    else
      warn "没有 ~/replay_bag.sh，跳过 scene$s"
    fi
    sleep 2
  done
  rec_stop
  [ -s "$TMP_SCENES" ] && ok "A~E 回放录完：$TMP_SCENES ($(du -h "$TMP_SCENES" | cut -f1))" || warn "A~E 录制为空"
fi

echo
say "产物："
ls -lh "$MEDIA"/*.mp4 2>/dev/null
