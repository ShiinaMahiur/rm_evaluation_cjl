#!/usr/bin/env bash
# =============================================================================
#  replay_bag.sh —— S0 验收项：一键回放 bag 并让云台跟随
# -----------------------------------------------------------------------------
#  做的三件事：
#    1. 确保仿真器在跑（没有就自动起；默认是 ROS 外部控制模式，别加 MANUAL_AIM）
#    2. 起 follow_target.py（图像重心 + P 控制 → /rm_gimbal/cmd）
#    3. 回放指定 bag → 画面里目标偏离中心时，云台自动转过去跟住
#
#  用法:
#    ~/replay_bag.sh                          # 回放最新 bag（自动起仿真器）
#    ~/replay_bag.sh /path/to/sceneB          # 指定 bag
#    RATE=0.5 ~/replay_bag.sh                 # 半速
#    START_SIM=0 ~/replay_bag.sh              # 仿真器已经在跑，不重复启动
#    VIEW=1 ~/replay_bag.sh                   # 同时弹 rqt 图像窗口
#    NO_FOLLOW=1 ~/replay_bag.sh              # 只回放，不驱动云台
#
#  退出：Ctrl+C 一次，脚本会收掉自己启动的所有进程（不留后台残留）
# =============================================================================
set -uo pipefail
BAGS_ROOT="${BAGS_ROOT:-/media/$USER/RJ/rm_vision/bags}"
RATE="${RATE:-1.0}"
START_SIM="${START_SIM:-1}"
NO_FOLLOW="${NO_FOLLOW:-0}"
VIEW="${VIEW:-0}"
SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ros_source() {
  local f
  for f in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$HOME/vision_ws/install/setup.bash"; do
    [ -f "$f" ] || continue
    set +u; . "$f" >/dev/null 2>&1; set -u
  done
}
ros_source
command -v ros2 >/dev/null || { echo "[X] ros2 不可用（先 source ~/.bashrc）"; exit 1; }

BAG="${1:-}"
[ -n "$BAG" ] || BAG="$(find "$BAGS_ROOT" -maxdepth 2 -name metadata.yaml -printf '%h\n' 2>/dev/null | sort | tail -1)"
[ -n "${BAG:-}" ] || { echo "[X] 没找到 bag"; exit 1; }
[ -f "$BAG/metadata.yaml" ] || { echo "[X] $BAG 不是 bag 目录"; exit 1; }

SIM=""; FT=""; VW=""; PB=""
cleanup() {
  for p in "$VW" "$FT" "$PB" "$SIM"; do
    [ -n "$p" ] && kill -TERM -- "-$p" 2>/dev/null
  done
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "==> bag: $BAG （rate=$RATE）"

# ---- 1. 仿真器 ----
if pgrep -x daedalus >/dev/null; then
  echo "==> 仿真器已在运行，直接复用"
else
  if [ "$START_SIM" = "1" ]; then
    echo "==> 启动仿真器（ROS 外部控制模式）"
    setsid env DAEDALUS_BACKEND=x11 "$HOME/run_daedalus.sh" >/tmp/replay_sim.log 2>&1 < /dev/null &
    SIM=$!
  else
    echo "[!] 仿真器没在跑且 START_SIM=0 —— 云台不会有反应（只能看图像）"
  fi
fi

echo "==> 等 /image_raw ..."
W=""
for i in $(seq 1 90); do
  W="$(timeout 3 ros2 topic echo /image_raw --once --field width 2>/dev/null | grep -oE '[0-9]+' | head -1)"
  [ "$W" = "1440" ] && break
  sleep 1
done
[ "$W" = "1440" ] || { echo "[X] /image_raw 没出来（$W）"; exit 1; }

# ---- 2. follow_target ----
if [ "$NO_FOLLOW" != "1" ]; then
  FT_SRC="$SELF_DIR/follow_target.py"; [ -f "$FT_SRC" ] || FT_SRC="$HOME/follow_target.py"
  [ -f "$FT_SRC" ] || { echo "[X] 找不到 follow_target.py"; exit 1; }
  echo "==> 启动 follow_target（图像重心 P 控制 → /rm_gimbal/cmd）"
  setsid python3 "$FT_SRC" >/tmp/follow_target.log 2>&1 < /dev/null &
  FT=$!
  sleep 1.5
else
  echo "==> NO_FOLLOW=1，不驱动云台"
fi

# ---- 3. 可选的图像窗口 ----
if [ "$VIEW" = "1" ]; then
  echo "==> 打开 rqt 图像窗口（关掉它不会中断回放）"
  setsid ros2 run rqt_image_view rqt_image_view /image_raw >/tmp/replay_view.log 2>&1 < /dev/null &
  VW=$!
fi

# ---- 4. 回放 ----
echo "==> 开始回放，Ctrl+C 结束"
setsid ros2 bag play "$BAG" -r "$RATE" --read-ahead-queue-size 2000 >/tmp/replay_play.log 2>&1 < /dev/null &
PB=$!
wait "$PB" 2>/dev/null
echo "==> 回放结束"

# 回放结束后留 2 秒让云台回到最后一个目标
sleep 2
echo "==> 清理"
