#!/usr/bin/env bash
# =============================================================================
#  bag_view.sh —— 一键回放 bag 并显示图像窗口（单终端，关窗/结束都不卡死）
# -----------------------------------------------------------------------------
#  原理：
#    - 后台 `ros2 bag play` 重放话题
#    - 后台 `rqt_image_view /image_raw` 显示图像
#    - 本脚本轮询窗口是否存在：**你点 X 关掉图像窗口 → 脚本自动收尾**
#    - bag 播完 / Ctrl+C 也会自动收尾
#
#  用法:
#    ~/bag_view.sh                    # 最新 bag
#    ~/bag_view.sh /path/to/sceneB    # 指定 bag
#    RATE=0.5 ~/bag_view.sh           # 半速
# =============================================================================
set -uo pipefail
BAGS_ROOT="${BAGS_ROOT:-/media/$USER/RJ/rm_vision/bags}"
RATE="${RATE:-1.0}"
WIN_PATTERN="${WIN_PATTERN:-rqt_image_view}"

ros_source() {
  local f
  for f in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$HOME/vision_ws/install/setup.bash"; do
    [ -f "$f" ] || continue
    set +u; . "$f" >/dev/null 2>&1; set -u
  done
}
ros_source
command -v ros2 >/dev/null || { echo "[X] ros2 不可用（先 source ~/.bashrc）"; exit 1; }
command -v xdotool >/dev/null || { echo "[X] 需要 xdotool"; exit 1; }

BAG="${1:-}"
if [ -z "$BAG" ]; then
  BAG="$(find "$BAGS_ROOT" -maxdepth 2 -name metadata.yaml -printf '%h\n' 2>/dev/null | sort | tail -1)"
fi
[ -n "${BAG:-}" ] || { echo "[X] 没找到 bag"; exit 1; }
[ -f "$BAG/metadata.yaml" ] || { echo "[X] $BAG 不是 bag 目录"; exit 1; }

PB=""; VP=""
cleanup() {
  [ -n "$VP" ] && kill -TERM -- "-$VP" 2>/dev/null
  [ -n "$PB" ] && kill -TERM -- "-$PB" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "==> 回放：$BAG （rate=$RATE）"
echo "==> 点图像窗口的 X 或 Ctrl+C 结束（bag 播完也会自动结束）"

setsid ros2 bag play "$BAG" -r "$RATE" --read-ahead-queue-size 2000 >/tmp/bag_play.log 2>&1 &
PB=$!
sleep 1
setsid ros2 run rqt_image_view rqt_image_view /image_raw >/tmp/rqt_image_view.log 2>&1 &
VP=$!

# 等窗口出现（最多 30 秒）
WID=""
for i in $(seq 1 60); do
  WID="$(xdotool search --onlyvisible --name "$WIN_PATTERN" 2>/dev/null | head -1)"
  [ -n "$WID" ] && break
  sleep 0.5
done
if [ -z "$WID" ]; then
  echo "[!] 图像窗口没出现，看 /tmp/rqt_image_view.log"; cleanup; exit 1
fi
echo "==> 图像窗口已打开（$WID）"

# 轮询：窗口没了 / 回放进程结束 → 收尾
while true; do
  if ! xdotool search --onlyvisible --name "$WIN_PATTERN" >/dev/null 2>&1; then
    echo "==> 图像窗口已关闭"; break
  fi
  if ! kill -0 "$PB" 2>/dev/null; then
    echo "==> bag 播放结束"; break
  fi
  sleep 0.5
done

cleanup
trap - EXIT INT TERM
echo "==> 已清理并退出"
