#!/usr/bin/env bash
# bag_view.sh —— 一键"回放 bag + 弹出图像窗口"（bag 本身不是视频文件）
# 用法:
#   ~/bag_view.sh                          # 最新 bag
#   ~/bag_view.sh /path/to/sceneB          # 指定 bag
#   RATE=0.5 EVERY=2 ~/bag_view.sh         # 半速 + 每2帧显示1帧
set -uo pipefail
BAGS_ROOT="${BAGS_ROOT:-/media/$USER/RJ/rm_vision/bags}"
RATE="${RATE:-1.0}"
EVERY="${EVERY:-1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ros_source() {
  local f
  for f in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$HOME/vision_ws/install/setup.bash"; do
    [ -f "$f" ] || continue
    set +u; . "$f" >/dev/null 2>&1; set -u
  done
}
ros_source
command -v ros2 >/dev/null || { echo "[X] ros2 不可用"; exit 1; }

BAG="${1:-}"
if [ -z "$BAG" ]; then
  BAG="$(find "$BAGS_ROOT" -maxdepth 2 -name metadata.yaml -printf '%h\n' 2>/dev/null | sort | tail -1)"
fi
[ -n "${BAG:-}" ] || { echo "[X] 没找到 bag"; exit 1; }
[ -f "$BAG/metadata.yaml" ] || { echo "[X] $BAG 不是 bag 目录"; exit 1; }

VIEW="$SCRIPT_DIR/bag_view.py"
[ -f "$VIEW" ] || VIEW="$HOME/bag_view.py"
[ -f "$VIEW" ] || { echo "[X] 找不到 bag_view.py"; exit 1; }
python3 "$VIEW" "$BAG" --rate "$RATE" --every "$EVERY"
