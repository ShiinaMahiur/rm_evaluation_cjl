#!/usr/bin/env bash
# =============================================================================
#  check_topics.sh —— S0 验收①：确认 /image_raw、/tf、/camera_info 有数据
# -----------------------------------------------------------------------------
#  检查内容（对每个话题）：
#    1. 是否存在 + 发布者数量（ros2 topic info）
#    2. 是否有数据流 + 实际频率（ros2 topic hz）
#    3. 抽样字段（width/height、K 矩阵、tf 变换数量）
#
#  用法:
#    ~/check_topics.sh          # 仿真器要已在跑
# =============================================================================
set -uo pipefail
ros_source() {
  local f
  for f in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$HOME/vision_ws/install/setup.bash"; do
    [ -f "$f" ] || continue
    set +u; . "$f" >/dev/null 2>&1; set -u
  done
}
ros_source
command -v ros2 >/dev/null || { echo "[X] ros2 不可用（先 source ~/.bashrc）"; exit 1; }

echo "========== S0 话题自检 =========="
echo "时间: $(date '+%F %T')"
echo

# 1) 发布者
echo "--- 1. 话题是否存在 / 谁在发 ---"
for t in /image_raw /camera_info /tf; do
  info="$(timeout 6 ros2 topic info "$t" 2>/dev/null)"
  if [ -z "$info" ]; then echo "[X] $t  不存在"; continue; fi
  typ=$(echo "$info" | grep -oE "Type: .*" | head -1)
  pub=$(echo "$info" | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+")
  echo "[ok] $t  $typ  publishers=$pub"
done
echo

# 2) 频率
echo "--- 2. 是否有数据流（各测 3 秒）---"
for t in /image_raw /camera_info /tf; do
  hz="$(timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -oE "average rate: [0-9.]+" | tail -1 | grep -oE "[0-9.]+")"
  if [ -n "${hz:-}" ]; then echo "[ok] $t  ≈ ${hz} Hz"; else echo "[X] $t  3 秒内没有数据"; fi
done
echo

# 3) 抽样字段
echo "--- 3. 抽样字段 ---"
w="$(timeout 6 ros2 topic echo /image_raw --once --field width 2>/dev/null | head -1)"
h="$(timeout 6 ros2 topic echo /image_raw --once --field height 2>/dev/null | head -1)"
e="$(timeout 6 ros2 topic echo /image_raw --once --field encoding 2>/dev/null | head -1)"
echo "[ok] /image_raw: width=$w height=$h encoding=$e"
k="$(timeout 6 ros2 topic echo /camera_info --once --field k 2>/dev/null | tr '\n' ' ' | sed 's/  */ /g' | cut -c1-120)"
echo "[ok] /camera_info K=$k"
n="$(timeout 8 ros2 topic echo /tf --once 2>/dev/null | grep -c "child_frame_id")"
echo "[ok] /tf: 一帧消息里有 $n 个变换"
echo
echo "========== 自检结束 =========="
