#!/usr/bin/env bash
# =============================================================================
#  run_all.sh —— 一键启动整套系统（提交要求里的"一键运行脚本"）
# -----------------------------------------------------------------------------
#  当前阶段（S0）：
#    1. 启动 Daedalus（ROS 外部控制模式）
#    2. 等 /image_raw 就绪
#    3. 启动 follow_target（图像重心 → /rm_gimbal/cmd，验证视觉→云台闭环）
#
#  后续阶段：把 PIPELINE_CMD 换成 `ros2 launch rm_vision ...` 即可，无需改其他逻辑。
#
#  用法:
#    ~/run_all.sh                 # 启动全部
#    VIEW=1 ~/run_all.sh          # 同时开图像窗口
#    START_SIM=0 ~/run_all.sh     # 仿真器已开
#  退出：Ctrl+C
# =============================================================================
set -uo pipefail
START_SIM="${START_SIM:-1}"
VIEW="${VIEW:-0}"
PIPELINE_CMD="${PIPELINE_CMD:-python3 $HOME/follow_target.py}"
SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ros_source() {
  local f
  for f in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$HOME/vision_ws/install/setup.bash"; do
    [ -f "$f" ] || continue
    set +u; . "$f" >/dev/null 2>&1; set -u
  done
}
ros_source
command -v ros2 >/dev/null || { echo "[X] ros2 不可用"; exit 1; }

SIM=""; PIPE=""; VW=""
cleanup() {
  for p in "$VW" "$PIPE" "$SIM"; do
    [ -n "$p" ] && kill -TERM -- "-$p" 2>/dev/null
  done
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

if pgrep -x daedalus >/dev/null; then
  echo "==> 仿真器已在运行"
else
  [ "$START_SIM" = "1" ] || { echo "[X] 仿真器没跑且 START_SIM=0"; exit 1; }
  echo "==> 启动仿真器"
  setsid env DAEDALUS_BACKEND=x11 "$HOME/run_daedalus.sh" >/tmp/run_all_sim.log 2>&1 < /dev/null &
  SIM=$!
fi

echo "==> 等 /image_raw ..."
W=""
for i in $(seq 1 90); do
  W="$(timeout 3 ros2 topic echo /image_raw --once --field width 2>/dev/null | grep -oE '[0-9]+' | head -1)"
  [ "$W" = "1440" ] && break
  sleep 1
done
[ "$W" = "1440" ] || { echo "[X] /image_raw 没出来"; exit 1; }
echo "==> /image_raw OK（1440）"

if [ "$VIEW" = "1" ]; then
  setsid ros2 run rqt_image_view rqt_image_view /image_raw >/tmp/run_all_view.log 2>&1 < /dev/null &
  VW=$!
fi

echo "==> 启动视觉链路：$PIPELINE_CMD"
setsid bash -c "$PIPELINE_CMD" >/tmp/run_all_pipeline.log 2>&1 < /dev/null &
PIPE=$!

echo "==> 系统已启动（日志：/tmp/run_all_*.log），Ctrl+C 退出"
wait "$PIPE" 2>/dev/null
