#!/usr/bin/env bash
# =============================================================================
#  bag_play.sh —— 一键回放 S0 录制的 bag，并让仿真云台"跟随"录到的姿态
# -----------------------------------------------------------------------------
#  用法:
#    bash bag_play.sh                        # 回放最新的一个 bag（自动找）
#    bash bag_play.sh /path/to/sceneB        # 回放指定 bag
#    RATE=0.5 bash bag_play.sh               # 半速回放（看清动作）
#    NO_FOLLOW=1 bash bag_play.sh            # 只回放，不驱动云台
#
#  它做的事:
#    1. source ROS + vision_ws
#    2. 自动找 bag（默认 /media/$USER/RJ/rm_vision/bags 下最新的）
#    3. 启动 gimbal_follow.py（订阅 /gimbal_pose → 发 /rm_gimbal/cmd，单位：度）
#    4. 用 ros2 bag play 回放，Ctrl+C 同时停掉两者
#
#  前提:
#    - 仿真器已在跑：~/run_daedalus.sh（否则只能看图像，云台没人执行）
#    - bag 里含 /gimbal_pose（record_bags.sh 默认会录）
# =============================================================================
set -uo pipefail

BAGS_ROOT="${BAGS_ROOT:-/media/$USER/RJ/rm_vision/bags}"
RATE="${RATE:-1.0}"
NO_FOLLOW="${NO_FOLLOW:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say() { echo; echo "==> $*"; }
die() { echo "[X] $*" >&2; exit 1; }

ros_source() {
  local f
  for f in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$HOME/vision_ws/install/setup.bash"; do
    [ -f "$f" ] || continue
    set +u; . "$f" >/dev/null 2>&1; set -u
  done
}
ros_source
command -v ros2 >/dev/null || die "ros2 不可用"

# ---- 选 bag ----
BAG="${1:-}"
if [ -z "$BAG" ]; then
  BAG="$(find "$BAGS_ROOT" -maxdepth 2 -name metadata.yaml -printf '%h\n' 2>/dev/null | sort | tail -1)"
fi
[ -n "${BAG:-}" ] || die "没找到 bag（$BAGS_ROOT 下没有 metadata.yaml），先录一个"
[ -f "$BAG/metadata.yaml" ] || die "$BAG 不是 bag 目录（缺 metadata.yaml）"

say "回放 bag：$BAG"
ros2 bag info "$BAG" 2>/dev/null | sed -n '1,20p' | sed 's/^/    /'

# ---- 云台跟随 ----
FOLLOW_PID=""
cleanup() {
  [ -n "$FOLLOW_PID" ] && kill "$FOLLOW_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

if [ "$NO_FOLLOW" != "1" ]; then
  FOLLOW="$SCRIPT_DIR/gimbal_follow.py"
  [ -f "$FOLLOW" ] || FOLLOW="$HOME/gimbal_follow.py"
  if [ -f "$FOLLOW" ]; then
    say "启动云台跟随（/gimbal_pose → /rm_gimbal/cmd）"
    python3 "$FOLLOW" >/tmp/gimbal_follow.log 2>&1 &
    FOLLOW_PID=$!
    sleep 2
    kill -0 "$FOLLOW_PID" 2>/dev/null && echo "    跟随节点 PID=$FOLLOW_PID" || { echo "    [!] 跟随节点挂了，看 /tmp/gimbal_follow.log"; FOLLOW_PID=""; }
  else
    echo "    [!] 没找到 gimbal_follow.py，跳过云台跟随"
  fi
else
  echo "    NO_FOLLOW=1，只回放不驱动云台"
fi

# ---- 回放 ----
say "开始回放（rate=$RATE）。Ctrl+C 结束"
if [ "$NO_FOLLOW" != "1" ] && [ -z "$FOLLOW_PID" ]; then
  echo "    [!] 没有跟随节点，云台不会动"
fi
ros2 bag play "$BAG" -r "$RATE" --read-ahead-queue-size 2000

say "回放结束"
