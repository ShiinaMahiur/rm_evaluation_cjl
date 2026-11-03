#!/usr/bin/env bash
# =============================================================================
#  record_bags.sh — S0 数据集录制（原生 Ubuntu 22.04）
# -----------------------------------------------------------------------------
#  一次录 5 个场景：A 静止 / B 平移 / C 自转+平移 / D 移出再回来 / E 大角度斜视
#
#  它替你做这些事（每一条都是真机上会踩的坑）：
#   1. 挂 D 盘（NTFS），bag 直接写到 /media/$USER/RJ/rm_vision/bags
#   2. 把 config.toml 的 present_mode 改成 auto_vsync（默认是 auto_no_vsync，
#      实测会跑到 280+ Hz，20 秒一个场景就 27 GB，D 盘装不下 5 个）
#   3. 用 XWayland 后端启动仿真器（DAEDALUS_BACKEND=x11）—— 因为 Wayland 下
#      xdotool 无法给原生 Wayland 窗口发按键，而假人必须靠按键驱动
#   4. 用 xdotool 按时间轴发 I/J/K/L/U/O/F/V/C/B/Tab 造场景
#   5. 用 zstd 压缩录 /image_raw /camera_info /tf + 几个小体量的真值位姿话题
#   6. 每个场景录完打印 ros2 bag info（消息数/时长/体积）并校验
#
#  用法:
#    bash record_bags.sh                  # 5 个场景 × 20 秒
#    DUR=20 SCENES="A B" bash record_bags.sh   # 只录 A、B
#    DUR=5 SCENES="A" bash record_bags.sh      # 先来个 5 秒试录，验证链路
#    FORCE_HIGH_FPS=1 bash record_bags.sh      # 不检查 60Hz（不建议）
#
#  前提：显示器刷新率切到 60 Hz（设置 → 显示器 → 刷新率 60.00 Hz）。
#        面板 EDID 实测支持 60~240 Hz，不切的话 bag 会大到装不下。
# =============================================================================
set -uo pipefail

# ------------------------------- 可调参数 -----------------------------------
DUR="${DUR:-20}"                                 # 每个场景录多少秒（考核要求 >=20）
SCENES="${SCENES:-A B C D E}"                    # 要录哪些场景
BAGS_ROOT="${BAGS_ROOT:-/media/$USER/RJ/rm_vision/bags}"
D_ONLY_DEV="${D_ONLY_DEV:-/dev/nvme0n1p4}"       # D 盘分区（按你机器 fstab/分区表）
D_MOUNT_POINT="${D_MOUNT_POINT:-/media/$USER/RJ}"
REPO="${REPO:-$HOME/projects/bevy_robomaster_simulator}"
LAUNCHER="${LAUNCHER:-$HOME/run_daedalus.sh}"
CFG="$REPO/config.toml"
SIMLOG="$HOME/daedalus_bags.log"
TOPLIST=(/image_raw /camera_info /tf /gimbal_pose /odom_pose /muzzle_pose /camera_pose)

C_G=$'\033[0;32m'; C_Y=$'\033[1;33m'; C_R=$'\033[0;31m'; C_B=$'\033[0;36m'; C_N=$'\033[0m'
say() { echo; echo "${C_B}==>${C_N} $*"; }
ok()  { echo "  ${C_G}[ok]${C_N} $*"; }
warn(){ echo "  ${C_Y}[!]${C_N} $*"; }
die() { echo "  ${C_R}[X]${C_N} $*" >&2; exit 1; }

# ROS 的 setup.bash 在 set -u 下会因未定义变量直接退出，必须这样 source
ros_source() {
  local f
  for f in /opt/ros/humble/setup.bash "$HOME/ros2_ws/install/setup.bash" "$HOME/vision_ws/install/setup.bash"; do
    [ -f "$f" ] || continue
    set +u; . "$f" >/dev/null 2>&1; set -u
  done
}
ros_source

# ------------------------------- 0. 依赖 ------------------------------------
say "0/6 检查依赖"
command -v ros2 >/dev/null || die "source ROS 失败（/opt/ros/humble/setup.bash）"
ok "ros2 可用"
if ! command -v xdotool >/dev/null; then
  say "安装 xdotool（合成按键用，约 1 MB）"
  sudo apt-get install -y xdotool || die "xdotool 安装失败"
fi
ok "xdotool 可用：$(xdotool --version 2>&1 | head -1)"
[ -x "$LAUNCHER" ] || die "找不到 $LAUNCHER"
[ -f "$CFG" ] || die "找不到 $CFG"
ok "仿真器与配置就位"

# ------------------------------- 1. 挂 D 盘 ---------------------------------
say "1/6 准备 D 盘（bag 直接写这里，home 只有十几 G）"
if ! mount | grep -q "$D_ONLY_DEV"; then
  say "D 盘没挂载，尝试 udisksctl mount"
  udisksctl mount -b "$D_ONLY_DEV" || die "挂载失败，请在文件管理器里点一下 $D_ONLY_DEV 对应的盘"
fi
[ -w "$D_MOUNT_POINT" ] || die "$D_MOUNT_POINT 不可写（NTFS 只读挂载？先在文件管理器点一下盘）"
mkdir -p "$BAGS_ROOT" || die "建不了 $BAGS_ROOT"
FREE_GB=$(df -Pk "$BAGS_ROOT" | awk 'NR==2{printf "%d", $4/1048576}')
[ "${FREE_GB:-0}" -ge 10 ] || die "D 盘只剩 ${FREE_GB} GB，录 5 个场景不够"
ok "D 盘可用，剩余 ${FREE_GB} GB，输出目录 $BAGS_ROOT"

# ------------------------------- 2. 限帧 ------------------------------------
say "2/6 把仿真器限到 60 Hz（present_mode: auto_no_vsync → auto_vsync）"
if grep -q '^present_mode = "auto_no_vsync"' "$CFG"; then
  [ -f "$CFG.pre_bag" ] || cp "$CFG" "$CFG.pre_bag"
  sed -i 's/^present_mode = "auto_no_vsync"/present_mode = "auto_vsync"/' "$CFG"
  ok "已切换（原文件备份：$CFG.pre_bag）"
elif grep -q '^present_mode = "auto_vsync"' "$CFG"; then
  ok "已经是 auto_vsync"
else
  warn "没找到预期的 present_mode 行，请手动确认 config.toml"
fi

# 录 bag 时必须关掉 preview：ros2 构建下主相机 is_active=false，画面是靠 preview
# 的 UI blit 显示的；preview 开着会多渲染一遍（掉帧），且窗口画面和采集内容重复。
python3 - "$CFG" <<'PYCFG'
import re, sys
p = sys.argv[1]
s = open(p, encoding='utf-8').read()
s2 = re.sub(r'(\[preview\][^\[]*?enabled = )true', r'\1false', s, count=1)
if s2 != s:
    open(p, 'w', encoding='utf-8').write(s2)
    print("  preview 已关（录 bag 用）")
PYCFG

# ------------------------------- 3. 起仿真器 ---------------------------------
say "3/6 启动仿真器（XWayland 后端，xdotool 才能发按键）"
pkill -f "target/release/daedalus" 2>/dev/null && { warn "杀掉了残留的仿真器实例"; sleep 2; }
pkill -f "ros2 bag record" 2>/dev/null && { warn "杀掉了残留的 bag 录制进程"; sleep 1; }
rm -f "$SIMLOG"
DAEDALUS_BACKEND=x11 setsid "$LAUNCHER" >"$SIMLOG" 2>&1 </dev/null &
SIM_PID=$!
cleanup_sim() { kill -TERM -"$SIM_PID" 2>/dev/null || kill -TERM "$SIM_PID" 2>/dev/null || true
                sleep 1; pkill -f "target/release/daedalus" 2>/dev/null || true; }
trap cleanup_sim EXIT

say "等待 /image_raw 出数据（最多 60 秒）"
W=0
for i in $(seq 1 60); do
  W="$(timeout 3 ros2 topic echo /image_raw --once --field width 2>/dev/null | grep -oE '[0-9]+' | head -1)"
  [ "${W:-}" = "1440" ] && break
  sleep 1
done
[ "${W:-}" = "1440" ] || { warn "没等到 /image_raw（width=${W:-空}），日志 $SIMLOG 前 30 行："; head -30 "$SIMLOG" | sed 's/^/    /'; die "仿真器没起来"; }
ok "/image_raw 宽度 = 1440"

say "量一下实际帧率"
HZ="$(timeout 8 ros2 topic hz /image_raw 2>/dev/null | grep -oE 'average rate: [0-9.]+' | tail -1 | grep -oE '[0-9.]+')"
if [ -n "${HZ:-}" ]; then
  ok "当前 /image_raw ≈ ${HZ} Hz"
  if [ "$(awk "BEGIN{print ($HZ > 90)}")" = "1" ] && [ "${FORCE_HIGH_FPS:-0}" != "1" ]; then
    warn "帧率还是 >90 Hz —— 说明显示器还没切到 60 Hz"
    warn "请打开：设置 → 显示器 → 刷新率 → 选 60.00 Hz，然后重新跑本脚本"
    die "为保护 D 盘，先停下。确认是 60 Hz 后重跑；确实要高帧率录请加 FORCE_HIGH_FPS=1"
  fi
else
  warn "没量到帧率，继续（体积可能偏大）"
fi

# ------------------------------- 4. 窗口焦点 ---------------------------------
say "4/6 找到仿真窗口并聚焦（xdotool 发按键的前提）"
[ -z "${DISPLAY:-}" ] && warn "DISPLAY 为空，xdotool 发不了键（请在桌面终端里跑）"
WID=""
DAE_PID=""
for i in $(seq 1 20); do
  DAE_PID="$(pgrep -f 'target/release/daedalus' 2>/dev/null | head -1)"
  # 三种找法，优先级：按进程 → 按标题 → 按窗口类
  [ -n "$DAE_PID" ] && WID="$(xdotool search --pid "$DAE_PID" 2>/dev/null | head -1)"
  [ -z "${WID:-}" ] && WID="$(xdotool search --onlyvisible --name "daedalus" 2>/dev/null | head -1)"
  [ -z "${WID:-}" ] && WID="$(xdotool search --onlyvisible --class "daedalus" 2>/dev/null | head -1)"
  [ -n "${WID:-}" ] && break
  sleep 1
done
if [ -n "${WID:-}" ]; then
  xdotool windowactivate --sync "$WID" 2>/dev/null || true
  xdotool windowfocus --sync "$WID" 2>/dev/null || true
  ok "窗口 ID=$WID（尝试 $xtry 次）"
else
  warn "没找到 daedalus 窗口，按键注入可能无效（场景会变成静止）"
fi

# ------------------------------- 场景动作 -----------------------------------
# 按键表（AGENTS.md）：假人移动 I J K L / 假人底盘旋转 U O / 假人云台 F V C B
#                      切换假人 Tab / 己方移动 W A S D / 视角 F3
# 注意：这些是“按实际画面调”的初始版本，录完必须抽查关键帧；见末尾提示。
keydown() { xdotool keydown "$@" 2>/dev/null || true; }
keyup()   { xdotool keyup   "$@" 2>/dev/null || true; }

scenario_A() {   # 静止：什么都不按（默认假人对枪状态）
  sleep "$DUR"
}
scenario_B() {   # 匀速平移：假人横移
  keydown j; sleep "$DUR"; keyup j
}
scenario_C() {   # 自转 + 平移：底盘旋转 + 横移
  keydown u; keydown j; sleep "$DUR"; keyup u; keyup j
}
scenario_D() {   # 移出视野再回来（近似遮挡后的丢失/重捕获）
  local t1=$((DUR/2)); local rest=$((DUR-t1-2))
  [ "$rest" -lt 1 ] && rest=1
  keydown j; sleep "$t1"; keyup j
  sleep 2
  keydown l; sleep "$rest"; keyup l
}
scenario_E() {   # 大角度斜视：横移 + 假人云台偏转
  keydown l; keydown b; sleep "$DUR"; keyup l; keyup b
}

run_scene() {
  local name="$1" desc="$2" fn="$3"
  local out="$BAGS_ROOT/scene$name"
  if [ -d "$out" ]; then
    mv "$out" "$out.old.$(date +%s)"
    warn "旧的 scene$name 已改名保留"
  fi
  say "录场景 $name：$desc（${DUR}s，zstd）"
  ros2 bag record --compression-mode file --compression-format zstd \
    -o "$out" "${TOPLIST[@]}" >"$HOME/bag_scene$name.log" 2>&1 &
  local rec=$!
  sleep 1
  "$fn"
  sleep 1
  kill -INT "$rec" 2>/dev/null || true
  wait "$rec" 2>/dev/null || true
  sleep 1

  if [ -d "$out" ]; then
    local sz
    sz="$(du -sh "$out" 2>/dev/null | cut -f1)"
    ok "scene$name 录完：$sz"
    ros2 bag info "$out" 2>/dev/null | sed -n '1,14p' | sed 's/^/    /'
  else
    warn "scene$name 没生成 bag 目录，看 $HOME/bag_scene$name.log"
  fi
}

# ------------------------------- 5. 逐场景录 ---------------------------------
say "5/6 开始录：$SCENES（每个 ${DUR}s）"
for s in $SCENES; do
  case "$s" in
    A) run_scene A "静止"            scenario_A ;;
    B) run_scene B "匀速平移"        scenario_B ;;
    C) run_scene C "自转+平移"       scenario_C ;;
    D) run_scene D "移出再回来"      scenario_D ;;
    E) run_scene E "大角度斜视"      scenario_E ;;
    *) warn "未知场景 $s，跳过" ;;
  esac
done

# ------------------------------- 6. 汇总 ------------------------------------
say "6/6 汇总"
echo "    bag 目录：$BAGS_ROOT"
df -h "$BAGS_ROOT" | sed 's/^/    /'
echo
for s in $SCENES; do
  [ -d "$BAGS_ROOT/scene$s" ] && printf "    scene%s  %s\n" "$s" "$(du -sh "$BAGS_ROOT/scene$s" | cut -f1)"
done

cat <<EOF

  下一步检查：
    1) 抽查画面（确认假人真的按场景在动）：
         ros2 bag info $BAGS_ROOT/sceneB     # 看 image_raw 消息数
    2) 回放看话题：
         ros2 bag play $BAGS_ROOT/sceneB
         ros2 run rm_vision_probe image_probe
    3) 想恢复不限帧（打游戏/测极限帧率用）：
         cp $CFG.pre_bag $CFG
EOF
