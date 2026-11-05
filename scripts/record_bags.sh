#!/usr/bin/env bash
# =============================================================================
#  record_bags.sh — S0 数据集录制（原生 Ubuntu 22.04）
# -----------------------------------------------------------------------------
#  一次录 5 个场景：A 静止 / B 平移 / C 自转+平移 / D 移出再回来 / E 大角度斜视
#
#  它替你做这些事（每一条都是真机上会踩的坑）：
#   1. 挂 D 盘（NTFS），bag 直接写到 /media/$USER/RJ/rm_vision/bags
#   2. 关闭 preview；录 bag 用「全速 + zstd 压缩」（实测 ~48 KB/帧，
#      20 s 单场景约 200 MB，5 场景约 1 GB；不再强求 60 Hz）
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
#    bash record_bags.sh                       # 全速录 + zstd（推荐）
#
#  前提：
#    - 当前桌面会话是 X11（xdotool 只能给 X11/XWayland 窗口发按键）
#    - 已装 xdotool：sudo apt install -y xdotool
#    - 显示器刷新率不用管（录制端靠 zstd 压缩控制体积）
# =============================================================================
set -uo pipefail

# ------------------------------- 可调参数 -----------------------------------
SIM_PID=""
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
DRIVER="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scene_driver.py"

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

# ------------------------------- 2. 录制配置 --------------------------------
# 实测（2026-11-03）：X11/XWayland 后端下 present_mode 的 vsync 不生效，
# /image_raw 会跑到 ~230-285 Hz；与其跟渲染帧率较劲，不如"全速录 + zstd 压缩"：
#   实测 1253 帧 → 48 MB（约 38 KB/帧），20 s 单场景 ≈ 140 MB，5 场景 ≈ 0.7 GB。
#   用 **message 级压缩**（不是 file 级）：播放/抽帧时按消息解压，不会产生几十 GB 临时文件。
# 需要标准 60 Hz 数据集时，可改用 scripts/bag_throttle_record.py 限频录制。
# 车辆速度上限来自 config.toml 的 [vehicle] max_speed（仿真支持热重载）。
# 数字键控只有"全油门"，靠按键没法稳定 2 m/s；但把 max_speed 直接设成 2.0，
# 按住键就会停在 2 m/s——这是录 B 场景"匀速平移"的正解。
CFG_BAK="$CFG.bak.record"
[ -f "$CFG_BAK" ] || cp "$CFG" "$CFG_BAK"
set_vehicle_speed() {
  local v="$1"
  python3 - "$CFG" "$v" <<'PYV'
import re, sys
p, v = sys.argv[1], sys.argv[2]
s = open(p, encoding='utf-8').read()
s2 = re.sub(r'(?m)^max_speed\s*=\s*[0-9.]+', f'max_speed = {v}', s, count=1)
if s2 != s:
    open(p, 'w', encoding='utf-8').write(s2)
    print(f"  [vehicle] max_speed → {v} m/s")
PYV
}
restore_vehicle_speed() { [ -f "$CFG_BAK" ] && cp "$CFG_BAK" "$CFG" && rm -f "$CFG_BAK" || true; }

say "2/6 录制配置（关 preview；present_mode 恢复不限帧）"
grep -q '^present_mode = "auto_vsync"' "$CFG" && sed -i 's/^present_mode = "auto_vsync"/present_mode = "auto_no_vsync"/' "$CFG" && ok "present_mode → auto_no_vsync"
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
KEEP_SIM="${KEEP_SIM:-0}"
RESTART_PER_SCENE="${RESTART_PER_SCENE:-1}"   # 每个场景前重启仿真器（清空上一场把车开到边界的副作用）
if [ "$KEEP_SIM" = "1" ] && pgrep -x daedalus >/dev/null; then
  say "3/6 沿用已在运行的仿真器（KEEP_SIM=1，不重启、不杀它）"
  if [ -z "${SIM_PID:-}" ]; then
    SIM_PID="$(pgrep -x daedalus | head -1)"
    # KEEP_SIM 模式下不要把用户的仿真器杀掉
    trap - EXIT
    cleanup_sim() { :; }
    trap "cleanup_sim; restore_vehicle_speed" EXIT
  fi
else
  say "3/6 启动仿真器（XWayland 后端，xdotool 才能发按键）"
  pkill -x daedalus 2>/dev/null; sleep 1
  rm -f "$SIMLOG"
  DAEDALUS_BACKEND=x11 setsid "$LAUNCHER" >"$SIMLOG" 2>&1 </dev/null &
  SIM_PID=$!
  cleanup_sim() { kill -TERM -"$SIM_PID" 2>/dev/null || kill -TERM "$SIM_PID" 2>/dev/null || true
                  sleep 1; pkill -x daedalus 2>/dev/null || true; }
  trap "cleanup_sim; restore_xrepeat; restore_vehicle_speed" EXIT
fi

wait_image() {
  local W=0
  say "等待 /image_raw 出数据（最多 60 秒）"
  for i in $(seq 1 60); do
    W="$(timeout 3 ros2 topic echo /image_raw --once --field width 2>/dev/null | grep -oE '[0-9]+' | head -1)"
    [ "${W:-}" = "1440" ] && break
    sleep 1
  done
  [ "${W:-}" = "1440" ] || { warn "没等到 /image_raw（width=${W:-空}），日志 $SIMLOG 前 30 行："; head -30 "$SIMLOG" | sed 's/^/    /'; die "仿真器没起来"; }
  ok "/image_raw 宽度 = 1440"
}

find_and_focus_window() {
  local WID="" DAE_PID xtry=0
  for i in $(seq 1 20); do
    DAE_PID="$(pgrep -x daedalus 2>/dev/null | head -1)"
    xtry=$i
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
}

start_sim_fresh() {
  say "重启仿真器（每场景前清场，避免机器人被上一场开到边界）"
  pkill -x daedalus 2>/dev/null; sleep 1
  rm -f "$SIMLOG"
  DAEDALUS_BACKEND=x11 setsid "$LAUNCHER" >"$SIMLOG" 2>&1 </dev/null &
  SIM_PID=$!
  wait_image
  find_and_focus_window
}

say "量一下实际帧率（仅供参考，不再限帧）"
HZ="$(timeout 8 ros2 topic hz /image_raw 2>/dev/null | grep -oE 'average rate: [0-9.]+' | tail -1 | grep -oE '[0-9.]+')"
[ -n "${HZ:-}" ] && ok "当前 /image_raw ≈ ${HZ} Hz（zstd 压缩后单场景约 140 MB）" || warning "没量到帧率，继续"

# ------------------------------- 4. 窗口焦点 ---------------------------------
say "4/6 找到仿真窗口并聚焦（xdotool 发按键的前提）"
[ -z "${DISPLAY:-}" ] && warn "DISPLAY 为空，xdotool 发不了键（请在桌面终端里跑）"
find_and_focus_window

# ------------------------------- 场景动作 -----------------------------------
# ⚠️ 关闭 X11 键盘自动重复：xdotool keydown 按住时，auto-repeat 会周期性发
#    KeyRelease，导致仿真器以为键被松开 —— 实测场景只动 0.5 s 就停（旧 bag B 场景
#    速度只有 0.01 m/s 就是这个原因）。录完在 cleanup 里恢复。
XRESTORE=0
if command -v xset >/dev/null; then
  if xset q 2>/dev/null | grep -qi 'auto repeat:.*on'; then xset -r && XRESTORE=1 && ok "已关闭键盘自动重复（录完恢复）"; fi
fi
restore_xrepeat() { [ "${XRESTORE:-0}" = "1" ] && xset r on 2>/dev/null || true; }
add_cleanup_restore() { :; }
# 按键表（AGENTS.md）：假人移动 I J K L / 假人底盘旋转 U O / 假人云台 F V C B
#                      切换假人 Tab / 己方移动 W A S D / 视角 F3
# 注意：这些是“按实际画面调”的初始版本，录完必须抽查关键帧；见末尾提示。
keydown() { xdotool keydown "$@" 2>/dev/null || true; }
keyup()   { xdotool keyup   "$@" 2>/dev/null || true; }

# 闭环驱动（scene_driver.py）：读 /tf 实时测距离/速度，按键反馈
# 旧版用固定时长按键，实测 B 场景速度只有 0.01 m/s、A 只有 1.5~2.4 m 一档。
run_driver() { python3 "$DRIVER" "$@"; }

scenario_A() {   # 静止：三档距离 3 / 6 / 10 m，每档静止 DUR/3 秒
  local each=$((DUR / 3)); [ "$each" -lt 3 ] && each=3
  ok "A：闭环开到 3/6/10 m，每档静止 ${each}s（需要窗口聚焦）"
  # 仿真场地只有约 ±3.5 m，10 m 单向到不了；先录 3/6 m（6 m 靠「两头拉」实现）
  run_driver --mode distance --targets 3 6 --dwell "$each" --tol 0.18 \
    || warn "闭环驱动失败，退化为静止不动：$(sleep 1; echo ok)"
}
scenario_B() {   # 匀速平移：闭环控制目标速度 ~2 m/s（来回平移，保持不出画）
  ok "B：闭环控制目标速度 2 m/s，持续 ${DUR}s"
  run_driver --mode speed --target "${VEH_SPEED_B:-2.0}" --duration "$DUR" --span 2.2 \
    || warn "闭环驱动失败；手动退化为按住 J：$(keydown j; sleep "$DUR"; keyup j; echo ok)"
}
scenario_C() {   # 自转 + 平移：假人底盘自转（U 切换）+ 闭环 ~1.5 m/s 平移
  ok "C：假人底盘自转 + 闭环平移 1.5 m/s"
  xdotool key u 2>/dev/null || true
  sleep 0.5
  run_driver --mode speed --target 1.5 --duration "$DUR" --span 2.0 \
    || warn "闭环驱动失败；退化为按住 u+j：$(keydown u; keydown j; sleep "$DUR"; keyup u; keyup j; echo ok)"
  xdotool key u 2>/dev/null || true
}
scenario_D() {   # 丢失/重捕获：镜头转开 ~0.5s（目标出画）再转回
  # 考核要求遮挡 0.5 s；云台固定转速，按同长回转即可回到原朝向
  local away=${D_OCCL:-0.25}
  keydown Right; sleep "$away"; keyup Right
  sleep 0.3                       # 目标在画外停留 ~0.5 s
  keydown Left; sleep "$away"; keyup Left
  sleep $((DUR - 1 > 0 ? DUR - 1 : 1))
}
scenario_E() {   # 大角度斜视：先横移拉开视角，再让假人底盘自转（U）
  # 先侧移 ~2 m 增加相机→目标的视线与板法线夹角
  ok "E：侧移约 2 m → 假人自转 ${DUR}s"
  local side=${E_SIDE:-2.0}
  keydown a; sleep "$side"; keyup a
  xdotool key u 2>/dev/null || true
  sleep "$DUR"
  xdotool key u 2>/dev/null || true
}

run_scene() {
  local name="$1" desc="$2" fn="$3"
  local out="$BAGS_ROOT/scene$name"
  # 按场景设置车辆速度上限（A/B 用 2 m/s 才能稳定匀速；C 用 1.5）
  case "$name" in
    A) set_vehicle_speed "${VEH_SPEED_A:-2.0}" ;;
    B) set_vehicle_speed "${VEH_SPEED_B:-2.0}" ;;
    C) set_vehicle_speed "${VEH_SPEED_C:-1.5}" ;;
    *) set_vehicle_speed "${VEH_SPEED_OTHER:-6.0}" ;;
  esac
  if [ "${RESTART_PER_SCENE:-1}" = "1" ] && [ "${KEEP_SIM:-0}" != "1" ]; then
    start_sim_fresh
  fi
  if [ -d "$out" ]; then
    mv "$out" "$out.old.$(date +%s)"
    warn "旧的 scene$name 已改名保留"
  fi
  say "录场景 $name：$desc（${DUR}s，zstd）"
  ros2 bag record --compression-mode message --compression-format zstd \
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
restore_xrepeat
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
