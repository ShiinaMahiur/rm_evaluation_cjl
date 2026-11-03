#!/usr/bin/env bash
# =============================================================================
#  run_daedalus.sh — 原生 Ubuntu 22.04 版（替代 WSL2/WSLg 版）
# -----------------------------------------------------------------------------
#  和 WSL 版的三个关键不同：
#   1) WSLg 的运行时目录 /mnt/wslg/runtime-dir 在原生 Ubuntu 上不存在；
#      原生是 /run/user/$UID（logind 提供）。这里自动取，不写死。
#   2) WSL 里 Vulkan 只有 llvmpipe（CPU 渲染，6.7 FPS），不用选设备；
#      原生有 Intel + NVIDIA 两个 ICD，不显式指定可能选到核显/软件渲染。
#      所以这里强制 VK_ICD_FILENAMES 指向 NVIDIA。
#   3) WSLg 只有 Wayland；原生 Wayland 下用独显偶有兼容问题，
#      失败自动回退 XWayland（WSL 版那个 5 次重试是等 WSLg socket，不是这件事）。
#
#  用法:
#    ./run_daedalus.sh                # 正常启动
#    ./run_daedalus.sh --some-arg     # 参数原样传给 daedalus
#    REPO=/path/to/repo ./run_daedalus.sh
# =============================================================================
set -uo pipefail

REPO="${REPO:-$HOME/projects/bevy_robomaster_simulator}"
BIN="$REPO/target/release/daedalus"
UID_NUM="$(id -u)"
RT="${XDG_RUNTIME_DIR:-/run/user/$UID_NUM}"
NVIDIA_ICD="/usr/share/vulkan/icd.d/nvidia_icd.json"

log() { echo "[run_daedalus] $*" >&2; }

[ -d "$REPO" ] || { log "找不到仓库：$REPO"; exit 1; }
[ -x "$BIN" ] || { log "找不到可执行文件：$BIN（先跑 setup_env.sh）"; exit 1; }

# ---------------------------------------------------------------------------
# 0) 原生会话环境兜底（在桌面的终端里一般已经设好，这里只防 ssh/异常情况）
# ---------------------------------------------------------------------------
export XDG_RUNTIME_DIR="$RT"

# ---------------------------------------------------------------------------
# 1) 强制 Vulkan 走 NVIDIA（原生特有：WSL 里只有一个 llvmpipe，不用管）
# ---------------------------------------------------------------------------
if [ -f "$NVIDIA_ICD" ]; then
  export VK_ICD_FILENAMES="$NVIDIA_ICD"
else
  log "[!] 找不到 $NVIDIA_ICD，回退到自动选择（可能选到核显）"
fi
export WGPU_BACKEND="${WGPU_BACKEND:-vulkan}"   # 别让 bevy 落到 GL 后端

# ---------------------------------------------------------------------------
# 2) ROS / Rust / 动态库路径
#    注意：本脚本开了 set -u，而 ROS 的 setup.bash 会引用未定义变量
#    （如 AMENT_TRACE_SETUP_FILES），直接 `. setup.bash` 会让脚本当场退出。
#    所以每次 source 都必须临时关掉 set -u —— 这是原生/WSL 都会踩的坑。
# ---------------------------------------------------------------------------
ros_source() {
  local f
  for f in "$@"; do
    [ -f "$f" ] || continue
    set +u
    # shellcheck disable=SC1090
    . "$f" >/dev/null 2>&1
    set -u
    log "已 source: $f"
  done
}

[ -f "$HOME/.cargo/env" ] && { set +u; . "$HOME/.cargo/env" >/dev/null 2>&1; set -u; }

ros_source /opt/ros/humble/setup.bash \
           "$HOME/ros2_ws/install/setup.bash" \
           "$HOME/vision_ws/install/setup.bash"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  log "[!] 没有 /opt/ros/humble/setup.bash——ROS 环境缺失，仿真器起来了也发不了话题"
fi

SYSROOT="$(rustc --print sysroot 2>/dev/null || true)"
if [ -n "$SYSROOT" ]; then
  export LD_LIBRARY_PATH="$REPO/target/release/deps:$SYSROOT/lib:$SYSROOT/lib/rustlib/x86_64-unknown-linux-gnu/lib:${LD_LIBRARY_PATH:-}"
else
  export LD_LIBRARY_PATH="$REPO/target/release/deps:${LD_LIBRARY_PATH:-}"
fi

# release 版要在 exe 旁边找 assets（原项目的老坑，软链幂等）
ln -sfn "$REPO/assets" "$REPO/target/release/assets" 2>/dev/null || true

# ---------------------------------------------------------------------------
# 3) 选显示后端：Wayland 优先 → XWayland 回退
#    说明：WSLg 只有 wayland-0；原生两种都可能。Wayland 下 wgpu 用独显偶发
#    面创建失败，所以保留一次 XWayland 重试。
# ---------------------------------------------------------------------------
have_wayland=0
[ -S "$RT/wayland-0" ] && have_wayland=1

have_x11=0
for xsock in /tmp/.X11-unix/X0 /tmp/.X11-unix/X1; do
  [ -S "$xsock" ] && { have_x11=1; break; }
done

# 可以用 DAEDALUS_BACKEND=x11 强制走 XWayland。
# 为什么要这个开关：录场景时要用 xdotool 合成按键（假人移动），而 Wayland 下
# xdotool 无法给原生 Wayland 窗口发键，只有 XWayland 窗口可以。
MODE_OVERRIDE="${DAEDALUS_BACKEND:-}"
MODES=()
if [ -n "$MODE_OVERRIDE" ] && [ "$MODE_OVERRIDE" != "auto" ]; then
  case "$MODE_OVERRIDE" in
    wayland|x11) MODES=("$MODE_OVERRIDE") ;;
    *) log "[!] DAEDALUS_BACKEND=$MODE_OVERRIDE 不认识（可选 wayland/x11/auto），忽略" ;;
  esac
fi
if [ "${#MODES[@]}" -eq 0 ]; then
  [ "$have_wayland" -eq 1 ] && MODES+=("wayland")
  [ "$have_x11" -eq 1 ] && MODES+=("x11")
  [ "${#MODES[@]}" -eq 0 ] && MODES=("auto")
fi

# 启动前把实际 GPU 打出来，方便一眼确认不是核显
if command -v nvidia-smi >/dev/null 2>&1; then
  log "GPU: $(nvidia-smi --query-gpu=name,driver_version,utilization.gpu --format=csv,noheader 2>/dev/null || echo 'nvidia-smi 不可用')"
fi
log "Vulkan ICD: ${VK_ICD_FILENAMES:-<自动>}"
log "会话: WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<无>} DISPLAY=${DISPLAY:-<无>} XDG_RUNTIME_DIR=$RT"

cd "$REPO" || { log "进不去 $REPO"; exit 1; }

for mode in "${MODES[@]}"; do
  case "$mode" in
    wayland) export WAYLAND_DISPLAY="wayland-0"; unset DISPLAY ;;
    x11)     unset WAYLAND_DISPLAY
             export DISPLAY="${DISPLAY:-:0}"
             export __GLX_VENDOR_LIBRARY_NAME=nvidia ;;
    auto)    : ;;
  esac
  log "=== 启动 daedalus（后端：$mode）==="
  "$BIN" "$@"
  rc=$?
  case "$rc" in
    0)   exit 0 ;;
    130|143) log "被中断（rc=$rc），不重试"; exit "$rc" ;;
  esac
  log "=== $mode 后端退出 rc=$rc ==="
done

log "两种后端都失败了，把上面的输出发我排查"
exit 1
