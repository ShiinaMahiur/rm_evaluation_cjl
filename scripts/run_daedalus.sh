#!/bin/bash
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
. "$HOME/.cargo/env"
source /opt/ros/humble/setup.bash
source /home/chen/ros2_ws/install/setup.bash
REPO=/home/chen/projects/bevy_robomaster_simulator
cd "$REPO" || exit 1
SYSROOT=$(rustc --print sysroot)
export LD_LIBRARY_PATH="$REPO/target/release/deps:$SYSROOT/lib:$SYSROOT/lib/rustlib/x86_64-unknown-linux-gnu/lib:${LD_LIBRARY_PATH:-}"

pick_rt() {
  for i in $(seq 1 40); do
    if [ -S /run/user/1000/wayland-0 ]; then
      export XDG_RUNTIME_DIR=/run/user/1000
      echo "XDG_RUNTIME_DIR=/run/user/1000 (attempt $i)" >&2
      return 0
    fi
    if [ -S /mnt/wslg/runtime-dir/wayland-0 ]; then
      export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
      echo "XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir (attempt $i)" >&2
      return 0
    fi
    sleep 1
  done
  echo "no wayland socket found, trying anyway" >&2
  export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
}

for attempt in 1 2 3 4 5; do
  pick_rt
  echo "=== daedalus attempt $attempt (XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR) ===" >&2
  "$REPO/target/release/daedalus"
  code=$?
  if [ "$code" -eq 0 ]; then
    exit 0
  fi
  echo "=== attempt $attempt exited $code, retry in 6s ===" >&2
  sleep 6
done
exit 1
