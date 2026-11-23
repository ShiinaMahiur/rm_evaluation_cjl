# 补丁 7（新增，2026-11-23）：ROS2 诊断日志输出命中统计

## 为什么

S4 验收需要「命中率」这个硬指标，但仿真器原来的 `ProjectileStatistics`
（`shot` 累计发射数、`hit` 累计命中数）只显示在屏幕左上角文字里，
ROS 侧拿不到 → 没法量化。这个补丁把统计塞进已有的 `[diag]` 日志（每 2 s 一行）。

## 改动

文件：`src/ros2/plugin.rs`

1. 顶部加 `use crate::statistic::ProjectileStatistics;`
2. `diag_ros2()` 增加参数 `stats: Res<ProjectileStatistics>`
3. `info!` 格式串末尾追加 `shots={} hits={} pct={:.3}`，并传入
   `stats.launch_count / stats.accurate_count / stats.accurate_pct()`

diff 摘要：

```diff
+use crate::statistic::ProjectileStatistics;
@@ fn diag_ros2(...)
+    stats: Res<ProjectileStatistics>,
 ) {
-        "[diag] autoaim={} controlled_gimbal={} all_infantry_gimbal={} controlled={} trackers={}",
+        "[diag] autoaim={} ... trackers={} shots={} hits={} pct={:.3}",
         ...
+        stats.launch_count,
+        stats.accurate_count,
+        stats.accurate_pct(),
     );
```

## 生效方式

```bash
cd ~/projects/bevy_robomaster_simulator
cargo build --release --features ros2      # 注意必须带 --features ros2，否则 ROS2 关掉
DAEDALUS_BACKEND=x11 ~/run_daedalus.sh
grep '\[diag\]' ~/sim.log | tail -1
# [diag] ... shots=44 hits=44 pct=1.000
```

## 命中判定的口径（源码）

`src/robomaster/armor/collision.rs: handle_armor_collision`
—— 弹丸碰撞体与任意 `Armor` 碰撞结束时 `stats.increase_accurate()` 一次，
同一发弹丸只计一次。因此 `pct = hit / shots` 就是**弹丸级命中率**（含误中其他装甲板）。
