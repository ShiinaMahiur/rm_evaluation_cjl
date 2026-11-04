# Daedalus 补丁说明

本项目使用 [Daedalus](https://github.com/Blackjack200/bevy_robomaster_simulator)
作为仿真环境，**不复制其源码**（AGPL v3）。

Daedalus 作者只测过 `talos` feature，`--features ros2` 这条编译路径有 6 处问题，
必须打补丁才能编译通过。本目录提供一键补丁脚本，保证环境可复现。

除 6 处编译修复外，脚本还包含 **2 处功能补丁**（手动云台开关、PID 调试日志），
都是可开关的，不影响默认行为。

## 使用方法

```bash
# 1. 克隆上游（固定 commit，保证可复现）
git clone https://github.com/Blackjack200/bevy_robomaster_simulator.git
cd bevy_robomaster_simulator

# 2. 打补丁（幂等，可重复运行）
python3 /path/to/patches/apply_patches.py .

# 3. 编译
source /opt/ros/humble/setup.bash
cargo build --release --features ros2
```

## 6 处编译修复 + 2 处功能补丁

| # | 文件 | 问题 | 修法 |
|---|---|---|---|
| 1 | `src/main.rs` | `should_enable_talos_plugin(_app: &App)` 但 ros2 cfg 块内用 `app` | 参数改名 `app` |
| 2 | `src/main.rs` | `SubscribeAutoAim(AtomicBool::new(false))` 默认关闭 | 改为 `true` |
| 3 | `src/ros2/plugin.rs` | `TextureFormat::bevy_default()` 在 bevy 0.19 不存在 | 改 `Rgba8Utf8Srgb`（见脚本实际值 `Rgba8UnormSrgb`） |
| 4 | `src/ros2/plugin.rs` | `capture_rune` 中 `one().unwrap()` 场景加载竞态 panic（2 处） | 改为 `match { None => continue }`；`center.get()` 返回 `Result` |
| 5 | `src/systems/controller.rs` | `auto_aim_active()` 每帧用控制器状态覆盖 `SubscribeAutoAim`，两者默认 false → 外部 ROS2 节点驱动不了云台 | 强制返回 `true` |
| 6 | `config.toml` | 默认开阴影/FXAA/preview/livox，帧率低 | 全关（见文件） |
| 7 | `src/systems/controller.rs` | 第 5 条强制常开后，**方向键控云台也一起失效**（`gimbal_controls()` 开头直接 return） | 加环境变量开关：`DAEDALUS_MANUAL_AIM=1` 时退回手动模式，默认仍常开 |
| 8 | `src/systems/gimbal_pid.rs` | 排查"发了 `/rm_gimbal/cmd` 云台不动"时看不到 PID 内部状态 | 加 `DAEDALUS_PID_DEBUG=1` 调试日志（目标/当前/误差/输出） |

### 两个功能开关（默认都不启用，不影响正常使用）

```bash
# 手动模式：方向键控制云台（调视角 / 录数据集 / 造考核场景时用）
DAEDALUS_MANUAL_AIM=1 ./target/release/daedalus

# 云台 PID 调试：打印目标角、当前角、误差、PID 输出速率
DAEDALUS_PID_DEBUG=1 ./target/release/daedalus
```

> 第 7 条的背景：补丁 5"强制常开"解决了外部节点驱动不了云台的问题，但副作用是
> `gimbal_controls()` 开头的 `if enabled { return; }` 会把**键盘方向键**也一起挡掉。
> 用环境变量做通道隔离后，两条路径都可用（默认仍保持 ROS 外部控制）。

### 第 5 条是最隐蔽的

只改 `SubscribeAutoAim` 的初始值**没有用**，因为
`update_auto_aim_subscription` 每帧都会用 `controller.auto_aim_active()` 覆盖它。
必须改 `auto_aim_active()` 本身。

## 性能数据（WSL2 + llvmpipe，仅供参考）

| 配置 | 帧率 |
|---|---|
| livox 开 | 4.53 FPS |
| livox 关 | 6.69 FPS |

**注意**：`[capture.depth]` 分辨率不要调太小（如 64x64），
会导致 ROS2 采集链路不启动，`/image_raw` 直接消失。保持 1280x720。

### 原生 Ubuntu 实测（RTX 5060 Laptop + Vulkan）

`/image_raw` **200~285 Hz**（`present_mode="auto_no_vsync"`，X11/XWayland 下 vsync 不生效），
`/camera_info`、`/tf` 同频。录 bag 采用「全速录 + message 级 zstd」（约 400 KB/帧）。

## 协议说明

Daedalus 为 AGPL v3。本项目仅以"调用外部程序 + 提供补丁"的方式使用，
不包含、不修改、不二次分发其源码，补丁脚本本身为独立可复现工具。
