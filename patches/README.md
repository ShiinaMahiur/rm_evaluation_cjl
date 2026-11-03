# Daedalus 补丁说明

本项目使用 [Daedalus](https://github.com/Blackjack200/bevy_robomaster_simulator)
作为仿真环境，**不复制其源码**（AGPL v3）。

Daedalus 作者只测过 `talos` feature，`--features ros2` 这条编译路径有 6 处问题，
必须打补丁才能编译通过。本目录提供一键补丁脚本，保证环境可复现。

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

## 6 处修改

| # | 文件 | 问题 | 修法 |
|---|---|---|---|
| 1 | `src/main.rs` | `should_enable_talos_plugin(_app: &App)` 但 ros2 cfg 块内用 `app` | 参数改名 `app` |
| 2 | `src/main.rs` | `SubscribeAutoAim(AtomicBool::new(false))` 默认关闭 | 改为 `true` |
| 3 | `src/ros2/plugin.rs` | `TextureFormat::bevy_default()` 在 bevy 0.19 不存在 | 改 `Rgba8Utf8Srgb`（见脚本实际值 `Rgba8UnormSrgb`） |
| 4 | `src/ros2/plugin.rs` | `capture_rune` 中 `one().unwrap()` 场景加载竞态 panic（2 处） | 改为 `match { None => continue }`；`center.get()` 返回 `Result` |
| 5 | `src/systems/controller.rs` | `auto_aim_active()` 每帧用控制器状态覆盖 `SubscribeAutoAim`，两者默认 false → 外部 ROS2 节点驱动不了云台 | 强制返回 `true` |
| 6 | `config.toml` | 默认开阴影/FXAA/preview/livox，帧率低 | 全关（见文件） |

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

## 协议说明

Daedalus 为 AGPL v3。本项目仅以"调用外部程序 + 提供补丁"的方式使用，
不包含、不修改、不二次分发其源码，补丁脚本本身为独立可复现工具。
