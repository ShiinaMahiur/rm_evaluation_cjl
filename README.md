# RoboMaster 视觉自瞄系统 · 26 秋考核

> 合肥工业大学 RoboMaster WDR 战队 · 算法组考核项目
> 作者：ShiinaMahiur
> 提交日期：2026-11（11/28 截止）

基于 [Daedalus 仿真器](https://github.com/Blackjack200/bevy_robomaster_simulator)
构建的完整自瞄流水线：

```
传感器图像 → 识别 → 观测器 → 估计器 → 规划器 → 控制/发射
           (是什么)  (量在哪)  (它在怎么动)  (怎么打)
```

---

## 目录

- [1. 环境与复现](#1-环境与复现)
- [2. 整体架构](#2-整体架构)
- [3. 各阶段方案选型与理由](#3-各阶段方案选型与理由)
- [4. 参数含义与整定方法](#4-参数含义与整定方法)
- [5. 实测指标](#5-实测指标)
- [6. 已知失败场景与原因分析](#6-已知失败场景与原因分析)
- [7. 实验日志](#7-实验日志)

---

## 1. 环境与复现

### 1.1 硬件

| 项 | 配置 |
|---|---|
| 机器 | 联想拯救者 Y9000P IRX11 |
| CPU | Intel i9-14900HX（24C32T） |
| GPU | **NVIDIA RTX 5060 Laptop（8 GB，Blackwell）** |
| 系统 | **Ubuntu 22.04.5 LTS（原生双系统）** |
| 屏幕 | 2560×1600，支持 60~240 Hz |

### 1.2 关键环境

```bash
# 驱动（Blackwell 必须用 open 内核模块，专有版会 RmInitAdapter failed）
sudo apt install -y nvidia-driver-580-open vulkan-tools

# ROS 2 Humble
sudo apt install -y ros-humble-desktop
source /opt/ros/humble/setup.bash

# Rust（编译 Daedalus）
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### 1.3 构建与运行

```bash
# 1. 克隆 Daedalus 并打补丁（补丁见 patches/）
git clone https://github.com/Blackjack200/bevy_robomaster_simulator.git
cd bevy_robomaster_simulator
python3 ../patches/apply_patches.py .

# 2. 编译（必须带 ros2 feature，默认的 talos feature 不对）
cargo build --release --features ros2

# 3. 跑仿真
./target/release/daedalus

# 4. 跑本项目的节点
cd <本仓库>
colcon build --symlink-install
source install/setup.bash
ros2 launch rm_vision_probe vision.launch.py
```

> **⚠️ 不要在 NTFS 分区上 `colcon build`** —— 符号链接与权限问题会导致构建失败。
> 请拷到 ext4 本地盘再编译。

---

## 2. 整体架构

### 2.1 数据流

```
/image_raw (1440×1080, rgb8, 278~293 Hz)
    ↓
[识别 S1]  → ArmorObs[]（4 角点 + 标签 + 颜色 + 置信度）
    ↓
[观测器 S2] → 相机系 PnP → TF 变换 → odom 系位姿
    ↓
[估计器 S3] → EKF 8 状态 → 预测 t+Δt 位置与 yaw
    ↓
[规划器 S4] → 弹道解算 → pitch/yaw + fire_advice
    ↓
/rm_gimbal/cmd (yaw/pitch 单位：**度**)
```

### 2.2 输出契约

```python
ArmorObs(
    corners: np.ndarray,   # shape (4,2)，顺序固定：左上→右上→右下→左下
    label:   str,          # 'sentry'/'1'/'2'/'3'/'4'/'5'/'outpost'/'base'
    color:   str,          # 'red'/'blue'
    score:   float,        # 置信度
)
```

### 2.3 节点划分

| 包 | 节点 | 作用 |
|---|---|---|
| `rm_vision_probe` | `image_probe` | S0 最小节点：订阅图像，打印分辨率与帧率 |
| `rm_vision_probe` | `gimbal_cmd` | S0 手动云台：发布 `/rm_gimbal/cmd` |
| `rm_vision_probe` | `armor_detector` | S1 识别（待实现） |
| `rm_vision_probe` | `armor_solver` | S2 观测器（待实现） |
| `rm_vision_probe` | `armor_tracker` | S3 估计器（待实现） |
| `rm_vision_probe` | `trajectory_planner` | S4 规划器（待实现） |

---

## 3. 各阶段方案选型与理由

> 每一层都必须回答"为什么选这个方案"，而不是"我调通了"。

### 3.1 识别（S1）

**必须对比两条路线，给出量化结论。**

| 维度 | 路线 A：传统灯条几何 | 路线 B：深度学习（YOLO） |
|---|---|---|
| 方法 | HSV 阈值 → 轮廓 → 灯条筛选 → 配对 → 分类 | YOLOv8/v11 检测角点或框 + 分类 |
| 准确率 | ⬜ 待测（目标 ≥ 0.90） | ⬜ 待测（目标 mAP@0.5 ≥ 0.85） |
| 单帧耗时（CPU, 1440×1080） | ⬜ 待测（目标 ≤ 8 ms） | ⬜ 待测（目标 ≤ 8 ms） |
| 光照敏感度 | ⬜ 待测 | ⬜ 待测 |
| 遮挡敏感度 | ⬜ 待测 | ⬜ 待测 |
| 标注成本 | 零 | 高（需手工/半自动标注） |
| 可解释性 | 高（中间结果可视） | 低（黑盒） |

**选型理由：** ⬜ 待填（做完对比后写）

**中间结果可视化：**

> 考核硬性要求：「**看不到中间结果的识别代码不算完成**」
> 必须提供二值化图、候选灯条、配对结果的调试图。

⬜ 待补（`media/s1_debug_*.png`）

---

### 3.2 观测器（S2）

**坐标系链路：**

```
camera_optical_frame → camera_link → gimbal_link → odom
```

**PnP 方法选择：**

| 方法 | 优点 | 缺点 | 选用？ |
|---|---|---|---|
| `SOLVEPNP_IPPE_SQUARE` | 针对正方形平面靶优化，快 | 只适用正方形 | ⬜ |
| `SOLVEPNP_ITERATIVE` | 通用，精度高 | 慢，可能局部最优 | ⬜ |
| `SOLVEPNP_SQPNP` | 闭式解，快且稳 | 对噪声敏感 | ⬜ |

**选型理由：** ⬜ 待填

**时间同步：**

> 图像时间戳与 TF 时间戳不同步会引入角度误差。
> 用 `tf2.lookup_transform(target, source, timestamp)` 指定时间，**不要用 `latest()`**。

⬜ 待实现

---

### 3.3 估计器（S3）

**状态向量（8 维）：**

```
x = [px, py, pz, vx, vy, vz, yaw, vyaw]
```

| 维 | 物理含义 | 为什么需要 |
|---|---|---|
| px, py, pz | odom 系位置 | 基础状态 |
| vx, vy, vz | 速度 | 提前量预测 |
| **yaw** | 目标朝向 | 多装甲板切换时确定"哪块板朝向相机" |
| **vyaw** | 转速 | 预判下一块装甲板出现的时刻 |

**滤波器选择：** ⬜ 待填（KF / EKF / IMM）

**Q/R 标定方法：**

> 考核要求：「**不允许凭感觉填 Q、R**」

⬜ 待填（拟用静止/匀速数据估计噪声 + 残差一致性检验）

**多装甲板观测关联：** ⬜ 待填（最近邻 / 匈牙利匹配）

---

### 3.4 规划器（S4）

**弹道参数：**

| 项 | 值 |
|---|---|
| 弹丸初速 | **25 m/s** |
| 重力 | 9.81 m/s² |
| 空气阻力 | 关闭 |

**两种弹道补偿方法对比：**

| 方法 | 原理 | 优点 | 缺点 |
|---|---|---|---|
| 恒定重力补偿近似 | `pitch ≈ atan(h/d) + g·t²/(2v²)·...` | 快，闭式解 | 远距误差大 |
| 迭代/数值解 | 迭代求解飞行时间与提前量 | 精确 | 需迭代，慢 |

**提前量的不动点问题：**

```
飞行时间 t = f(距离)
距离 d = g(预测位置(t))     ← 循环依赖
```

⬜ 待填（迭代收敛 / 解析近似）

**开火判据：** ⬜ 待填（角度误差阈值 + 角速度阈值 + 置信度）

---

### 3.5 能量机关（S5，加分）

⬜ 时间允许再做

---

## 4. 参数含义与整定方法

> 考核要求：「每个参数都要能说出为什么是这个值」

**所有参数外置在 `config/`，带注释。**

| 参数文件 | 内容 |
|---|---|
| `config/detector.yaml` | 识别阈值、颜色范围、灯条几何约束 |
| `config/solver.yaml` | 相机内参、装甲板物理尺寸、PnP 方法 |
| `config/tracker.yaml` | Q/R 矩阵、关联阈值、丢失判定时间 |
| `config/planner.yaml` | 弹道参数、开火阈值、延迟补偿 |

⬜ 待填（每个参数的含义 + 整定过程 + 最终取值依据）

---

## 5. 实测指标

### 5.1 环境验证（已实测）

| 指标 | WSL2（对照） | **原生 Ubuntu** | 备注 |
|---|---|---|---|
| `/image_raw` 帧率 | 6.3 Hz | **278~293 Hz** | `[window] present_mode = "auto_no_vsync"` |
| 渲染设备 | llvmpipe（CPU） | **RTX 5060 / Vulkan / DiscreteGpu** | 独显利用率 20~38% |

**测量方法：**
```bash
ros2 topic hz /image_raw
nvidia-smi --query-gpu=name,utilization.gpu --format=csv
```

### 5.2 考核指标（待填）

| 代号 | 指标 | 目标 | 实测 | 测量方法 |
|---|---|---|---|---|
| I1 | 识别准确率 | ≥ 0.90（传统）/ mAP@0.5 ≥ 0.85（DL） | ⬜ | 自建测试集 |
| I2 | 识别耗时 | ≤ 8 ms（CPU 1440×1080） | ⬜ | 节点内计时（不含绘图） |
| I3 | 解算精度 | 距离误差 ≤ 5%，角误差 ≤ 1° | ⬜ | 与 `/tf` 的 `armor_<id>` 真值对拍 |
| I4 | 解算稳定性 | 无跳变/翻转 | ⬜ | 大角度斜视场景目视 + 曲线 |
| I5 | 预测精度 | ≤ 0.08 m @ 100 ms（I5 定义写 200 ms，两个都测） | ⬜ | 预测值与真值对拍 |
| I6 | 丢失恢复 | 判定 ≤ 30 ms，重捕获 ≤ 50 ms | ⬜ | 计时统计 |
| I7 | 首发命中率 | ≥ 40% | ⬜ | 场景 A~E 汇总 |
| I8 | 连续命中率 | ≥ 60% | ⬜ | 匀速平移 + 自转场景 |
| I10 | 端到端帧率 | ≥ 30 FPS | ⬜ | 图像时间戳 → 指令时间戳 |

**指标 JSON 自动输出：**

⬜ 待实现（节点输出 `metrics/report_*.json`）

```json
{
  "fps": 0.0,
  "detect_ms": 0.0,
  "solve_error_m": 0.0,
  "lost_count": 0,
  "hit_rate": 0.0
}
```

---

## 6. 已知失败场景与原因分析

### 6.1 环境类

| 现象 | 原因 | 解决 |
|---|---|---|
| `nvidia-smi` 失败，dmesg 报 `RmInitAdapter failed` | RTX 50 系（Blackwell）必须用 **open 内核模块** | `apt install nvidia-driver-580-open` |
| `preview=false` 时窗口全黑 | `--features ros2` 下主相机 `is_active=false`，画面靠 `setup_preview_window` blit 采集纹理 | 想看画面必须临时开 `preview=true`（帧率减半） |
| `colcon build` 在 NTFS 上失败 | 符号链接/权限不支持 | 拷到 ext4 本地盘编译 |
| 深度图分辨率调太小导致 `/image_raw` 消失 | 采集链路整个不启动 | 保持 1280×720 |

### 6.2 算法类

⬜ 待填（每完成一层，记录已知失败场景）

---

## 7. 实验日志

> 考核明确：「记录你的实验。每次改动的假设、改动内容、指标变化，写成简单的实验日志。」

| 日期 | 改动 | 指标变化 | 结论 |
|---|---|---|---|
| 2026-11-02 | 双系统 + open 驱动 | `/image_raw` 6.3 Hz → **278~293 Hz** | WSL 的 Vulkan 缺失是根本瓶颈，硬件升级解决 |
| ⬜ | ⬜ | ⬜ | ⬜ |

---

## 8. 目录结构

```
rm_evaluation_cjl/
├── README.md                  ← 本文件（评分重点）
├── vision_ws/                 ← ROS 2 功能包
│   └── src/rm_vision_probe/
├── ros2_src/rm_interfaces/    ← r2r 编译所需的接口包
├── patches/                   ← Daedalus 补丁（保证可复现）
│   ├── apply_patches.py
│   ├── config.toml
│   └── README.md
├── config/                    ← 所有参数外置（带注释）
├── metrics/                   ← 指标 JSON（节点自动生成）
├── media/                     ← 调试图、录屏（大文件走外链）
├── scripts/                   ← 一键运行脚本
└── docs/                      ← 场景说明、方法论文档
```

---

## 9. 参考

- [Daedalus 仿真器](https://github.com/Blackjack200/bevy_robomaster_simulator)（河北科技大学 Actor&Thinker 战队开源，AGPL v3）
- [RoboMaster 开发者社区](https://bbs.robomaster.com/)
- 《视觉 SLAM 十四讲》高翔
- 《A Tour of C++》Bjarne Stroustrup
- [动手学深度学习 (d2l)](https://zh.d2l.ai)

---

## 许可

本项目代码为考核作品。Daedalus 为 AGPL v3，本项目**不包含、不修改、不二次分发**其源码，
仅通过「外部调用 + 补丁脚本」的方式使用，补丁脚本本身为独立工具。
