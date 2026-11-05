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

- [1. 环境与复现](#1-环境与复现)（含 [1.4 一键脚本](#14-一键脚本s0-验收--日常调试)）
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

### 1.4 一键脚本（S0 验收 / 日常调试）

| 脚本 | 作用 | 对应考核要求 |
|---|---|---|
| **`scripts/replay_bag.sh`** | **一键回放 bag + 云台跟随**（自动起仿真器、起图像重心跟踪节点、回放） | ✅ **S0 验收项** |
| `scripts/run_all.sh` | 一键启动整套系统（仿真器 → 等待图像 → 视觉链路） | ✅ 提交要求「一键运行脚本」 |
| `scripts/sim.sh` | 仿真器一键启动/停止/状态（`--preview` 看画面、`--manual` 方向键控云台） | 日常调试 |
| `scripts/follow_target.py` | 图像重心 + P 控制 → `/rm_gimbal/cmd`（无识别阶段的云台跟随闭环） | S0 链路验证 |
| `scripts/record_bags.sh` | 录 S0 数据集（5 场景 × 20 s，message 级 zstd） | 数据集采集 |
| `scripts/bag_view.sh` | 回放 bag 并显示图像窗口（不用 rqt 也行） | 看数据 |
| `scripts/bag_extract.py` | 压缩 bag 抽帧成 JPG（`SequentialCompressionReader`） | 验证数据 |
| `scripts/gimbal_diag.py` | 云台指令通路诊断（发绝对角度 ↔ 读 `/gimbal_pose`） | 排障 |

```bash
# S0 验收演示：一键回放 + 云台跟随
~/replay_bag.sh /media/$USER/RJ/rm_vision/bags/sceneB

# 只回放不驱动云台
NO_FOLLOW=1 ~/replay_bag.sh /media/$USER/RJ/rm_vision/bags/sceneB

# 仿真器一键启动（可选：看画面 / 手动控云台 / 停止 / 状态）
~/sim.sh                        # 正常启动（后台）
~/sim.sh --preview --manual     # 看画面 + 方向键控云台
~/sim.sh --status               # 看状态
~/sim.sh --stop                 # 停止

# 整套系统一键启动
~/run_all.sh

# 云台指令通路诊断（排查 /rm_gimbal/cmd）
python3 ~/gimbal_diag.py
```

> 说明：`follow_target.py` 用「画面里最亮彩色块的重心」做 P 控制，目的是**先证明
> 视觉 → 云台的闭环链路通**，不依赖识别阶段。S1 之后替换为「识别 → 解算」的真值目标。

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
| `rm_vision_probe` | `detector_traditional` | **S1 传统灯条法**：`/detector/armors` + `/detector/debug_image` |
| （离线） | `scripts/make_yolo_dataset.py` + `eval_yolo.py` | **S1 深度学习路线**：真值自动标注 → YOLOv8n 训练/评估 |
| `rm_vision_probe` | `armor_solver` | **S2 观测器（已实现）**：PnP(IPPE) → `/solver/armors`（相机系/odom 系位姿） |
| `rm_vision_probe` | `armor_tracker` | S3 估计器（待实现） |
| `rm_vision_probe` | `trajectory_planner` | S4 规划器（待实现） |

---

## 3. 各阶段方案选型与理由

> 每一层都必须回答"为什么选这个方案"，而不是"我调通了"。

### 3.1 识别（S1）

**必须对比两条路线，给出量化结论。**

| 维度 | 路线 A：传统灯条几何 | 路线 B：深度学习（YOLO） |
|---|---|---|
| 方法 | HSV 阈值 → 灯条筛选 → 两两配对（几何 + 数字区校验） → 4 角点 | YOLOv8/v11 检测角点或框 + 分类 |
| 单帧耗时 | **7.44 ms**（CPU, 1440×1080 ≈134 FPS） | **21.7 ms（CPU）/ 7.6 ms（RTX 5060 GPU）** |
| 检测率（≥1 目标/图） | A/B/C 100%，D 83.4%，**E 18.3%** | **验证集 100%**（conf≥0.25） |
| 光照敏感度 | 强光/过曝会让灯条糊成一片（阈值法通病） | 待测 |
| 遮挡敏感度 | 单灯条被挡住即无法配对（D 场景 -16.6%） | 待测 |
| 标注成本 | 零 | 高（需手工/半自动标注） |
| 可解释性 | 高（灯条/配对/角点全部可视） | 低（黑盒） |

**当前实现（路线 A，已完成第一版）**

```
/image_raw (1440×1080 rgb8)
   ↓ HSV 颜色阈值（蓝 90<H<140 / 红 H<12或H>168，V>110，S>90）
二值图 → 轮廓 → 灯条筛选（高度≥8px、高宽比≥1.5、填充率≥0.25）
   ↓ 两两配对
几何约束：两灯高度差≤35%、中心 y 差≤55%、灯间距/平均高 ∈ [0.6, 3.0]
   ↓ 数字区校验（关键）
两个灯条之间必须有"低饱和中等亮度"的数字像素（≥4%），
否则会把前哨站/建筑的两条装饰灯误配成装甲板（实测这是最大误检来源）
   ↓
装甲板：4 角点（左上→右上→右下→左下）+ 颜色标签 red/blue
```

**节点 & 话题**

| 项 | 值 |
|---|---|
| 节点 | `ros2 run rm_vision_probe detector_traditional` |
| 输入 | `/image_raw` |
| 输出 1 | `/detector/armors`（`std_msgs/String` JSON：4 角点、颜色、分数、耗时） |
| 输出 2 | `/detector/debug_image`（`sensor_msgs/Image`，灯条绿框 + 装甲板黄框 + 统计） |
| 离线评估 | `scripts/eval_detector.py`（在 A~E bag 上跑，输出 `metrics/s1_traditional.json`） |

**深度学习路线（YOLOv8n，2026-11-05 修正标注重训）**

- **标注来源：仿真真值自动标注**（`scripts/make_yolo_dataset.py`）
  用 bag 里的 `/tf`（`map -> armor_*`）+ `/camera_info`，把**真实板面**（0.15 m × 0.125 m）四角投影到图像
  - ⚠️ 旧版把 `/tf` 的 marker 中心当板中心、且用错板面平面 → 标注框画在装甲板**底部地面上**。
    2026-11-05 统一改走 `solver.plate_pose_from_marker()`（与 S2 真值同一套定义），并加「框内必须有灯条亮像素」的
    可见性过滤（剔除被遮挡/背面不可见的板）
  - 产出：1851 张图 / 10253 个框，**零人工标注成本**
- **数据划分：跨场景验证**——训练只用 A/B/C/D，**验证集是整段 E**（旧版是同段 bag 抽帧、指标虚高）
- 训练：`scripts/train_yolo.py`，`yolov8n.pt` 预训练权重，40 epochs，imgsz=640，batch=16，RTX 5060（**约 5 分钟**）
- 评估：`scripts/eval_yolo.py --per-scene` → `metrics/s1_yolo.json`；权重 `models/s1_yolo_v8n_best.pt`（6.2 MB）
- 数据集统计：`~/yolo_dataset/stats.json`（旧错误标注数据集备份在 `~/yolo_dataset_wrongplane_bak`）

**实测（跨场景验证 + 逐场景实例级）**

| 场景 | 图级检出率 | 实例召回 | 精度 |
|---|---|---|---|
| A 静止（训练） | 100% | 100% | 100% |
| B 平移（训练） | 100% | 99.95% | 99.47% |
| C 自转+平移（训练） | 100% | 88.7% | 79.7% |
| D 遮挡（训练） | 100% | 99.2% | 98.7% |
| **E 斜视（验证，跨场景）** | **100%** | **81.1%** | 80.3% |

| 指标 | 值 |
|---|---|
| mAP@0.5（跨场景 E） | **0.845** |
| mAP@0.5:0.95（跨场景 E） | 0.778 |
| P / R（跨场景 E） | 0.972 / 0.773 |
| 延迟（RTX 5060，含前后处理） | **6.9 ms**（GPU）/ 22.2 ms（CPU） |

> 对比传统法：E 场景实例检出从 **18.3% → 81.1%**（跨场景验证，不是同段 bag 自测）。C 场景精度偏低（79.7%）
> 是因为自转时侧板以窄条出现，模型会多给框（误检）；后续加 NMS/几何后处理或颜色分类可压。

**选型理由（对比完成后）**

- 先做传统法：**零标注成本 + 中间结果全可视**，能在 1~2 天内跑通全链路，
  先把 S1→S2 的数据契约（4 角点 + 颜色）定下来
- 传统法的硬边界已经量化：**大角度斜视（E）只有 18.3%**、遮挡时掉 16.6% ——
  这两个数字就是 M2 上深度学习的动机（不是"为了用 YOLO 而用"）

**中间结果可视化（考核硬性要求）**

- 调试图：`metrics/s1_samples/*.jpg`（每个场景 3 张，绿框=灯条、黄框=装甲板、角点顺序可见）
- 实时：`ros2 topic echo /detector/debug_image` 或 `rqt_image_view /detector/debug_image`

---

### 3.2 观测器（S2）

**链路：像素角点 → 标定展开 → PnP → 相机系位姿 → TF → odom 系位姿**

```
[识别 S1] 灯条矩形（亚像素）
   ↓  refine_corners()：
   │    ① 按 DET_SCALE_X/Y 把「灯条外沿矩形」展开成「装甲板外沿矩形」
   │    ② 使用两根灯条各自的亚像素宽高，保留倾斜
   ↓  cv2.solvePnP(OBJ_PTS, corners, K, flags=SOLVEPNP_IPPE)
相机系位姿 (R, t) + 重投影误差
   ↓  tf: camera_optical_frame → odom
odom 系位姿 → 交给 S3
```

**⚠️ 2026-11-05 修正了真值定义（这条影响所有历史数字）**

| 项 | 旧做法（错） | 新做法（对） |
|---|---|---|
| 板面平面 | marker 局部 y-z 平面 | marker 局部 **x-y 平面**（法向 z） |
| 真值参考点 | `/tf` marker 原点 | 板面中心 = marker 原点 + **4.24 cm**（marker 局部系） |
| 检测角点 | 直接按 0.15×0.125 解算 | 先按标定尺度展开到板外沿 |

依据来自仿真源码 `src/ros2/plugin.rs`（armor marker 是 0.03×0.15×0.125 立方体，且与 `/tf` 姿态差 `Rz(-90°)`）+ 数据拟合，见 `scripts/calib_s2_plate.py`。

**离线标定常数**（`metrics/s2_calib.json`，A/B 场景 184 对样本，四角残差 6.3 px）：

| 常数 | 值 | 含义 |
|---|---|---|
| `MARKER_TO_PLATE_D` | `[-0.0292, 0.0205, -0.0237] m` | marker 原点 → 可见板面中心（marker 局部系） |
| `DET_SCALE_X` | 1.0421 | 灯条外沿宽度 → 板宽 |
| `DET_SCALE_Y` | 2.2508 | 灯条高度 → 板高（灯条只有板高约 44%） |

**实测精度（A~E，每 20 帧取 1，与 `/tf` 板面真值对拍）**

| 场景 | GT 角点（自洽性） | 端到端 距离误差 | 端到端 视线角误差 | 关联成功率 |
|---|---|---|---|---|
| A 静止 | 0.000 m / 0.017° | **0.0115 m（0.77%）** | **0.041°** | 100%（193/193） |
| B 平移 | 0.000 m / 0.026° | 0.0234 m（1.58%） | 0.050° | 87.7%（186/212） |
| C 自转+平移 | 0.000 m / 0.027° | 0.0282 m（1.90%） | 0.127° | 73.9%（184/249） |
| D 遮挡 | 0.000 m / 0.038° | **检测失败**（166 帧只有 1 个误检/帧，0 关联） | — | 0%（0/166） |
| E 斜视 | 0.000 m / 0.040° | **检测失败**（41 检测，0 关联） | — | 0%（0/41） |

**消融实验（A 场景，同一套 GT）**

| 配置 | 距离误差 | 视线角误差 |
|---|---|---|
| 整像素角点 + 无标定 | 6.64% | 0.106° |
| 整像素角点 + 尺度标定 | 6.60% | 0.108° |
| **亚像素角点 + 尺度标定** | **0.77%** | **0.041°** |
| 亚像素 + 标定，但用旧的 marker 参考点 | 3.27% | 1.29° |

> **结论**：
> 1. 历史上报的「A 场景 4.44% / 1.27°」里约 **2.5 pp / 1.25°** 来自真值参考点偏移（marker 原点 vs 板中心），
>    不是解算误差；改用正确的板面真值 + 亚像素角点后是 **0.77% / 0.041°**，达到 I3 目标。
> 2. 亚像素是主要贡献（整像素 6.6% → 亚像素 0.77%）；整像素下尺度标定几乎无效——
>    因为灯条高度只有 ±1 px 的量化误差被 `DET_SCALE_Y=2.25` 放大，必须先有亚像素高度才谈得上标定。
> 3. D/E 的失败点在 **S1 检测**（遮挡/斜视传统法漏检，与 S1 实测 E 检出率 18.3% 一致），
>    不是 PnP；S2 在检测成功的帧上没有系统性误差。
> 4. **已知问题：板面姿态角不可靠**（A 场景 25°）。平面矩形在 PnP 里存在镜像/倾斜分支歧义，
>    我们的矩形化重建抹掉了透视线索，导致法向解不唯一；位置与视线角不受影响。
>    解决方向：YOLO 关键点给出真实四角、或用多帧 S3 滤波约束姿态。


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
| I1 | 识别准确率 | ≥ 0.90（传统）/ mAP@0.5 ≥ 0.85（DL） | 传统：图级检出 A/B/C=100%、D=83.4%、E=18.3%；**YOLO（跨场景验证 E）：mAP@0.5=0.845、mAP@0.5:0.95=0.778、实例召回 E=81.1% / A~D=88.7~100%**（见 `metrics/s1_traditional.json`、`metrics/s1_yolo.json`） | 传统：A~E bag 每 5 帧取 1；YOLO：跨场景验证（训练 A/B/C/D，验证 E）+ 逐场景实例级 IoU≥0.5 |
| I2 | 识别耗时 | ≤ 8 ms（CPU 1440×1080） | 传统：**7.44 ms**（CPU，p50 7.0/p95 10.7）；YOLO：**22.2 ms（CPU）/ 6.9 ms（GPU）** | 节点内计时 / `model.predict` 端到端计时 |
| I3 | 解算精度 | 距离误差 ≤ 5%，角误差 ≤ 1° | 端到端 A **0.77% / 0.041°**、B 1.58% / 0.050°、C 1.90% / 0.127°（D/E 受 S1 检测限制，见 3.2 / 6.2） | 与 `/tf` 板面真值对拍（`scripts/eval_s2_pnp.py`，`metrics/s2_pnp.json`） |
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

| 现象 | 原因 | 解决 |
|---|---|---|
| **大角度斜视（E）传统法检出率仅 18.3%** | 斜视时装甲板投影变窄，两个灯条间距/高度比超出几何约束（`gap_ratio` 0.6~3.0、`max_dy` 0.55），配对直接被过滤 | **YOLO 路线跨场景验证实例召回 81.1%**（训练不含 E）；S1 选型因此定为「传统法兜底 + 斜视/遮挡切 YOLO」 |
| 单灯条被遮挡就漏检 | 传统法必须有成对灯条才能出装甲板 | M2：单灯补全策略 / DL 直接回归角点 |
| **D 遮挡 / E 斜视下 S2 无可评估结果** | 传统检测器在这两个场景基本检不到目标板（D：每帧 1 个误检、0 关联；E：41 个检测、0 关联），PnP 本身没问题 | 归到 S1 修（YOLO 标签重做 + 关键点回归）；S2 已具备条件，检测一通就能出数 |
| **YOLO 自动标注曾经标错（已修）** | 旧版 `make_yolo_dataset.py` 把 `/tf` marker 中心当板中心、且用错板面平面 → 标注框画在板底部地面上，导致旧 mAP 0.974/检出 100% 是"自己标自己测"的循环论证 | 2026-11-05 改走 `plate_pose_from_marker()` + 灯条可见性过滤，重训后跨场景 mAP@0.5=0.845。**教训：自动标注必须先叠图目视验证** |
| **板面姿态角误差大（A 场景 25°）** | 平面矩形的 PnP 存在镜像/倾斜分支歧义；我们用灯条矩形重建，丢掉了真实透视边角线索 | 用 YOLO 关键点给真实四角；或用 S3 多帧滤波约束姿态。位置与视线角不受影响 |

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
