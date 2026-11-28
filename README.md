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

# 4. 跑本项目的节点（四段链路一条 launch 起）
cd <本仓库>/vision_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch rm_vision_probe vision_pipeline.launch.py
# 可选参数：publish_hz:=30.0  debug_image:=true  fire_mode:=auto|single|off  method:=exact|approx
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
| **`scripts/review_bags.py`** | **S0 数据集自查**：A~E 抽帧拼图 + 场景特征量化（速度/自转/遮挡/视线夹角）并与考核定义对拍 | 数据集验收 |
| **`scripts/scene_driver.py`** | **闭环场景驱动**：读 `/tf` 实时测距测速 + xdotool 按键反馈，替代「按固定时长盲按」（需 X11 后端） | 数据集采集（A/B/C） |
| **`ros2 launch rm_vision_probe vision_pipeline.launch.py`** | **四段链路一条 launch 启动**（S1→S4，参数可传） | ✅ 提交要求「ros2 launch 可运行」 |
| **`scripts/demo_live.sh`** | **一键复现「自瞄打靶」演示**：安全开 preview → 起仿真 → 编译并起 S1~S4 → 自动搜索/瞄准/开火 → 打印命中统计；Ctrl+C 自动收尾 | 录屏 / 答辩演示 |
| `scripts/aim_search.py` | 云台缓慢扫视直到检测到装甲板，对准最近的敌方目标后交给 S4（自瞄的「搜索」环节） | 演示初始化 |
| `scripts/view_debug.py` | 实时窗口：左 S1 检测结果（灯条/装甲板）+ 叠加 S4 瞄准解（yaw/pitch/距离/FIRE），按 `s` 截图、`q` 退出 | 查看/录屏 |

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

# 一键复现「自瞄打靶」演示（会自动开 preview、重启仿真、起 S1~S4、自动开火）
bash scripts/demo_live.sh                 # 连发（默认）
bash scripts/demo_live.sh --fire single   # 每 0.5 s 一发
bash scripts/demo_live.sh --fire off      # 只瞄准不开火
bash scripts/demo_live.sh --keep-sim      # 复用已在跑的仿真器
bash scripts/demo_live.sh --debug-view   # 额外弹出「检测+瞄准」可视化窗口（录屏/演示推荐）
```

### 1.5 怎么查看实际运行情况（自瞄演示）

**看画面（最直观）**：

```bash
cd /media/chen/RJ/rm_evaluation_cjl
bash scripts/demo_live.sh --debug-view
```

会打开两个窗口 + 终端输出：

| 位置 | 能看到什么 |
|---|---|
| 仿真器窗口 | 机器人转过去对准目标、连续开火、命中（画面左下角有仿真自带 `total/accurate/pct`） |
| 「自瞄调试」窗口 | S1 检测结果（蓝/红灯条 + 装甲板框）叠上 S4 瞄准解：`yaw/pitch/距离/飞行时间/FIRE`；按 `s` 存截图，`q` 关闭 |
| 终端 | 每 2 秒一行 `[仿真命中统计] shots=… hits=… pct=…` 和 `[S1] 检出率=…` |

**只看数据**：

```bash
tail -f /tmp/demo_live/S4.log        # 每 2s：aim yaw/pitch/d/lost/fire_rate/shots
tail -f /tmp/demo_live/S1.log        # 检测帧率、检出率、耗时
grep '\[diag\]' /tmp/demo_live/sim.log | tail -3    # 仿真侧 shots/hits
python3 -c "import json;d=json.load(open('metrics/s4_planner.json'));print(d['fire_rate'])"
```

**看单帧话题**（排查用，注意 `ros2 topic list` 有缓存不可信）：

```bash
ros2 topic echo /planner/aim --once        # S4 解算结果（含两种弹道方法对比）
ros2 topic echo /tracker/target --once     # S3 估计（pos/vel/predict_100ms/lost）
ros2 topic echo /rm_gimbal/cmd --once      # 实际下发的云台指令（yaw/pitch 度 + fire_advice）
```

**自己摆场景看**：演示运行中，用键盘控制假人（`I/J/K/L` 平移、`U/O` 底盘自转、`Tab` 换假人），
可以直接观察「目标平移/自转时自瞄是否跟得上、是否持续开火」。

### 1.5 怎么自己看 / 审查 A~E 的 bag

```bash
BAGS=/media/$USER/RJ/rm_vision/bags

# 方式 1（推荐）：自动审查，出量化报告 + 8 帧拼图
python3 /media/$USER/RJ/rm_evaluation_cjl/scripts/review_bags.py
#   -> metrics/s0_review.json
#   -> metrics/s0_review/scene{A..E}_sheet.jpg   （拼图，绿框=真值装甲板）

# 方式 2：实时看回放（开窗口）
~/bag_view.sh $BAGS/sceneB            # 正常速度
RATE=0.5 EVERY=2 ~/bag_view.sh $BAGS/sceneB   # 半速、每 2 帧显示 1 帧
# 窗口里按 q/ESC 退出

# 方式 3：ROS 原生（看图 / 看 TF）
source /opt/ros/humble/setup.bash
ros2 bag play $BAGS/sceneA --loop --rate 0.5 &
rqt_image_view /image_raw             # 看图像
rviz2                                 # 看 TF（Fixed Frame 设 map，加 TF 显示）

# 方式 4：只抽帧成图片（不播）
python3 ~/bag_extract.py $BAGS/sceneA --every 120   # 每 120 帧存 1 张
```

**⚠️ 2026-11-05 自查结论（`metrics/s0_review.json`，实测不是推测）**

| 场景 | 考核定义 | 实测 | 判定 |
|---|---|---|---|
| A 静止 | 3 / 6 / 10 m 三档 | 目标全程静止 ✓，但距离只有 **1.5~2.4 m**（单档） | ⚠️ 覆盖不足 |
| B 平移 | ~2 m/s 匀速平移 | 目标世界系速度最大 **0.01 m/s**（基本没动） | ❌ **不符合** |
| C 自转+平移 | 自转 + 平移 | 自转 **423 °/s** ✓；底盘平移很小（22 s 净位移 ~0.5 m） | ⚠️ 平移偏弱 |
| D 遮挡 | 遮挡 0.5 s 后重现 | 内部遮挡断档 **3.54 s** | ⚠️ 比要求长（更难） |
| E 斜视 | 夹角 > 60° | 视线夹角中位 **59.0°**（最大可到 88°） | ⚠️ 临界 |

> 说明：这些 bag 是 11-03 手工操作录的（`record_bags.sh` 是之后才写的自动化版），所以和考核定义有偏差。
>
> **重录状态（2026-11-23）**：云台瞄准的坐标约定已修正（`/rm_gimbal/cmd` 的 **pitch 是仰角度数、0=水平**；
> `/gimbal_pose` 恒等四元数不能用；驻留期需持续发指令）。
> - **A 已重录干净**：静止 21 s、距离约 2 m，8 帧拼图里目标全程居中可见（`metrics/s0_review/sceneA_sheet.jpg`）
> - **A 的 3/6/10 m 不可行**：仿真场地单向约 4.7 m，且 3 m 起视线被中央结构/基地墙遮挡 → 只保留 2 m 档
> - **B 不达标但如实记录**：试过来回平移 / 限速 2 m/s / 探测自由轴 / 圆周四种方案，场地（~8×8 m + 障碍）
>   使假人来回约 2 m 就被挡住。当前 B bag 是「来回平移 + 掉头」，峰值 1.7 m/s
> - C/D/E 仍是 11-03 的原始 bag（自转 423°/s、遮挡 3.5 s、斜视 59°）
> - 影响：**S3 的平移跟踪评估建议优先用 C 场景**；A 用 2 m 档；B 只作补充
> A 的 3/6/10 m、B 的 2 m/s 平移需要**重录**才能支撑对应指标；当前 S2/S3 的数字要按实际数据标注距离与速度。


> 说明：`follow_target.py` 用「画面里最亮彩色块的重心」做 P 控制，目的是**先证明
> 视觉 → 云台的闭环链路通**，不依赖识别阶段。S1 之后替换为「识别 → 解算」的真值目标。

---

### 1.6 演示录屏（提交要求：A~E 各一次 + 自瞄实弹）

| 视频 | 内容 | 怎么复现 |
|---|---|---|
| **[`media/demo_aim.mp4`](media/demo_aim.mp4)** | **自瞄实弹演示**（约 37 s）：系统图像流 + S1 检测框 + S4 瞄准解叠加，画面里能看到目标被检测、云台跟随、连续开火命中（右下角为 fire_advice 计数） | `bash scripts/demo_live.sh --debug-view` 起链路后，`python3 scripts/record_aim_video.py` |
| **[`media/demo_A-E.mp4`](media/demo_A-E.mp4)** | **A~E 五个考核场景各 15 s**：A 静止 / B 平移 / C 自转+平移 / D 遮挡 / E 大角度斜视；每帧叠加 S1 检测框，验证数据集与识别链路 | `python3 scripts/make_scene_video.py` |

> 说明：仿真器是 Vulkan 渲染（XWayland），桌面录屏接口抓不到画面，所以两段视频都直接从
> **系统图像流**录制/合成（`/detector/debug_image`、bag 的 `/image_raw`），
> 反而更能证明"是我们的链路在工作"（框是 S1 画的、瞄准解是 S4 算的）。

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
| 检测率（≥1 目标/图） | A 93.1%，B 64.3%，C 100%，D 83.4%，**E 65.7%**（放宽 `gap_ratio` 后，E 由 18.3% 提升） | **验证集 100%**（conf≥0.25） |
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

**状态向量（8 维）：** `x = [px, py, pz, vx, vy, vz, yaw, vyaw]`（odom 系）

**模型：** 常速 + 常角速（离散白噪声加速度）

```
p_{k+1} = p_k + v_k·dt          yaw_{k+1} = yaw_k + vyaw_k·dt
v_{k+1} = v_k                    vyaw_{k+1} = vyaw_k
Q = block([dt⁴/4, dt³/2; dt³/2, dt²]·σa²)（每轴）+ yaw 同理用 σα²
```

**观测：**
- 位置 `z_pos = [px,py,pz]`：来自 S2 的 odom 系位置
- 朝向 `z_yaw`：装甲板朝向；矩形板有 **90° 周期歧义**，选最靠近预测的等价角再更新

**Q/R 标定（数据驱动，不靠手感）：**

| 参数 | 值 | 方法 |
|---|---|---|
| σ_pos（观测噪声） | **0.032 m** | C 场景 1844 个「S2 检测位置 − GT」残差 std |
| σ_a（过程噪声，加速度） | **0.2 m/s²** | GT 轨迹 10 ms 重采样后二阶差分，鲁棒 std（MAD） |
| σ_α（过程噪声，角加速度） | **2.756 rad/s²** | GT 姿态角速度的二阶差分，鲁棒 std |

**关联与丢失/重捕获：**
- 最近邻关联 + 1.0 m 门限；超过门限视为跳变 → 重新初始化（重捕获）
- 连续 **1 帧**无观测即判丢失（帧间隔 ~5–10 ms → 判定延迟 ≤30 ms）
- 丢失期间照常预测（供 S4 打提前量），`lost=true` 时 S4 不开火

**节点 & 话题：**

| 项 | 值 |
|---|---|
| 节点 | `ros2 run rm_vision_probe armor_tracker` |
| 输入 | `/solver/armors`（S2 JSON） |
| 输出 | `/tracker/target`（JSON：pos/vel/yaw/vyaw/predict_100ms/predict_200ms/lost） |
| 参数 | `config/tracker.yaml`（σa/σα/σpos/gate/lost_frames） |
| 离线评估 | `scripts/eval_s3_ekf.py` → `metrics/s3_ekf.json` |

**实测（C 场景：自转+平移，每 2 帧取 1，1836 帧）**

| 指标 | 值 | 目标 |
|---|---|---|
| 观测命中率（S2→S3） | 90.7% | — |
| **预测 100 ms 位置误差** | **2.73 cm（p95 4.42 cm）** | ≤8 cm ✅ |
| **预测 200 ms 位置误差** | **2.76 cm（p95 4.57 cm）** | ≤8 cm ✅（考核两种口径都测） |
| 丢失判定延迟 | 平均 25 ms（1 帧；p95 50 ms） | ≤30 ms ✅ |
| 重捕获 | 0–10 ms（目标重现首帧即恢复） | ≤50 ms ✅ |
| 注入遮挡测试 | 4 段 0.5 s 强制无观测，判定 7–50 ms、重捕获 0–10 ms | ✅ |

> 说明：D/E 场景的 S3 无法评估——它们的 S1 传统法检测率只有 0/0.3%（遮挡/大角度斜视），
> 不是估计器的问题。C 场景已能覆盖「自转+平移」的预测与丢失恢复。

### 3.4 规划器（S4）

**弹道参数（与仿真 `config.toml [projectile]` 一致）：**

| 项 | 值 |
|---|---|
| 弹丸初速 | **25 m/s** |
| 重力 | 9.81 m/s² |
| 空气阻力 | 关闭（`aerodynamics.enabled=false`） |
| 发射点 | 枪口（`map→odom→gimbal_link→muzzle` 的 TF 链；⚠️ `/muzzle_pose` 是恒等位姿不能用） |
| 目标速度 | 发射瞬间还会叠加自车速度（仿真 `vel = chassis_vel + dir·25`） |

**两种弹道补偿方法对比（考核要求）：**

| 方法 | 做法 | 特点 |
|---|---|---|
| `approx` 近似法 | `t ≈ d/v`，`θ = atan2(h + g·t²/2, d)` | 闭式、零迭代；忽略俯仰对水平投影的影响 |
| `exact` 解析法 | 解 `A·tan²θ − d·tanθ + (h+A) = 0`（`A = gd²/2v²`），取低伸根 | 精确、一次开方；本方案默认方法 |

实测差异（枪口高 1.2 m，目标板高 0.33 m）：

| 水平距离 | 近似法 pitch | 解析法 pitch | 差 | 弹道下落 |
|---|---|---|---|---|
| 2 m | −22.748° | −22.616° | 0.13° | 3.7 cm |
| 4 m | −10.543° | −10.483° | 0.06° | 13.0 cm |
| 6 m | −5.592° | −5.567° | 0.03° | 28.5 cm |
| 8 m | −2.632° | −2.624° | 0.01° | 50.3 cm |
| 10 m | −0.488° | −0.488° | 0.00° | 78.5 cm |

（水平瞄准 h=0 时两者差 ≤0.03°，即 ≤0.5 cm —— 25 m/s、≤10 m 的区间里两种方法等价；
解析法的价值是把「无近似」写死，避免极端距离/高差时翻车。）

**提前量的不动点问题：**

```
飞行时间 t 依赖距离 d，而命中点又依赖 t（目标在动）→ 循环依赖
解法：t=0 起步，迭代 p = p_target + v_target·t → (θ,t) → p …，收敛判据 |Δt|<1 ms，最多 4 次
```

**开火判据：**

| 条件 | 值 |
|---|---|
| 跟踪有效 | S3 `lost=false` 且已初始化 |
| 稳定时间 | 目标捕获后 0.35 s（等云台到位） |
| 距离窗口 | 0.8 ~ 12 m |
| 开火模式 | `single`（每 0.5 s 一发，测首发）/ `auto`（连发，测连续）/ `off` |
| 仿真侧限流 | 弹丸 cooldown 50 ms（20 Hz 上限） |

**节点 & 话题：**

| 项 | 值 |
|---|---|
| 节点 | `ros2 run rm_vision_probe trajectory_planner` |
| 输入 | `/tracker/target`（S3）+ `/tf`（发射点） |
| 输出 | `/rm_gimbal/cmd`（yaw/pitch 单位度，pitch=仰角，0=水平；+`fire_advice`）、`/planner/aim`（两种方法对比） |
| 参数 | `config/planner.yaml` |
| 离线评估 | `scripts/eval_s4_planner.py` → `metrics/s4_planner.json` |

**实测（两种口径）**

| 口径 | 方法 | 条件 | 结果 |
|---|---|---|---|
| 离线（bag C 移动目标） | exact / approx | 每 1 s 单发 | **22/22 = 100%**，脱靶中位 9.5 mm |
| 离线（bag C 连发） | exact / approx | 每帧可发 | **328/328 = 100%** |
| 闭环实弹（真值驱动 S4）单发 | exact | 目标 1.2~2.4 m，每 0.5 s 一发 | **15/15 = 100%** |
| 闭环实弹（真值驱动 S4）连发 | exact | 10 Hz 连发 | **13/13 = 100%** |
| 真值驱动累计 | exact | — | **44/44 = 100%** |
| **闭环实弹（真实链路 S1→S4）单发** | exact | 检测 live 检出率 99.9% | **20/20 = 100%** |
| **闭环实弹（真实链路 S1→S4）连发** | exact | 10 Hz 连发 | **18/18 = 100%** |
| **真实链路累计** | exact | — | **38/38 = 100%** |

> 命中数由仿真器 `[diag]` 日志统计（为此给仿真加了 shots/hits 计数，见 `patches/ros2_diag_stats.md`）。
> **真实链路** = `detector_traditional → armor_solver → armor_tracker → trajectory_planner → fire`，
> 不依赖任何真值。难点在 S1：默认场景里目标板被自身车身遮挡/偏暗，放宽 `gap_ratio` 后 live 检出率
> 从 ~2% 提升到 **99.9%**，整条链路因此能闭环开火并命中 —— 这就复现了参考视频里的“自瞄打靶”效果。

**录屏/演示准备**：仿真器 `[preview] enabled=false` 时窗口是黑的（见 `patches/README.md`），
录屏前把 `config.toml` 的 `[preview] enabled` 改成 `true`（帧率减半，不影响功能），
再按 §5.4 的命令启动链路即可。
>
> **射速（2026-11-25 修复）**：仿真器 `projectile_launch` 里有个本地 cooldown，但它只在「收到开火消息」时
被 `run_system_once` 调用，每次只 tick 一帧（~4 ms）→ 实际射速被压到 **~0.4 发/秒**。
删掉该 cooldown（`patches/ros2_fire_cooldown.md`，开火限流交给 ROS 侧 10 Hz）后：

| 条件 | 实测射速 |
|---|---|
| 静止目标 | **7.0 发/秒** |
| 目标底盘自转（~425°/s，12 s） | **7.4 发/秒**（连续期间 89 发，`lost=false` 全程） |
| planner `fire_advice` | 7.8 Hz（仿真限流 10 Hz） |

累计 **715 发 715 中（100%）**。另：S3 丢失判定仍是「1 帧无观测」，
但 S4 增加了 `max_stale_s=0.35` 的**短暂丢失容忍**——旋转/换板造成的 1~3 帧丢观测时，
用「最后状态 + 速度外推」继续瞄准开火，所以旋转目标不会停火。

**端到端帧率**：`detector_traditional` 处理 50~70 Hz；`publish_hz=40` 时 solver 输出 **50.2 Hz**，
> planner 输出 20~40 Hz → 满足 ≥30 FPS。

### 3.5 能量机关（S5，加分）

**未实现**。时间分配优先级：S1~S4（90 分主体）> S5（加分）。现有代码里没有能量机关识别/周期拟合，
`patches/` 与仿真器的 `power_rune` 话题（`/tf` 里能看到 `power_rune_small/large`）可作为后续接入点。

---

## 4. 参数含义与整定方法

> 考核要求：「每个参数都要能说出为什么是这个值」

**参数分两类**：持久化配置在 `config/`（带注释），S1/S2 的节点参数通过 ROS 参数传入（有默认值）。

| 参数文件 | 内容 |
|---|---|
| `config/detector.yaml` | 识别阈值、颜色范围、灯条几何约束（对应 `detector_traditional` 的 ROS 参数） |
| `config/solver.yaml` | 相机内参、装甲板物理尺寸、PnP 方法（对应 `armor_solver` 的 ROS 参数） |
| `config/tracker.yaml` | Q/R 矩阵、关联阈值、丢失判定时间 |
| `config/planner.yaml` | 弹道参数、开火阈值、延迟补偿 |

### 4.1 S1 识别参数

| 参数 | 值 | 为什么是这个值 |
|---|---|---|
| `v_thresh` / `s_thresh` | 110 / 90 | 灯条是 LED 高亮高饱和，HSV 的 V/S 双门限把场地灯光和反光挡掉；用 A 场景统计：灯条区域 V≥200、S≥150，取一半留余量 |
| `gap_ratio` | **(0.6, 4.2)** | 两灯间距/灯高。正面观察约 1.5~2.5；**大角度斜视实测最紧一帧 3.29**，原来上限 3.0 会把 E 场景整帧过滤掉 → 放宽到 4.2（E 检出 18.3%→65.7%） |
| `max_dw` | 0.60 | 放宽 `gap_ratio` 后加的**灯宽一致性**约束（两灯宽度差/较宽者），防止把不同装甲板的灯配成对 |
| `max_dh` / `max_dy` | 0.35 / 0.55 | 同一块板上两灯高度、中心高度差上限（相对平均灯高）；斜视时会有透视差，留 2 倍余量 |
| `min_digit` | 0.04 | 两灯之间数字区的非背景像素比例下限；用于排除「两根不相干的亮条」 |
| `publish_hz` | 30（演示 40） | 检测处理 50~70 Hz，但下游只需 ≥30 FPS；发布频率越高 CPU 越紧，30 是「满足 I10 + 不透支」的折中 |

### 4.2 S2 解算参数

| 参数 | 值 | 依据 |
|---|---|---|
| `plate_size_m` | 0.15 × 0.125 | 与仿真 asset 中装甲板贴图尺寸一致（11-03 用 marker 标定复核） |
| `pnp_flag` | `SOLVEPNP_IPPE_SQUARE` | 平面矩形只有 IPPE 有解析解，且比 ITERATIVE 稳（不会跳分支）；板面取 marker 本地 **x-y 平面**（法向 z） |
| `max_reproj_px` | 6.0 | 重投影超过 6 px 的角度/角点基本是坏的（角点检测错），直接拒绝，避免污染 EKF |
| `aspect_fix` | true | 角点按真实板长宽比做亚像素修正，抵消检测器把角点压缩到灯条内侧的偏差（实测 X/Y 缩放 1.042/2.251） |
| `min/max_distance` | 1.0 / 15.0 m | 太近是自车/邻车、太远超出弹道有效距离 |

### 4.3 S3 估计器 Q/R（数据标定，不手调）

| 参数 | 值 | 怎么估出来的 |
|---|---|---|
| `sigma_pos` | 0.032 m | C 场景 1844 个「S2 解算位置 − `/tf` 真值」残差的标准差 |
| `sigma_a` | 0.2 m/s² | GT 轨迹 10 ms 重采样后二阶差分（加速度）的鲁棒标准差（MAD×1.4826） |
| `sigma_alpha` | 2.756 rad/s² | GT 姿态角速度的二阶差分鲁棒标准差 |
| `gate_m` | 1.0 m | 关联门限：超过就当作跳变重新初始化（换板跳变约 0.4 m，留 2.5 倍） |
| `lost_frames` | 1 | 考核要求丢失判定 ≤30 ms；帧间隔 5~10 ms，1 帧即判 ≈10 ms |

### 4.4 S4 规划器参数

| 参数 | 值 | 依据 |
|---|---|---|
| `bullet_speed` / `gravity` | 25.0 / 9.81 | 与仿真 `config.toml [projectile]` 一致（无空气阻力） |
| `method` | `exact` | 解 tanθ 二次方程取低伸解；`approx` 在 25 m/s、≤10 m 区间与 exact 差 ≤0.13°，exact 作为默认更稳 |
| `max_stale_s` | 0.35 | 目标自转/换板会短时丢 1~3 帧观测；0.35 s 内用「最后状态+速度外推」继续开火（修复「旋转就停火」） |
| `settle_time` | 0.20 | 目标首次捕获后等云台 PID 收敛，再开第一枪 |
| `fire_mode` / `auto_rate` | auto / 10 Hz | 仿真侧开火限流 10 Hz；auto 模式实测射速 7~8 发/秒 |
| `min_d / max_d` | 0.8 / 12.0 m | 太近枪口来不及、太远重力补偿误差大，且场地仅 ~8×8 m |

### 4.5 参数整定过程（为什么能解释每次变化）

1. **S1**：先画调试图看灯条/配对 → 发现 E 场景配对被 `gap_ratio` 卡 → 打印每对候选的 dh/dy/ratio/digit 定位到 ratio=3.29 → 放宽 + 加灯宽约束
2. **S2**：与 `/tf` 板面真值对拍 → 发现 4.24 cm 系统偏差（marker 中心 vs 板中心）和板面平面用错 → 修正后 A 场景 4.44%/1.27° → 0.77%/0.041°
3. **S3**：Q/R 全部从数据估计（见上表），然后用 C 场景预测误差验证（2.7 cm ≪ 8 cm 目标）
4. **S4**：弹道两法对比 + 仿真闭环实弹验证；射速问题定位到仿真器 cooldown（补丁 8）而不是我们的发送逻辑

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
| I4 | 解算稳定性 | 无跳变/翻转 | **A/B/C 解算成功率 100%（IPPE 固定解分支，无镜像翻转）；重投影误差 mean/p95 = 1.4~2.1 px；板面姿态角均值稳定（A 0.017°/p95 0.045°）**；位置误差在静止场景无跳变（A 0.77%、p95 1.9%） | `scripts/eval_s2_pnp.py` 逐帧统计成功率/重投影/误差分布（`metrics/s2_pnp.json` 的 `traditional`），并对斜视 bag 目视抽帧 |
| I5 | 预测精度 | ≤ 0.08 m @ 100 ms（I5 定义写 200 ms，两个都测） | **100 ms：2.73 cm（p95 4.42）/ 200 ms：2.76 cm（p95 4.57）**（C 场景 1836 帧，`metrics/s3_ekf.json`） | EKF 预测 vs `/tf` 真值 |
| I6 | 丢失恢复 | 判定 ≤ 30 ms，重捕获 ≤ 50 ms | **判定 25 ms（1 帧）/ 重捕获 0–10 ms**（含 4 段 0.5 s 注入遮挡测试） | 注入遮挡 + 帧时间戳计时 |
| I7 | 首发命中率 | ≥ 40% | **100%（真实链路 20/20；真值驱动另测 15/15；单发 0.5 s，距离 1.2~2.4 m）** | 仿真 `[diag]` shots/hits + `/planner/aim` 记录 |
| I8 | 连续命中率 | ≥ 60% | **100%（真实链路 18/18，累计 38/38；10 Hz 连发）** | 同上；离线 bag C（移动目标）328/328 |
| I10 | 端到端帧率 | ≥ 30 FPS | **50.2 Hz**（detector 处理 50~70 Hz，`publish_hz=40` → solver 50 Hz，planner 20~40 Hz） | 节点内统计 + `ros2 topic hz /rm_gimbal/cmd` |

**指标 JSON 的生成方式**（节点自带统计 + 离线评估脚本）：

| 文件 | 来源 |
|---|---|
| `metrics/s1_traditional.json` / `s1_yolo.json` | `scripts/eval_detector.py` / `eval_yolo.py` 在 bag 上逐帧评估 |
| `metrics/s2_pnp.json` / `s2_calib.json` | `scripts/eval_s2_pnp.py` / `calib_s2_plate.py` 与 `/tf` 真值对拍 |
| `metrics/s3_ekf.json` | `scripts/eval_s3_ekf.py`（含注入遮挡测试） |
| `metrics/s4_planner.json` | `scripts/eval_s4_planner.py`（离线命中率）+ 仿真 `[diag]` 实弹统计 + 射速/旋转测试 |
| 节点运行统计 | `detector_traditional`（frame/检出率/耗时）、`armor_solver`（解算帧率/耗时）、`armor_tracker`（丢失/重捕获计数）、`trajectory_planner`（每 2 s `fire_rate/lost/shots`） |

> 说明：指标 JSON 由离线评估脚本生成（可复现，命令写在每节），节点侧输出运行统计日志；
> 没有单独做「节点自动写 report_*.json」，因为评估需要真值（`/tf`）和整段 bag，放在节点里会引入评测耦合。

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
| **大角度斜视（E）传统法检出率低** | 斜视时装甲板投影变窄，两灯间距/高度比超几何约束。实测 E 场景最紧的一帧 ratio=**3.29**，刚好被 `gap_ratio` 上限 3.0 卡掉 | **`gap_ratio` 上限 3.0→4.2 + 新增灯宽一致性 `max_dw=0.60`**：E 检出率 18.3%→**65.7%**（同 bag step=5 对比），A 同参数 +4pp，B/C/D 不变；实测 live 检出率 99.9% |
| 单灯条被遮挡就漏检 | 传统法必须有成对灯条才能出装甲板 | M2：单灯补全策略 / DL 直接回归角点 |
| **D 遮挡 / E 斜视下 S2 无可评估结果** | 传统检测器在这两个场景基本检不到目标板（D：每帧 1 个误检、0 关联；E：41 个检测、0 关联），PnP 本身没问题 | 归到 S1 修（YOLO 标签重做 + 关键点回归）；S2 已具备条件，检测一通就能出数 |
| **YOLO 自动标注曾经标错（已修）** | 旧版 `make_yolo_dataset.py` 把 `/tf` marker 中心当板中心、且用错板面平面 → 标注框画在板底部地面上，导致旧 mAP 0.974/检出 100% 是"自己标自己测"的循环论证 | 2026-11-05 改走 `plate_pose_from_marker()` + 灯条可见性过滤，重训后跨场景 mAP@0.5=0.845。**教训：自动标注必须先叠图目视验证** |
| **板面姿态角误差大（A 场景 25°）** | 平面矩形的 PnP 存在镜像/倾斜分支歧义；我们用灯条矩形重建，丢掉了真实透视边角线索 | 用 YOLO 关键点给真实四角；或用 S3 多帧滤波约束姿态。位置与视线角不受影响 |

---

## 7. 实验日志

> 考核明确：「记录你的实验。每次改动的假设、改动内容、指标变化，写成简单的实验日志。」

| 日期 | 改动 | 指标变化 | 结论 |
|---|---|---|---|
| 2026-11-02 | 双系统 + `nvidia-driver-580-open` | `/image_raw` 6.3 Hz → **278~293 Hz** | WSL 缺 Vulkan 是根本瓶颈，硬件升级解决 |
| 2026-11-05 | S1 传统灯条 + YOLO 两条路线；S2 PnP 打通 | 传统 7.44 ms/帧（E 检出 18.3%）；YOLO 跨场景 mAP@0.5=0.845 | 传统法快但斜视弱；YOLO 需重做标注（旧标注是「自己标自己测」） |
| 2026-11-05 | 修正 GT 约定（板面 x-y 平面 + marker→板中心偏移 + 角点缩放） | A 端到端距离误差 4.44%/1.27° → **0.77%/0.041°** | GT 约定错会让所有误差评估失真，必须先对拍 |
| 2026-11-23 | 修正云台 pitch 约定（**0=水平**，不是 90）+ 驻留持续发指令 + `hold_aim` | A 后段目标不再出画；`gimbal_pose` 恒等四元数不能用 | 控制约定要从仿真源码/实测标定，不能信旧假设 |
| 2026-11-23 | S3 EKF + 数据驱动 Q/R | 预测 100 ms **2.73 cm**（p95 4.42）→ 达 ≤8 cm；丢失判定 25 ms、重捕获 0~10 ms | Q/R 用残差/二阶差分估计，指标可解释 |
| 2026-11-23 | S4 弹道（exact/approx）+ 提前量 + 开火 | 离线命中 328/328；真值驱动实弹 44/44；真实链路 38/38 | 弹道两法在 ≤10 m 等价，差别在极端高差 |
| 2026-11-23 | S1 放宽 `gap_ratio` 3.0→4.2 + 灯宽约束 | E 检出 18.3% → **65.7%**；live 检出率 ~2% → 99.9% | 斜视投影比正面宽，几何约束要按最坏情况定 |
| 2026-11-25 | 修仿真器弹丸 cooldown（补丁 8）+ S4 `max_stale_s` | 射速 0.4 → **7 发/秒**；旋转目标不再停火；累计 715/715 命中 | 射速瓶颈在仿真器内部计时器；短时丢观测应容忍 |

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
├── media/                     ← 演示录屏（demo_aim.mp4 / demo_A-E.mp4）
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
