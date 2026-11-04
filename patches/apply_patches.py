#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Daedalus (bevy_robomaster_simulator) 编译修复补丁
在 --features ros2 编译路径上，原项目有 6 处问题 + 我们加的 2 处功能补丁，本脚本全部处理。
可重复运行（幂等）。

用法:  python3 apply_patches.py /path/to/bevy_robomaster_simulator
"""
import os
import re
import sys

REPO = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/projects/bevy_robomaster_simulator")

changed = []


def edit(relpath, pairs):
    path = os.path.join(REPO, relpath)
    if not os.path.isfile(path):
        print(f"[跳过] 找不到 {relpath}")
        return
    with open(path, encoding="utf-8") as f:
        s = f.read()
    orig = s
    for item in pairs:
        old, new, tag = item[0], item[1], item[2]
        marker = item[3] if len(item) > 3 else new
        if marker in s:
            print(f"  [已改过] {tag}")
            continue
        n = s.count(old)
        if n == 0:
            print(f"  [警告] 没匹配到: {tag}")
            continue
        s = s.replace(old, new)
        print(f"  [OK] {tag}  (替换 {n} 处)")
    if s != orig:
        with open(path, "w", encoding="utf-8") as f:
            f.write(s)
        changed.append(relpath)


print("=" * 60)
print("Daedalus 补丁  ->", REPO)
print("=" * 60)

# ---------- 1. main.rs ----------
print("\n[1/5] src/main.rs")
edit("src/main.rs", [
    # 1a. cfg(feature="ros2") 块里用到 app，但参数名叫 _app -> 编译错误
    ("fn should_enable_talos_plugin(_app: &App) -> bool {",
     "fn should_enable_talos_plugin(app: &App) -> bool {",
     "should_enable_talos_plugin 参数 _app -> app"),

    # 1b. 自瞄订阅默认关闭 -> 外部 ROS2 节点驱动不了云台
    ("SubscribeAutoAim(AtomicBool::new(false))",
     "SubscribeAutoAim(AtomicBool::new(true))",
     "SubscribeAutoAim 默认 true"),
])

# ---------- 2. src/ros2/plugin.rs ----------
print("\n[2/5] src/ros2/plugin.rs")
edit("src/ros2/plugin.rs", [
    # 2a. TextureFormat::bevy_default() 在 bevy 0.19 里没有这个路径
    ("texture_format: TextureFormat::bevy_default(),",
     "texture_format: TextureFormat::Rgba8UnormSrgb,",
     "TextureFormat::bevy_default -> Rgba8UnormSrgb"),

    # 2b. 场景加载竞态：one().unwrap() 会 None -> panic
    ('let tf = center.get(qq.of(entity).suffix("CENTER").any().one().unwrap())'
     '.unwrap().1.compute_transform();',
     'let _ce = match qq.of(entity).suffix("CENTER").any().one() '
     '{ Some(e) => e, None => continue };\n'
     '                let _cv = match center.get(_ce) '
     '{ Ok(v) => v, Err(_) => continue };\n'
     '                let tf = _cv.1.compute_transform();',
     "capture_rune 第 1 处 unwrap -> 安全处理"),

    # 2c. 同上（多行写法）
    ("let mut tff = center\n"
     '            .get(qq.of(entity).suffix("CENTER").any().one().unwrap())\n'
     "            .unwrap()\n"
     "            .1\n"
     "            .compute_transform();",
     'let _ce2 = match qq.of(entity).suffix("CENTER").any().one() '
     '{ Some(e) => e, None => continue };\n'
     '        let _cv2 = match center.get(_ce2) '
     '{ Ok(v) => v, Err(_) => continue };\n'
     "        let mut tff = _cv2.1.compute_transform();",
     "capture_rune 第 2 处 unwrap -> 安全处理"),
])

# ---------- 3. src/systems/controller.rs ----------
print("\n[3/5] src/systems/controller.rs")
edit("src/systems/controller.rs", [
    # 3a. 让 ControllerState 的 keyboard_auto_aim 默认开启（默认进入自瞄模式），
    #     但 F5 仍然可以切到手动模式 —— 原版 Default 全是 false，F5 切不到自瞄
    ("#[derive(Resource, Debug, Default)]\n"
     "pub struct ControllerState {",
     "#[derive(Resource, Debug)]\n"
     "pub struct ControllerState {",
     "ControllerState 去掉 derive(Default)"),

    ("impl ControllerState {\n"
     "    pub fn reset_frame(&mut self) {",
     "impl Default for ControllerState {\n"
     "    fn default() -> Self {\n"
     "        // [dsh patch] 自瞄订阅默认开启：外部 ROS2 节点可直接驱动云台；\n"
     "        // 按 F5 可切到手动模式（方向键控云台），再按一次切回自瞄。\n"
     "        Self {\n"
     "            controlled: ControllerInput::default(),\n"
     "            remote: ControllerInput::default(),\n"
     "            keyboard_auto_aim: true,\n"
     "            controlled_chassis_spin: ChassisSpinMode::default(),\n"
     "            remote_chassis_spin: ChassisSpinMode::default(),\n"
     "            active_gamepad: None,\n"
     "            help: ControllerHelp::default(),\n"
     "        }\n"
     "    }\n"
     "}\n"
     "\n"
     "impl ControllerState {\n"
     "    pub fn reset_frame(&mut self) {",
     "ControllerState 手写 Default：自瞄默认开启",
     "impl Default for ControllerState"),

    # 3b. auto_aim_active：默认自瞄 + F5 可切 + 环境变量可强制
    #     （官方原版是 self.keyboard_auto_aim || self.controlled.auto_aim，
    #      但默认值 false 导致外部节点驱动不了云台）
    ("    pub fn auto_aim_active(&self) -> bool {\n"
     "        self.keyboard_auto_aim || self.controlled.auto_aim\n"
     "    }",
     "    pub fn auto_aim_active(&self) -> bool {\n"
     "        // [dsh patch] 自瞄订阅默认开启（见上面的 Default 实现），F5 可在\n"
     "        // 「自瞄(外部 ROS 驱动) / 手动(方向键)」之间切换。\n"
     "        // 另外支持环境变量强制指定：\n"
     "        //   DAEDALUS_MANUAL_AIM=1  -> 强制手动（脚本/录数据时方便）\n"
     "        //   DAEDALUS_MANUAL_AIM=0  -> 强制自瞄\n"
     "        static OVERRIDE: std::sync::OnceLock<Option<bool>> = std::sync::OnceLock::new();\n"
     "        if let Some(force_manual) = *OVERRIDE.get_or_init(|| {\n"
     "            std::env::var(\"DAEDALUS_MANUAL_AIM\")\n"
     "                .ok()\n"
     "                .map(|v| v == \"1\")\n"
     "        }) {\n"
     "            return !force_manual;\n"
     "        }\n"
     "        self.keyboard_auto_aim || self.controlled.auto_aim\n"
     "    }",
     "auto_aim_active：默认自瞄 + F5 切换 + 环境变量强制",
     "DAEDALUS_MANUAL_AIM"),
])

# ---------- 4. src/systems/gimbal_pid.rs（调试日志，可选） ----------
print("\n[4/5] src/systems/gimbal_pid.rs")
edit("src/systems/gimbal_pid.rs", [
    # 4a. 设 DAEDALUS_PID_DEBUG=1 时打印云台 PID 内部状态（目标/当前/误差/输出），
    #     用来排查“发了 /rm_gimbal/cmd 但云台不动”这类问题
    ("    let pid = &config.vehicle.gimbal_pid;\n"
     "    let yaw_rate = tracker.yaw.step(yaw_error, &pid.yaw, dt);\n"
     "    let pitch_rate = tracker.pitch.step(pitch_error, &pid.pitch, dt);\n",
     "    let pid = &config.vehicle.gimbal_pid;\n"
     "    let yaw_rate = tracker.yaw.step(yaw_error, &pid.yaw, dt);\n"
     "    let pitch_rate = tracker.pitch.step(pitch_error, &pid.pitch, dt);\n"
     "\n"
     "    // [dsh debug] 设 DAEDALUS_PID_DEBUG=1 打印云台 PID 内部状态\n"
     "    if std::env::var(\"DAEDALUS_PID_DEBUG\").is_ok() {\n"
     "        let (ty, tp, _) = tracker.target().rotation().to_euler(EulerRot::YXZ);\n"
     "        let (cy, cp, _) = gimbal_transform.rotation.to_euler(EulerRot::YXZ);\n"
     "        let (gw_y, gw_p, _) = gimbal_global.rotation().to_euler(EulerRot::YXZ);\n"
     "        let (m_y, m_p, _) = muzzle.rotation().to_euler(EulerRot::YXZ);\n"
     "        info!(\n"
     "            \"[pid] tgt(y={:.3},p={:.3}) cur_local(y={:.3},p={:.3}) gimbal_world(y={:.3},p={:.3}) muzzle(y={:.3},p={:.3}) err=({:.3},{:.3}) rate=({:.2},{:.2})\",\n"
     "            ty, tp, cy, cp, gw_y, gw_p, m_y, m_p, yaw_error, pitch_error, yaw_rate, pitch_rate\n"
     "        );\n"
     "    }\n",
     "gimbal_pid 调试日志（DAEDALUS_PID_DEBUG=1）",
     "DAEDALUS_PID_DEBUG"),
])

# ---------- 5. config.toml 性能与采集 ----------
print("\n[5/5] config.toml")
cfg = os.path.join(REPO, "config.toml")
if os.path.isfile(cfg):
    with open(cfg, encoding="utf-8") as f:
        text = f.read()
    original = text

    # 4a. 全局 [render]
    text = text.replace("shadows = true", "shadows = false")
    text = text.replace("main_camera_fxaa = true", "main_camera_fxaa = false")

    # 4b. 段内 enabled 开关
    def set_section_flag(text, section, key, value):
        out, sec = [], ""
        for line in text.split("\n"):
            m = re.match(r"^\[(.+?)\]", line.strip())
            if m:
                sec = m.group(1)
                out.append(line)
                continue
            if sec == section and line.strip().startswith(key + " "):
                out.append(f"{key} = {value}")
            elif sec == section and line.strip().startswith(key + "="):
                out.append(f"{key} = {value}")
            else:
                out.append(line)
        return "\n".join(out)

    text = set_section_flag(text, "preview", "enabled", "false")
    text = set_section_flag(text, "livox_ros", "enabled", "false")

    if text != original:
        with open(cfg, "w", encoding="utf-8") as f:
            f.write(text)
        changed.append("config.toml")

    print("  [OK] shadows=false, main_camera_fxaa=false")
    print("  [OK] preview=false (关掉预览窗口回渲染)")
    print("  [OK] livox_ros=false (关掉深度点云，帧率 +48%)")
else:
    print("  [跳过] 找不到 config.toml")

# ---------- 汇总 ----------
print("\n" + "=" * 60)
if changed:
    print("已修改文件:")
    for c in changed:
        print("  -", c)
else:
    print("没有文件被修改（可能已经打过补丁了）")
print("=" * 60)
print("\n下一步: cargo build --release --features ros2")
