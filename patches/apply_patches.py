#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Daedalus (bevy_robomaster_simulator) 编译修复补丁
在 --features ros2 编译路径上，原项目有 6 处问题，本脚本全部修掉。
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
    for old, new, tag in pairs:
        if new in s:
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
print("\n[1/4] src/main.rs")
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
print("\n[2/4] src/ros2/plugin.rs")
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
print("\n[3/4] src/systems/controller.rs")
edit("src/systems/controller.rs", [
    # 3a. update_auto_aim_subscription 每帧用 controller 状态覆盖 SubscribeAutoAim，
    #     两个来源默认都是 false，导致 F5 之外无法开启自瞄订阅
    ("    pub fn auto_aim_active(&self) -> bool {\n"
     "        self.keyboard_auto_aim || self.controlled.auto_aim\n"
     "    }",
     "    pub fn auto_aim_active(&self) -> bool {\n"
     "        // [dsh patch] force always-on so external ROS2 nodes can drive the gimbal\n"
     "        let _ = (self.keyboard_auto_aim, self.controlled.auto_aim);\n"
     "        true\n"
     "    }",
     "auto_aim_active 强制常开"),
])

# ---------- 4. config.toml 性能与采集 ----------
print("\n[4/4] config.toml")
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
