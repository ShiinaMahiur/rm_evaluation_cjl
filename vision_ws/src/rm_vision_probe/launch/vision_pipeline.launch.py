"""一键启动自瞄四段链路：S1 检测 → S2 PnP → S3 EKF → S4 规划

用法：
  ros2 launch rm_vision_probe vision_pipeline.launch.py
  ros2 launch rm_vision_probe vision_pipeline.launch.py fire_mode:=single publish_hz:=30.0
  ros2 launch rm_vision_probe vision_pipeline.launch.py debug_image:=true   # 发布检测可视化图

注意：仿真器需另行启动（~/.run_daedalus.sh 或 scripts/demo_live.sh 会自动处理）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_hz = LaunchConfiguration("publish_hz")
    debug_image = LaunchConfiguration("debug_image")
    fire_mode = LaunchConfiguration("fire_mode")
    method = LaunchConfiguration("method")
    return LaunchDescription([
        DeclareLaunchArgument("publish_hz", default_value="30.0",
                              description="S1 检测结果发布频率 (Hz)"),
        DeclareLaunchArgument("debug_image", default_value="false",
                              description="是否发布 /detector/debug_image（可视化）"),
        DeclareLaunchArgument("fire_mode", default_value="auto",
                              description="S4 开火模式：auto|single|off"),
        DeclareLaunchArgument("method", default_value="exact",
                              description="弹道补偿方法：exact|approx"),
        Node(package="rm_vision_probe", executable="detector_traditional",
             name="detector_traditional", output="screen",
             parameters=[{"publish_hz": publish_hz, "debug_image": debug_image}]),
        Node(package="rm_vision_probe", executable="armor_solver",
             name="armor_solver", output="screen"),
        Node(package="rm_vision_probe", executable="armor_tracker",
             name="armor_tracker", output="screen"),
        Node(package="rm_vision_probe", executable="trajectory_planner",
             name="trajectory_planner", output="screen",
             parameters=[{"fire_mode": fire_mode, "method": method}]),
    ])
