#!/bin/bash
set +u
source /opt/ros/humble/setup.bash
source /home/chen/ros2_ws/install/setup.bash
source /home/chen/vision_ws/install/setup.bash

GLOG=/home/chen/gimbal_only.log
rm -f "$GLOG"
setsid nohup ros2 run rm_vision_probe gimbal_cmd --ros-args -p yaw:=0.0 -p pitch:=0.0 > "$GLOG" 2>&1 < /dev/null &
sleep 8
echo "--- A) yaw=0.0 ---"
timeout 10 ros2 topic echo /gimbal_pose --once --field pose.orientation 2>&1 | tail -5
ros2 param set /gimbal_cmd yaw 0.6 2>&1 | tail -1
sleep 10
echo "--- B) yaw=0.6 ---"
timeout 10 ros2 topic echo /gimbal_pose --once --field pose.orientation 2>&1 | tail -5
ros2 param set /gimbal_cmd yaw -0.6 2>&1 | tail -1
sleep 10
echo "--- C) yaw=-0.6 ---"
timeout 10 ros2 topic echo /gimbal_pose --once --field pose.orientation 2>&1 | tail -5
pkill -f "rm_vision_probe/gimbal_cmd" 2>/dev/null
echo GIMBAL_DONE
