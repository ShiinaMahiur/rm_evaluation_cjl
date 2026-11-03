#!/usr/bin/env python3
"""闭环瞄准（v2）：按 /tf 真值算目标方位，用方向键把云台转过去；带方向自校正。"""
import math, subprocess, sys, time
import rclpy
from rclpy.node import Node
import tf2_ros

def focus():
    subprocess.run(["bash","-c","W=$(xdotool search --name daedalus | head -1); xdotool windowactivate --sync $W 2>/dev/null"],check=False)

def press(key, dur=0.1):
    subprocess.run(["xdotool","keydown",key],check=False); time.sleep(dur)
    subprocess.run(["xdotool","keyup",key],check=False)

class Aim:
    def __init__(self, node, buf):
        self.node, self.buf = node, buf
        self.frames=[f"armor_{i}" for i in range(6,18)]
    def errors(self, prefer_far=True):
        try: Tc=self.buf.lookup_transform('map','camera_optical_frame',rclpy.time.Time())
        except Exception: return None
        q=Tc.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
        R=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
           [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
           [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        cp=Tc.transform.translation; best=None
        for f in self.frames:
            try: T=self.buf.lookup_transform('map',f,rclpy.time.Time())
            except Exception: continue
            p=T.transform.translation
            d=[p.x-cp.x,p.y-cp.y,p.z-cp.z]
            cx=R[0][0]*d[0]+R[1][0]*d[1]+R[2][0]*d[2]
            cy=R[0][1]*d[0]+R[1][1]*d[1]+R[2][1]*d[2]
            cz=R[0][2]*d[0]+R[1][2]*d[1]+R[2][2]*d[2]
            dist=math.sqrt(d[0]**2+d[1]**2+d[2]**2)
            if cz<=0.2 or not (1.0<dist<25.0): continue
            yaw=math.degrees(math.atan2(cx,cz)); pitch=math.degrees(math.atan2(-cy,math.hypot(cx,cz)))
            score=abs(yaw)+(0 if prefer_far and dist>=3 else 0)
            if best is None or score<best[0]: best=(score,f,dist,yaw,pitch)
        return best

def main():
    rclpy.init(); node=Node("aim_loop2"); buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,node)
    for _ in range(60):
        rclpy.spin_once(node,timeout_sec=0.1)
        if buf.can_transform('map','camera_optical_frame',rclpy.time.Time()): break
    aim=Aim(node,buf); focus()
    dir_yaw, dir_pitch = 1, 1     # 1=Right / 1=Down，猜的方向，错了会自动翻
    last=None
    for it in range(220):
        rclpy.spin_once(node,timeout_sec=0.02)
        e=aim.errors()
        if e is None:
            press("Right",0.12); continue
        _,f,dist,yaw,pitch=e
        if it%8==0: print(f"[{it:3d}] {f} dist={dist:.1f} yaw={yaw:+.1f} pitch={pitch:+.1f} dirY={dir_yaw} dirP={dir_pitch}")
        if abs(yaw)<2 and abs(pitch)<2:
            print(f"[OK] {f} dist={dist:.1f} yaw={yaw:+.1f} pitch={pitch:+.1f}"); rclpy.shutdown(); return 0
        # 选轴
        axis = "yaw" if abs(yaw)>=2 else "pitch"
        val = yaw if axis=="yaw" else pitch
        prev_abs = abs(val)
        if axis=="yaw":
            key = "Right" if dir_yaw>0 else "Left"; press(key,0.1)
        else:
            key = "Down" if dir_pitch>0 else "Up"; press(key,0.1)
        time.sleep(0.12); rclpy.spin_once(node,timeout_sec=0.02)
        e2=aim.errors()
        if e2 is None: continue
        _,_,_,yaw2,pitch2=e2
        new_abs = abs(yaw2 if axis=="yaw" else pitch2)
        if new_abs > prev_abs + 0.5:   # 方向反了 → 翻转
            if axis=="yaw": dir_yaw*=-1
            else: dir_pitch*=-1
    print("[X] 超时"); rclpy.shutdown(); return 1

if __name__=="__main__": sys.exit(main())
