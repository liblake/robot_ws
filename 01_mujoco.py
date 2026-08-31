import mujoco
import numpy as np
import time

# ---------- 1. 定义 MuJoCo 模型（XML 字符串） ----------
XML = """
<mujoco>
  <worldbody>
    <!-- 地面：半透明灰色平面，范围 5x5 -->
    <geom type="plane" size="5 5 0.1" rgba="0.5 0.5 0.5 0.5"/>

    <!-- 木块：初始位置 z=1 米（质心高度），边长 0.2 米，红色 -->
    <body name="block" pos="0 0 2">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 0.8"/>
    </body>
  </worldbody>
</mujoco>
""" 

# ---------- 2. 创建模型和数据 ----------
model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

dt = model.opt.timestep  # 默认 0.002 s

# ---------- 3. 模拟参数 ----------
total_duration = 5.0          # 模拟总时长 5 秒
print_interval = 0.2          # 打印间隔 200 ms
next_print_time = 0.0

print("时间 (s)\t高度 (m)")
print("-------------------")

# ---------- 4. 主循环 ----------
while data.time < total_duration:
    mujoco.mj_step(model, data)

    if data.time >= next_print_time:
        height = data.body("block").xpos[2]   # 木块质心 Z 坐标
        print(f"{data.time:.3f}\t{height:.4f}")
        next_print_time += print_interval

    # 若木块掉出视野则提前终止
    if data.body("block").xpos[2] < -5.0:
        print("木块已超出范围，提前终止")
        break

print("模拟结束。")