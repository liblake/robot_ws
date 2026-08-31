#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 MuJoCo 中打开 robot.urdf（浮动底座 + 地面）。

用法（在 WSL 中，先把整个 robot_urdf 文件夹复制过去）:
    pip install mujoco
    python open_in_mujoco.py

Windows 11 的 WSL2 自带 WSLg，查看器窗口会直接弹出；
如果窗口不出现，请确认已启用 WSLg 或安装了 X Server。
"""
from pathlib import Path
import mujoco

HERE = Path(__file__).resolve().parent

# 1) 读取 URDF，给 base_link 加一个自由关节，让机器人可以漂浮并落回地面。
#    （MuJoCo 的 URDF 扩展语法：在 <link> 里嵌 <mujoco><freejoint/></mujoco>）
urdf = (HERE / "robot.urdf").read_text(encoding="utf-8")
urdf = urdf.replace(
    '<link name="base_link">',
    '<link name="base_link">\n    <mujoco>\n      <freejoint/>\n    </mujoco>',
    1,
)
float_urdf = HERE / "robot_float.urdf"
float_urdf.write_text(urdf, encoding="utf-8")

# 2) 包一层 MJCF 场景：保留视觉/碰撞，加一块地面。
#    strippath="false" 保证 meshes/ 相对路径能正确解析。
scene = HERE / "robot_scene.xml"
scene.write_text(
    """<mujoco model="wheel_robot">
  <compiler strippath="false" discardvisual="false" balanceinertia="true" autolimits="true"/>
  <include file="robot_float.urdf"/>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" pos="0 0 -0.5" rgba="0.8 0.8 0.8 1"/>
  </worldbody>
</mujoco>
""",
    encoding="utf-8",
)

# 3) 加载并打开交互式查看器。
model = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(model)
print(f"加载成功：{model.nbody} 个 body / {model.njnt} 个关节 / {model.ngeom} 个几何体")
print("提示：关闭查看器窗口即退出；可在左侧 UI 里拖动关节。")
mujoco.viewer.launch(model, data)
