#!/usr/bin/env python3
"""把 robot.urdf 转成 MuJoCo 可用的 MJCF（robot.xml）。

背景：MuJoCo 的 URDF 导入器会把根 link（base_link，即机身）摊平焊进 worldbody，
导致机身是静态的、拖不动也选不中。本脚本把它重新包进一个带 <freejoint> 的 body，
让整个机器人可以自由抓取、移动。

用法：python convert_robot.py
以后 robot.urdf 改了，重跑本脚本即可重新生成 robot.xml。
"""
import mujoco as mj
import xml.etree.ElementTree as ET

URDF = "assets/robot_urdf/robot.urdf"
OUT = "assets/robot_urdf/robot.xml"

# 1) URDF -> MJCF（MuJoCo 内部会摊平 base_link）
spec = mj.MjSpec.from_file(URDF)
root = ET.fromstring(spec.to_xml())

# 2) 把 worldbody 的所有直接子元素（机身 geom + link_002/link_005）包进 base_link
worldbody = root.find("worldbody")
children = list(worldbody)
for c in children:
    worldbody.remove(c)

base = ET.SubElement(worldbody, "body")
base.set("name", "base_link")
base.set("pos", "0 0 0")
ET.SubElement(base, "freejoint")  # 6 自由度，让机器人可抓取
for c in children:
    base.append(c)

# 3) 编译器设置：inertiafromgeom 让 base_link 从网格自动算质量/惯量（否则无质量）
compiler = root.find("compiler")
if compiler is None:
    compiler = ET.Element("compiler")
    root.insert(0, compiler)
compiler.set("angle", "radian")
compiler.set("inertiafromgeom", "true")

# 4) 写回
tree = ET.ElementTree(root)
ET.indent(tree, space="  ")
tree.write(OUT, encoding="utf-8", xml_declaration=True)
print(f"已生成 {OUT}")
