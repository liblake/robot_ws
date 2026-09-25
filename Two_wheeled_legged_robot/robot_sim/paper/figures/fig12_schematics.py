"""生成论文 图 1（含 a/b 两栏）与 图 2。

图 1(a)：MuJoCo 渲染的机器人外形（由 fig1_render.py 产出 fig1_render.png）
图 1(b)：矢状面单腿几何（L1、L2、R、e、θ1、θ2、h）——几何取自 URDF 与 serial_leg_ik
图 2   ：等效双轮二阶倒立摆模型（m_b、m_p、m_w、I_b、I_p、L_p、θ_p、φ、W）

用法（在 robot_sim/ 下执行，需先 `uv sync --extra figures`）：
    cd robot_sim
    .venv/bin/python paper/figures/fig12_schematics.py --check
    # 或者：uv run python paper/figures/fig12_schematics.py --check

几何来源：`src/controllers/serial_leg_ik.py`（THIGH0/SHIN0/L1/L2/HIP_Z_BASE），
名义姿态由 `SerialLegIk.angles_from_base_height(0.37, "right", -0.075)` 解出，
因此图里的关节角与正文式 (3) 完全一致。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from reportlab.lib.colors import Color, black, white
from reportlab.pdfgen import canvas as rl_canvas

HERE = Path(__file__).resolve().parent
ROBOT_SIM = HERE.parent.parent
if str(ROBOT_SIM) not in sys.path:
    sys.path.insert(0, str(ROBOT_SIM))

import figlib as L  # noqa: E402
from src.controllers.serial_leg_ik import (  # noqa: E402
    HIP_Z_BASE, L1, L2, SHIN0, THIGH0, SerialLegIk,
)

WHEEL_R = 0.070
NOMINAL_H = 0.37
WHEEL_OFFSET = -0.075


# --------------------------------------------------------------------------- #
# 小工具
# --------------------------------------------------------------------------- #
def rot_x(v, deg):
    """矢状面内绕 +X 旋转（与 serial_leg_ik 的约定一致）：(y,z) 形式。"""
    a = math.radians(deg)
    y, z = v
    return (y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a))


def ang_deg(v):
    return math.degrees(math.atan2(v[1], v[0]))


def text_rot(c, x, y, s, deg, size=7.0, bold=False, color=black):
    c.saveState()
    c.translate(x, y)
    c.rotate(deg)
    c.setFont(L.BOLD if bold else L.REG, size)
    c.setFillColor(color)
    c.drawCentredString(0, 0, s)
    c.restoreState()


class Frame:
    """把 (y,z) 米坐标映射到画布 pt 坐标（y 向右、z 向上）。"""

    def __init__(self, x0, y0, w, h, ymin, ymax, zmin, zmax, pad=16.0):
        usable_w = w - 2 * pad
        usable_h = h - 2 * pad
        self.s = min(usable_w / (ymax - ymin), usable_h / (zmax - zmin))
        span_y = (ymax - ymin) * self.s
        span_z = (zmax - zmin) * self.s
        self.ox = x0 + pad + (usable_w - span_y) / 2 - ymin * self.s
        self.oy = y0 + pad + (usable_h - span_z) / 2 - zmin * self.s

    def pt(self, p) -> tuple[float, float]:
        y, z = p
        return (self.ox + y * self.s, self.oy + z * self.s)

    def m(self, pt_len: float) -> float:
        return pt_len * self.s


# --------------------------------------------------------------------------- #
# 图 1(b)：矢状面单腿几何
# --------------------------------------------------------------------------- #
def draw_leg_geometry(c, x0, y0, w, h) -> None:
    ik = SerialLegIk()
    th1, th2 = ik.angles_from_base_height(NOMINAL_H, "right", WHEEL_OFFSET)

    hip = (0.0, 0.0)
    thigh = rot_x(THIGH0, th1)
    knee = (hip[0] + thigh[0], hip[1] + thigh[1])
    shank = rot_x(SHIN0, th1 + th2)
    wheel = (knee[0] + shank[0], knee[1] + shank[1])
    base = (0.0, -HIP_Z_BASE)                       # 机体原点在髋上方 0.070
    thigh0 = THIGH0
    shank0_rot = rot_x(SHIN0, th1)                  # θ2 = 0 时的参考方向

    f = Frame(x0, y0, w, h, ymin=-0.345, ymax=0.105, zmin=-0.345, zmax=0.115)
    P = f.pt

    # 参考线：过髋的竖直线、过轮心的水平线、过轮心的竖直线（量 e 用）
    L.line(c, *P((0.0, 0.115)), *P((0.0, -0.345)), lw=0.6, dash=(2.2, 2.2), color=L.REF)
    L.line(c, *P((-0.345, wheel[1])), *P((0.105, wheel[1])), lw=0.6, dash=(2.2, 2.2), color=L.REF)
    # 机体原点与机体下缘
    L.line(c, *P((-0.13, base[1])), *P((0.13, base[1])), lw=1.4)

    # 零位参考连杆（虚线）
    L.thick_link(c, *P(hip), *P((hip[0] + thigh0[0] * 0.62, hip[1] + thigh0[1] * 0.62)),
                 width=6.0, fill=Color(0.88, 0.88, 0.88), lw=0.6)
    L.thick_link(c, *P(knee), *P((knee[0] + shank0_rot[0] * 0.55,
                                  knee[1] + shank0_rot[1] * 0.55)),
                 width=6.0, fill=Color(0.88, 0.88, 0.88), lw=0.6)

    # 实际连杆
    L.thick_link(c, *P(hip), *P(knee), width=8.0)
    L.thick_link(c, *P(knee), *P(wheel), width=8.0)

    # 车轮
    wx, wy = P(wheel)
    L.circle(c, wx, wy, f.m(WHEEL_R), fill=L.WHEEL)
    L.circle(c, wx, wy, 1.4, fill=black)
    # 半径标注
    L.arrow(c, wx, wy, wx + f.m(WHEEL_R) * 0.707, wy - f.m(WHEEL_R) * 0.707, lw=0.7, head=3.0)
    L.text(c, wx + f.m(WHEEL_R) * 0.78, wy - f.m(WHEEL_R) * 0.62, "R", size=L.FS_TINY
           if hasattr(L, "FS_TINY") else 6.8, bold=True)

    # 关节
    for p in (hip, knee):
        px, py = P(p)
        L.circle(c, px, py, 2.1, fill=white)

    # 连杆长度标注
    mid_t = ((hip[0] + knee[0]) / 2, (hip[1] + knee[1]) / 2)
    mid_s = ((knee[0] + wheel[0]) / 2, (knee[1] + wheel[1]) / 2)
    text_rot(c, *P(mid_t), "L1", ang_deg(thigh), size=7.2, bold=True)
    text_rot(c, *P(mid_s), "L2", ang_deg(shank), size=7.2, bold=True)
    text_rot(c, *P(((hip[0] + knee[0] * 2) / 3, (hip[1] + knee[1] * 2) / 3)),
             "zero pose", ang_deg(thigh0), size=6.0, color=L.REF)

    # 角度弧
    r_deg = 0.075
    x, y = P(hip)
    L.arc_angle(c, x, y, f.m(r_deg), ang_deg(thigh), ang_deg(thigh0), lw=0.7)
    L.text(c, x + f.m(r_deg + 0.045), y + f.m(0.035), "θ1", size=7.2, bold=True)
    x, y = P(knee)
    L.arc_angle(c, x, y, f.m(r_deg), ang_deg(shank0_rot), ang_deg(shank), lw=0.7)
    L.text(c, x + f.m(0.10), y - f.m(0.02), "θ2", size=7.2, bold=True)

    # 尺寸标注：h（机体原点到轮心）、h_hip（髋到轮心）、e（髋竖直线到轮心）
    xh, _ = P((0.085, 0.0))
    L.arrow(c, xh, P(base)[1], xh, P((0, wheel[1]))[1], lw=0.7, head=3.0)
    text_rot(c, xh + 7, (P(base)[1] + P((0, wheel[1]))[1]) / 2, "h", 90, size=7.2, bold=True)
    xh2, _ = P((0.135, 0.0))
    L.arrow(c, xh2, P(hip)[1], xh2, P((0, wheel[1]))[1], lw=0.7, head=3.0)
    text_rot(c, xh2 + 7, (P(hip)[1] + P((0, wheel[1]))[1]) / 2, "h_hip", 90, size=6.4)

    y_e = P((0.0, wheel[1] - WHEEL_R - 0.035))[1]
    x_hip_line, _ = P((0.0, 0.0))
    L.arrow(c, x_hip_line, y_e, P(wheel)[0], y_e, lw=0.7, head=3.0)
    L.line(c, x_hip_line, y_e - 4, x_hip_line, y_e + 4, lw=0.5, color=L.REF)
    L.line(c, P(wheel)[0], y_e - 4, P(wheel)[0], y_e + 4, lw=0.5, color=L.REF)
    L.text(c, (x_hip_line + P(wheel)[0]) / 2, y_e + 3, "e", size=7.2, bold=True)

    # 机体原点标注
    bx, by = P(base)
    L.circle(c, bx, by, 1.6, fill=black)
    L.text(c, bx - 6, by + 5, "base origin", size=6.4, align="right")
    # 髋关节标注
    hx, hy = P(hip)
    L.text(c, hx - 6, hy - 8, "hip", size=6.4, align="right")
    kx, ky = P(knee)
    L.text(c, kx + 8, ky + 4, "knee", size=6.4, align="left")
    wx, wy = P(wheel)
    L.text(c, wx - 4, wy + f.m(WHEEL_R) + 4, "wheel center", size=6.4, align="left")


# --------------------------------------------------------------------------- #
# 图 2：等效双轮二阶倒立摆
# --------------------------------------------------------------------------- #
def draw_pendulum(c, x0, y0, w, h) -> None:
    f = Frame(x0, y0, w, h, ymin=-0.34, ymax=0.28, zmin=-0.19, zmax=0.52)
    P = f.pt
    axle = (0.0, 0.0)
    theta_p = 9.0          # 摆杆倾角（示意）
    phi = -7.0             # 机体俯仰角（示意）
    lp = 0.30
    hip = rot_x((0.0, lp), theta_p)

    # 地面与参考竖线
    L.line(c, *P((-0.40, -WHEEL_R)), *P((0.30, -WHEEL_R)), lw=0.9)
    L.line(c, *P((0, -WHEEL_R)), *P((0, 0.56)), lw=0.6, dash=(2.2, 2.2), color=L.REF)

    # 车轮（后轮用浅色表示在另一侧）
    ax, ay = P(axle)
    L.circle(c, ax + 6, ay - 2, f.m(WHEEL_R), fill=Color(0.78, 0.78, 0.78))
    L.circle(c, ax, ay, f.m(WHEEL_R), fill=L.WHEEL)
    L.circle(c, ax, ay, 1.5, fill=black)
    L.text(c, ax - f.m(WHEEL_R) - 6, ay - 2, "rear wheel", size=6.2,
           align="right", color=L.REF)
    L.text(c, ax + f.m(WHEEL_R) + 6, ay - 10, "m_w, I_w, R", size=6.4, align="left")

    # 摆杆（可变长度 L_p）
    L.thick_link(c, *P(axle), *P(hip), width=8.0)
    mid = (hip[0] * 0.55, hip[1] * 0.55)
    text_rot(c, *P(mid), "L_p", ang_deg(hip), size=7.2, bold=True)
    L.text(c, P(mid)[0] - 10, P(mid)[1] - 2, "m_p, I_p", size=6.4, align="right")

    # 机体（绕髋转动，俯仰角 φ）
    bw, bh = 0.42, 0.115
    cx = hip[0] + bw * 0.5 * math.sin(math.radians(-phi))
    cz = hip[1] + 0.055
    c.saveState()
    bx, by = P((cx, cz))
    c.translate(bx, by)
    c.rotate(phi)
    c.setFillColor(L.ACCENT)
    c.setStrokeColor(black)
    c.setLineWidth(0.9)
    c.roundRect(-f.m(bw) / 2, -f.m(bh) / 2, f.m(bw), f.m(bh), 3, stroke=1, fill=1)
    c.restoreState()
    L.text(c, bx, by + 3, "torso", size=7.0, bold=True)
    L.text(c, bx, by - 6, "m_b, I_b", size=6.4)
    L.circle(c, bx, by, 1.6, fill=black)

    # 髋关节
    L.circle(c, *P(hip), 2.1, fill=white)
    L.text(c, P(hip)[0] - 9, P(hip)[1] - 9, "hip", size=6.4, align="right")

    # 角度弧：θ_p（摆杆相对竖直）、φ（机体相对水平）
    px, py = P(axle)
    L.arc_angle(c, px, py, f.m(0.13), 90, 90 - theta_p, lw=0.7)
    L.text(c, px + f.m(0.045), py + f.m(0.145), "θ_p", size=7.2, bold=True)
    L.line(c, *P(hip), *P((hip[0], hip[1] + 0.10)), lw=0.6, dash=(2.2, 2.2), color=L.REF)
    hx, hy = P(hip)
    L.arc_angle(c, hx, hy, f.m(0.10), 90, 90 - phi if phi < 0 else 90, lw=0.7)
    L.text(c, hx + f.m(0.12), hy + f.m(0.02), "φ", size=7.2, bold=True)

    # 轮距 W 标注
    yW = P((0, -WHEEL_R - 0.055))[1]
    L.arrow(c, ax - f.m(WHEEL_R), yW, ax + f.m(WHEEL_R) + 6, yW, lw=0.7, head=3.0)
    L.text(c, ax + f.m(WHEEL_R / 2), yW - 9, "W", size=7.0, bold=True)
    L.line(c, ax, yW - 5, ax, ay - f.m(WHEEL_R), lw=0.5, color=L.REF)

    # 控制输入提示
    L.arrow(c, ax - f.m(0.30), ay, ax - f.m(WHEEL_R) - 4, ay, lw=0.9, head=3.4)
    L.text(c, ax - f.m(0.30) - 4, ay + 3, "τ_w", size=7.0, bold=True, align="right")


# --------------------------------------------------------------------------- #
# 组装
# --------------------------------------------------------------------------- #
def build_fig1(pdf: Path) -> None:
    from reportlab.lib.utils import ImageReader

    W_FIG, H_FIG = 5.55 * 72, 2.85 * 72
    c = rl_canvas.Canvas(str(pdf), pagesize=(W_FIG, H_FIG))

    img = ImageReader(str(HERE / "fig1_render.png"))
    iw, ih = img.getSize()
    a_w = 2.20 * 72
    a_h = a_w * ih / iw
    a_x, a_y = 6.0, H_FIG / 2 - a_h / 2
    c.drawImage(img, a_x, a_y, a_w, a_h, mask=None)

    b_x = a_x + a_w + 10.0
    b_w = W_FIG - b_x - 4.0
    draw_leg_geometry(c, b_x, 6.0, b_w, H_FIG - 26.0)

    L.text(c, a_x + a_w / 2, H_FIG - 10, "(a) 3D view", size=7.6, bold=True)
    L.text(c, b_x + b_w / 2, H_FIG - 10, "(b) Sagittal leg geometry", size=7.6, bold=True)
    c.showPage()
    c.save()


def build_fig2(pdf: Path) -> None:
    W_FIG, H_FIG = 2.60 * 72, 2.10 * 72
    c = rl_canvas.Canvas(str(pdf), pagesize=(W_FIG, H_FIG))
    draw_pendulum(c, 0, 0, W_FIG, H_FIG)
    c.showPage()
    c.save()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    if not (HERE / "fig1_render.png").exists():
        raise SystemExit("缺少 fig1_render.png，先运行 fig1_render.py")

    L.reset_trace()
    build_fig1(HERE / "fig1_schematic.pdf")
    print("wrote fig1_schematic.pdf")
    if args.check:
        L.check_layout(5.55 * 72, 2.85 * 72, HERE / "fig1_schematic.pdf")

    L.reset_trace()
    build_fig2(HERE / "fig2_pendulum.pdf")
    print("wrote fig2_pendulum.pdf")
    if args.check:
        L.check_layout(2.60 * 72, 2.10 * 72, HERE / "fig2_pendulum.pdf")
