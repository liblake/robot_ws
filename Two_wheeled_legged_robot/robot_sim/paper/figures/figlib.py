"""论文插图的公共绘图原语（reportlab + DejaVu Sans）。

被 fig3_architecture.py / fig12_schematics.py 共用。改动这里会影响所有图，
改完请对每张图重跑 `--check`。
"""

from __future__ import annotations

import subprocess
from pathlib import Path

from reportlab.lib.colors import Color, black, white
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

FONT_DIR = Path("/usr/share/fonts/truetype/dejavu")
pdfmetrics.registerFont(TTFont("DJ", str(FONT_DIR / "DejaVuSans.ttf")))
pdfmetrics.registerFont(TTFont("DJB", str(FONT_DIR / "DejaVuSans-Bold.ttf")))
REG, BOLD = "DJ", "DJB"

LIGHT = Color(0.94, 0.94, 0.94)
MID = Color(0.85, 0.85, 0.85)
ACCENT = Color(0.80, 0.86, 0.94)
REF = Color(0.45, 0.45, 0.45)          # 参考线/零位线
PART = Color(0.72, 0.78, 0.86)         # 连杆填充
WHEEL = Color(0.55, 0.55, 0.55)        # 车轮填充

RECTS: list[tuple[float, float, float, float]] = []
TEXTS: list[tuple[str, float, float, float, bool, tuple | None]] = []


def reset_trace() -> None:
    RECTS.clear()
    TEXTS.clear()


def rbox(c, x, y, w, h, fill=white, lw=0.8, radius=3.0, dash=None, stroke=black):
    RECTS.append((x, y, w, h))
    c.setFillColor(fill)
    c.setStrokeColor(stroke)
    c.setLineWidth(lw)
    if dash:
        c.setDash(*dash)
    c.roundRect(x, y, w, h, radius, stroke=1, fill=1)
    c.setDash()


def text(c, x, y, s, size=7.0, bold=False, align="center", box=None, color=black):
    TEXTS.append((s, x, y, size, bold, box))
    c.setFont(BOLD if bold else REG, size)
    c.setFillColor(color)
    if align == "center":
        c.drawCentredString(x, y, s)
    elif align == "left":
        c.drawString(x, y, s)
    else:
        c.drawRightString(x, y, s)


def line(c, x1, y1, x2, y2, lw=0.8, dash=None, color=black):
    c.setStrokeColor(color)
    c.setLineWidth(lw)
    if dash:
        c.setDash(*dash)
    c.line(x1, y1, x2, y2)
    c.setDash()


def arrow(c, x1, y1, x2, y2, dashed=False, lw=0.8, head=4.2, color=black):
    line(c, x1, y1, x2, y2, lw=lw, dash=((2.2, 2.2) if dashed else None), color=color)
    c.setFillColor(color)
    dx, dy = x2 - x1, y2 - y1
    n = (dx * dx + dy * dy) ** 0.5
    if n < 1e-6:
        return
    ux, uy = dx / n, dy / n
    px, py = -uy, ux
    p = c.beginPath()
    p.moveTo(x2, y2)
    p.lineTo(x2 - head * ux + head * 0.45 * px, y2 - head * uy + head * 0.45 * py)
    p.lineTo(x2 - head * ux - head * 0.45 * px, y2 - head * uy - head * 0.45 * py)
    p.close()
    c.drawPath(p, stroke=0, fill=1)


def poly_arrow(c, pts, dashed=False, lw=0.8, head=4.2, color=black):
    for i in range(len(pts) - 2):
        line(c, *pts[i], *pts[i + 1], lw=lw, dash=((2.2, 2.2) if dashed else None), color=color)
    arrow(c, *pts[-2], *pts[-1], dashed=dashed, lw=lw, head=head, color=color)


def arc_angle(c, cx, cy, r, a0_deg, a1_deg, lw=0.8, color=black):
    """画角度弧线（角度以 +x 轴为 0，逆时针为正）。"""
    c.setStrokeColor(color)
    c.setLineWidth(lw)
    if a1_deg < a0_deg:
        a0_deg, a1_deg = a1_deg, a0_deg
    c.arc(cx - r, cy - r, cx + r, cy + r, a0_deg, a1_deg - a0_deg)


def thick_link(c, x1, y1, x2, y2, width=7.0, fill=PART, lw=0.8):
    """画一根有厚度的连杆（圆角矩形沿线段方向）。"""
    import math

    dx, dy = x2 - x1, y2 - y1
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return
    ux, uy = dx / n, dy / n
    px, py = -uy * width / 2, ux * width / 2
    c.setFillColor(fill)
    c.setStrokeColor(black)
    c.setLineWidth(lw)
    p = c.beginPath()
    p.moveTo(x1 + px, y1 + py)
    p.lineTo(x2 + px, y2 + py)
    p.lineTo(x2 - px, y2 - py)
    p.lineTo(x1 - px, y1 - py)
    p.close()
    c.drawPath(p, stroke=1, fill=1)


def circle(c, x, y, r, fill=white, lw=0.8):
    c.setFillColor(fill)
    c.setStrokeColor(black)
    c.setLineWidth(lw)
    c.circle(x, y, r, stroke=1, fill=1)


# --------------------------------------------------------------------------- #
# 版式自检
# --------------------------------------------------------------------------- #
def check_layout(page_w: float, page_h: float, pdf: Path, dpi: int = 600) -> None:
    from PIL import Image
    from reportlab.pdfbase.pdfmetrics import stringWidth
    import numpy as np

    png = pdf.with_suffix("")
    subprocess.run(["pdftocairo", "-png", "-r", str(dpi), "-singlefile", str(pdf), str(png)],
                   check=True)
    img = Image.open(png.with_suffix(".png")).convert("L")
    a = np.asarray(img)
    h, w = a.shape
    scale = dpi / 72.0
    ink = a < 200
    ys, xs = np.where(ink)
    print(f"  PNG {w}x{h}px @{dpi}dpi  (page {page_w/72:.2f}x{page_h/72:.2f} in)")
    if len(ys):
        print(f"  内容包围盒: 左 {xs.min()/scale:.1f}, 右 {(w-1-xs.max())/scale:.1f}, "
              f"下 {(h-1-ys.max())/scale:.1f}, 上 {ys.min()/scale:.1f} pt")

    def band_ink(name: str) -> int:
        m = int(2 * scale)
        if name == "上":
            return int(ink[:m].sum())
        if name == "下":
            return int(ink[h - m:].sum())
        if name == "左":
            return int(ink[:, :m].sum())
        return int(ink[:, w - m:].sum())

    bleeds = [n for n in "上下左右" if band_ink(n)]
    print("  最外 2pt 出血: " + ("无 ✓" if not bleeds else f"{bleeds} ← 需修正"))

    def ov(a_, b_):
        ox = min(a_[0] + a_[2], b_[0] + b_[2]) - max(a_[0], b_[0])
        oy = min(a_[1] + a_[3], b_[1] + b_[3]) - max(a_[1], b_[1])
        return ox, oy

    def contains(a_, b_):
        return (a_[0] <= b_[0] and a_[1] <= b_[1]
                and a_[0] + a_[2] >= b_[0] + b_[2] and a_[1] + a_[3] >= b_[1] + b_[3])

    bad = 0
    for i in range(len(RECTS)):
        for j in range(i + 1, len(RECTS)):
            ox, oy = ov(RECTS[i], RECTS[j])
            if ox > 0.5 and oy > 0.5 and not (contains(RECTS[i], RECTS[j])
                                              or contains(RECTS[j], RECTS[i])):
                bad += 1
                print(f"  矩形部分重叠: {RECTS[i]} vs {RECTS[j]}")
    print(f"  矩形 {len(RECTS)} 个，部分重叠 {bad} 处" + ("  ✓" if not bad else "  ← 需修正"))

    over = 0
    checked = 0
    for s, x, y, size, bold, box in TEXTS:
        if box is None:
            continue
        checked += 1
        bx, by, bw_, bh_ = box
        wt = stringWidth(s, BOLD if bold else REG, size)
        left = x - wt / 2 if abs(x - (bx + bw_ / 2)) < 1e-6 else x
        if left < bx + 1 or left + wt > bx + bw_ - 1 or y < by + 1 or y + size * 0.75 > by + bh_ - 1:
            over += 1
            print(f"  文字可能超框: {s!r} 宽 {wt:.1f} / 框宽 {bw_:.1f}")
    print(f"  受检文字 {checked} 条，超框 {over} 处" + ("  ✓" if not over else "  ← 需修正"))

    # 文字互相重叠（只查未指定所在框、且非旋转的文字）
    boxes = []
    for s, x, y, size, bold, box in TEXTS:
        if box is not None or not s.strip():
            continue
        wt = stringWidth(s, BOLD if bold else REG, size)
        if abs(x - round(x, 6)) >= 0 and False:
            pass
        left = x - wt / 2          # text() 默认居中；左右对齐的也按包围盒近似
        boxes.append((s, left, y - 1.5, wt, size + 2.0, x, wt))

    clash = 0
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            s1, x1, y1, w1, h1, _, _ = boxes[i]
            s2, x2, y2, w2, h2, _, _ = boxes[j]
            ox = min(x1 + w1, x2 + w2) - max(x1, x2)
            oy = min(y1 + h1, y2 + h2) - max(y1, y2)
            if ox > 0.6 and oy > 0.6:
                clash += 1
                print(f"  文字重叠: {s1!r} 与 {s2!r}  (重叠 {ox:.1f}x{oy:.1f} pt)")
    print(f"  文字互检 {len(boxes)} 条，重叠 {clash} 处" + ("  ✓" if not clash else "  ← 需修正"))
