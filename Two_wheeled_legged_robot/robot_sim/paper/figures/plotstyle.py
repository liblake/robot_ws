"""论文数据图的统一 matplotlib 样式（IEEE 会议）。

用法：
    import plotstyle as PS
    PS.apply()
    fig, ax = PS.subplots(...)      # 或直接用 plt
    PS.save(fig, "fig4_terrain")    # 同时导出 PDF（矢量）与 600 dpi PNG

要点：
- **必须在 import matplotlib 之前 import 本模块**（它负责设置 MPLCONFIGDIR，
  否则沙箱下 ~/.config 只读会导致 matplotlib 每次回退到临时目录并打印告警）；
- 字体 DejaVu Sans（支持 θ、φ、τ 等符号），正文 8 pt、刻度 7.5 pt、图例 7 pt，
  满足"100% 阅读时清晰可读"的要求；
- 线宽 ≥1.2、网格淡、无边框图例；
- 三色配色在黑白打印下也能区分（深蓝 / 深红 / 中灰 + 线型区分）；
- MPLCONFIGDIR 指向项目内可写目录，避免沙箱下 ~/.config 只读导致的告警。
"""

from __future__ import annotations

import os
from pathlib import Path

_CACHE = Path(__file__).resolve().parent.parent / ".cache" / "mpl"
_CACHE.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_CACHE))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

DPI = 600

# 配色：深蓝 / 深红 / 中灰 / 墨绿（黑白打印仍可区分）
C1 = "#1f4e79"
C2 = "#c00000"
C3 = "#7f7f7f"
C4 = "#2f6f4f"
COLORS = (C1, C2, C3, C4)


def apply() -> None:
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["DejaVu Sans"],
        "font.size": 8.0,
        "axes.labelsize": 8.0,
        "axes.titlesize": 8.0,
        "legend.fontsize": 7.0,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "axes.linewidth": 0.8,
        "axes.grid": True,
        "grid.linewidth": 0.4,
        "grid.alpha": 0.30,
        "lines.linewidth": 1.2,
        "lines.markersize": 3.6,
        "legend.frameon": False,
        "legend.handlelength": 1.6,
        "legend.borderaxespad": 0.2,
        "legend.labelspacing": 0.25,
        "mathtext.fontset": "dejavusans",
        "figure.dpi": 150,
        "savefig.dpi": DPI,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
        "axes.axisbelow": True,
    })


def save(fig, stem: str | Path) -> None:
    """同时导出 PDF（投稿用矢量）与 PNG（预览/Word 插图用）。"""
    base = Path(stem)
    for ext in ("pdf", "png"):
        fig.savefig(base.with_suffix(f".{ext}"))
    print(f"wrote {base.with_suffix('.pdf')} 和 {base.with_suffix('.png')}")


def panel_label(ax, label: str) -> None:
    """左上角 (a)/(b) 标号，放在坐标轴外侧以免遮挡曲线。"""
    ax.text(-0.16, 1.04, label, transform=ax.transAxes, fontsize=8, fontweight="bold",
            va="bottom", ha="left")
