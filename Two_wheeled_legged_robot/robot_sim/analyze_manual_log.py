#!/usr/bin/env python3
"""分析手柄手动跑出来的遥测日志（logs/manual/run_*/telemetry.csv）。

和 paper/run_cases.py 产出的论文级 CSV 不同，手动日志有两处要转换：
  1. roll / pitch 两列的单位是**弧度**（论文 CSV 是 roll_deg / pitch_deg，度）；
  2. 列名是 time / pos_y，不是 t / base_y。
另外手动驾驶是「自由跑」，同一个 y 区间可能来回穿越好几次，所以本脚本会先把
日志切成若干趟「窗口内持续前进」的连续片段，方便你挑最干净的那一趟来算指标。

典型用法：

    # 单次：看这一趟过坡时侧倾有多大
    .venv/bin/python analyze_manual_log.py logs/manual/run_20260919_174119

    # 只看 5 m 梯形坡那一段的机身 y，并列出检测到的每一趟
    .venv/bin/python analyze_manual_log.py logs/manual/run_20260919_174119 \
        --window-y 0.22:5.22 --pass 1

    # 两次对比（前馈关 vs 开）
    .venv/bin/python analyze_manual_log.py \
        logs/manual/run_20260919_173421 logs/manual/run_20260919_174119 \
        --label "rate FF off" --label "rate FF on" \
        --x y --window-y 0.22:5.22 --pass 1 --out /tmp/ff_compare.png
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SIM_ROOT = Path(__file__).resolve().parent
MIN_PASS_STEPS = 150   # 短于 0.3 s 的片段不算一趟穿越
MIN_PASS_SPAN_Y = 0.2  # 纵向跨度小于 0.2 m 的片段只是在原地蹭，也不算


def load_log(path: Path) -> dict[str, object]:
    """读一条手动遥测 CSV；角度换算成度，并算出本体前向速度。"""
    csv_path = path / "telemetry.csv" if path.is_dir() else path
    if not csv_path.is_file():
        raise FileNotFoundError(f"找不到遥测文件：{csv_path}")
    with csv_path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError(f"{csv_path} 是空的")

    def col(name: str) -> np.ndarray:
        return np.array([float(r.get(name) or 0.0) for r in rows])

    q = np.column_stack([col("quat_w"), col("quat_x"), col("quat_y"), col("quat_z")])
    vel = np.column_stack([col("vel_x"), col("vel_y"), col("vel_z")])
    # 本体 +Y 轴在世界系下的方向 = 四元数旋转矩阵的第 1 列；前向速度是它的投影。
    w, x, yy, z = q.T
    fwd_axis = np.column_stack([
        2.0 * (x * yy - w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (yy * z + w * x),
    ])
    data = {
        "path": csv_path,
        "t": col("time"),
        "y": col("pos_y"),
        "x": col("pos_x"),
        "z": col("pos_z"),
        "roll": np.degrees(col("roll")),
        "pitch": np.degrees(col("pitch")),
        "yaw": np.degrees(col("yaw")),
        "v_fwd": np.einsum("ij,ij->i", vel, fwd_axis),
        "speed": np.linalg.norm(vel, axis=1),
        "contact": col("contact_count"),
    }
    data["airborne"] = (data["contact"] == 0).astype(int)
    return data


def parse_window(text: str, name: str) -> tuple[float, float]:
    parts = text.replace(",", ":").split(":")
    if len(parts) != 2:
        raise ValueError(f"{name} 需要写成 LO:HI，例如 0.22:5.22（收到 {text!r}）")
    lo, hi = (float(v) for v in parts)
    if hi <= lo:
        raise ValueError(f"{name} 的上界必须大于下界（收到 {text!r}）")
    return lo, hi


def forward_passes(data: dict, lo: float, hi: float,
                   v_min: float = 0.05) -> list[tuple[int, int]]:
    """把日志切成若干段「机身 y 在 [lo, hi] 内且确实在前进」的连续片段。"""
    inside = (data["y"] >= lo) & (data["y"] <= hi) & (data["v_fwd"] > v_min)
    spans: list[tuple[int, int]] = []
    start = None
    for i, flag in enumerate(inside):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            spans.append((start, i - 1))
            start = None
    if start is not None:
        spans.append((start, len(inside) - 1))
    return [
        (a, b)
        for a, b in spans
        if b - a + 1 >= MIN_PASS_STEPS
        and float(data["y"][b] - data["y"][a]) >= MIN_PASS_SPAN_Y
    ]


def describe(data: dict, idx: np.ndarray) -> str:
    roll = data["roll"][idx]
    pitch = data["pitch"][idx]
    t = data["t"][idx]
    y = data["y"][idx]
    return (
        f"t={t[0]:6.2f}~{t[-1]:6.2f}s  y={y[0]:5.2f}~{y[-1]:5.2f}m  "
        f"roll p2p={np.ptp(roll):6.3f}°  |roll|max={np.abs(roll).max():6.3f}°  "
        f"|pitch|max={np.abs(pitch).max():5.2f}°  "
        f"v_fwd={data['v_fwd'][idx].mean():5.3f} m/s  air={int(data['airborne'][idx].sum())}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="分析 logs/manual/run_*/telemetry.csv：算侧倾峰峰值并画对比图。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("logs", nargs="+", type=Path,
                        help="run 目录或 telemetry.csv 路径（可给多个做对比）")
    parser.add_argument("--label", action="append", default=[],
                        help="图例名，按 logs 顺序对应；不给就用目录名")
    parser.add_argument("--x", choices=("time", "y"), default="time",
                        help="横轴：time（仿真时间，默认）或 y（机身纵向位置）")
    parser.add_argument("--window-t", metavar="LO:HI",
                        help="只看这段时间（秒），例如 3.0:5.0")
    parser.add_argument("--window-y", metavar="LO:HI",
                        help="只看这段机身 y（米）；过坡建议用它，例如 0.22:5.22")
    parser.add_argument("--pass", dest="pass_no", type=int, default=None,
                        help="配合 --window-y：只取第 N 趟前进穿越（从 1 开始）")
    parser.add_argument("--v-min", type=float, default=0.05,
                        help="判定「在前进」的速度阈值，m/s（默认 0.05）")
    parser.add_argument("--no-pitch", action="store_true",
                        help="只画侧倾，默认会多画一栏俯仰做对照")
    parser.add_argument("--title", default=None, help="图标题")
    parser.add_argument("--out", type=Path, default=None,
                        help="输出 PNG 路径（默认写到 paper/figures/manual_compare_<时间戳>.png）")
    # parse_intermixed_args 允许把路径和 --label 交错写，例如
    # `run_A --label "ff off" run_B --label "ff on"`。
    args = parser.parse_intermixed_args()

    if args.window_t and args.window_y:
        parser.error("--window-t 和 --window-y 只能用其中一个")
    if args.pass_no is not None and not args.window_y:
        parser.error("--pass 需要配合 --window-y 使用")

    runs = []
    for i, path in enumerate(args.logs):
        data = load_log(path)
        data["label"] = args.label[i] if i < len(args.label) else data["path"].parent.name
        runs.append(data)

    lo = hi = None
    if args.window_t:
        lo, hi = parse_window(args.window_t, "--window-t")
    elif args.window_y:
        lo, hi = parse_window(args.window_y, "--window-y")

    colors = ["#c1272d", "#1f6fb4", "#2e8b57", "#8a5a2b"]
    panels = 1 if args.no_pitch else 2
    fig, axes = plt.subplots(panels, 1, figsize=(7.6, 2.6 * panels),
                             sharex=True, squeeze=False)
    axes = axes[:, 0]
    xkey, xlabel = ("y", "Base y (m)") if args.x == "y" else ("t", "Time (s)")

    for data, color in zip(runs, colors):
        label = data["label"]
        print(f"\n=== {label}  ({data['path']}) ===")
        if lo is None:
            idx = np.arange(len(data["t"]))
        elif args.window_y:
            spans = forward_passes(data, lo, hi, args.v_min)
            if not spans:
                print(f"  y∈[{lo}, {hi}] 内没有检测到前进穿越（速度阈值 {args.v_min} m/s）")
                continue
            print(f"  y∈[{lo}, {hi}] 内检测到 {len(spans)} 趟前进穿越：")
            for n, (a, b) in enumerate(spans, start=1):
                print(f"    #{n}  {describe(data, np.arange(a, b + 1))}")
            pick = args.pass_no or 1
            if pick > len(spans):
                print(f"  --pass {pick} 超出范围（只有 {len(spans)} 趟），跳过")
                continue
            if args.pass_no is None and len(spans) > 1:
                print("  （默认取第 1 趟；想换其它趟加 --pass N）")
            a, b = spans[pick - 1]
            idx = np.arange(a, b + 1)
        else:
            idx = np.where((data["t"] >= lo) & (data["t"] <= hi))[0]
            if idx.size == 0:
                print(f"  t∈[{lo}, {hi}] 内没有数据点")
                continue
        print("  → " + describe(data, idx))
        axes[0].plot(data[xkey][idx], data["roll"][idx], color=color, lw=0.9, label=label)
        if not args.no_pitch:
            axes[1].plot(data[xkey][idx], data["pitch"][idx], color=color, lw=0.9, label=label)

    axes[0].set_ylabel("Roll (deg)")
    axes[0].set_title(args.title or "Manual gamepad run — body roll", fontsize=9)
    axes[0].legend(loc="best", fontsize=7)
    if not args.no_pitch:
        axes[1].set_ylabel("Pitch (deg)")
        axes[1].set_title("Pitch (control channel: should stay unaffected)", fontsize=9)
        axes[1].legend(loc="best", fontsize=7)
    axes[-1].set_xlabel(xlabel)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()

    out = args.out or (SIM_ROOT / "paper" / "figures"
                       / f"manual_compare_{dt.datetime.now():%Y%m%d_%H%M%S}.png")
    out = out.expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    print(f"\n图已保存：{out.resolve()}")
    print(f"（用图片查看器打开：xdg-open {out.resolve()}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
