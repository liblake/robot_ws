"""控制回路延迟细扫（论文第 4 章要用"延迟预算"这条结论）。

复用 `test_robustness.py` 的场景与判据，只扫描延迟这一个维度，
给出"哪个延迟开始退化、哪个延迟开始失稳"的完整曲线。

用法：
    cd robot_sim
    .venv/bin/python -m paper.delay_scan                    # 全部四个场景
    .venv/bin/python -m paper.delay_scan --only stand       # 只跑站立（最快）
    .venv/bin/python -m paper.delay_scan --delays 2 4 6 8 10 12 16 20

输出：控制台表格 + paper/data/delay_scan.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROBOT_SIM = Path(__file__).resolve().parent.parent
if str(ROBOT_SIM) not in sys.path:
    sys.path.insert(0, str(ROBOT_SIM))

import test_robustness as R  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None, help="只跑某个场景：stand / drive / turn / ramp")
    ap.add_argument("--delays", type=float, nargs="*",
                    default=[2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0])
    args = ap.parse_args()

    scenarios = [args.only] if args.only else list(R.SCENARIOS)
    results: dict[str, dict] = {}

    print("=" * 84)
    print("控制回路延迟细扫（数值为各场景最坏指标相对标称门槛的倍率，>1 即超门槛）")
    print("=" * 84)
    header = f"{'延迟 (ms)':<12}" + "".join(f"{n:^17}" for n in scenarios)
    print(header)
    print("-" * 84)

    for delay in args.delays:
        setting = R.Setting("延迟", f"{delay:g} ms", delay_ms=delay)
        cells = []
        for name in scenarios:
            try:
                metrics = R.SCENARIOS[name](setting)
                verdict, detail = R.evaluate(name, metrics)
            except Exception as exc:  # noqa: BLE001
                metrics, verdict, detail = {}, "ERROR", f"{type(exc).__name__}: {exc}"

            worst_ratio, worst_key = 0.0, ""
            for key, limit in R.THRESHOLDS[name].items():
                value = metrics.get(key)
                if value is None or not np.isfinite(value):
                    continue
                ratio = value / limit
                if ratio > worst_ratio:
                    worst_ratio, worst_key = ratio, key
            fell = metrics.get("fell")
            cells.append(f"{verdict}" if verdict != "OK" else "OK")
            results[f"{delay:g}|{name}"] = {
                "delay_ms": delay, "scenario": name, "verdict": verdict,
                "detail": detail, "fell_at_s": fell,
                "worst_ratio": round(worst_ratio, 2), "worst_metric": worst_key,
                "metrics": {k: (None if v is None else float(v))
                            for k, v in metrics.items() if isinstance(v, (int, float))},
            }
        print(f"{delay:<12g}" + "".join(f"{c:^17}" for c in cells))

    print("-" * 84)
    print("逐场景详情：")
    for name in scenarios:
        print(f"\n[{name}]")
        for delay in args.delays:
            rec = results[f"{delay:g}|{name}"]
            mark = "摔倒" if rec["fell_at_s"] is not None else rec["worst_metric"]
            extra = f"，摔倒于 {rec['fell_at_s']:.2f} s" if rec["fell_at_s"] is not None else ""
            print(f"  {delay:5g} ms  {rec['verdict']:<6} 最坏 {mark} = {rec['worst_ratio']:.2f}× 门槛{extra}")

    out = Path(__file__).resolve().parent / "data" / "delay_scan.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n已写入 {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
