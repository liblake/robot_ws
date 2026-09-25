"""按名字跑仿真场景，输出 CSV + JSON（论文方案 A：纯仿真）。

用法（在 Two_wheeled_legged_robot/robot_sim 目录下，务必用项目虚拟环境）：

    .venv/bin/python -m paper.run_cases --list                  # 看所有场景
    .venv/bin/python -m paper.run_cases --case stand_12s        # 跑一个场景
    .venv/bin/python -m paper.run_cases --case jump_static_1p0 --viewer
    .venv/bin/python -m paper.run_cases --group terrain         # 跑一组
    .venv/bin/python -m paper.run_cases --all                   # 跑全部
    .venv/bin/python -m paper.run_cases --case ramp_65_03 --set vmc.stand_rate_ff_scale=0 --tag ff_off

输出（默认写到 paper/data/）：
    <name>[_<tag>].csv    逐步遥测（t/相位/速度/姿态/腿高/力矩/接触）
    <name>[_<tag>].json   汇总指标（摔倒、峰峰值、弹道、力矩峰、各分析窗口指标）
    index.json            所有跑过的场景索引（跑论文数据时汇总用）

想加/改场景 → 改 paper/scenarios.py；临时改参数 → 用 --set，不用改代码。
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import os
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
_ROBOT_SIM = _HERE.parent.parent
if str(_ROBOT_SIM) not in sys.path:
    sys.path.insert(0, str(_ROBOT_SIM))

import mujoco  # noqa: E402

from src.controllers.combined import CombinedController  # noqa: E402
from src.controllers.default_params import STAND_PARAMS  # noqa: E402
from src.controllers.jump_trajectory import JumpTrajectory  # noqa: E402
from src.controllers.phase import JumpPhase, JumpPhaseMachine  # noqa: E402
from src.controllers.vmc import LEG_CLOSED_LOOP  # noqa: E402
from src.geometry import wheel_center_world, wheel_center_z  # noqa: E402
from src.launch_mujoco import (  # noqa: E402
    MANUAL_JUMP_PHASE_PARAMS,
    MANUAL_JUMP_TRAJECTORY_PARAMS,
    build_controlled_model,
)
from src.logger import _rpy_from_quaternion  # noqa: E402
from src.state import body_id, extract_sim_state, model_addresses  # noqa: E402
from paper.scenarios import (  # noqa: E402
    URDF_RELATIVE_PATH,
    Scenario,
    groups,
    select,
)

FALL_PITCH_RAD = 1.0
FALL_BASE_Z = 0.20

CSV_COLUMNS = [
    "t", "phase", "cmd_v", "cmd_yaw", "cmd_h",
    "v_fwd", "v_wheel", "pitch_deg", "roll_deg", "yaw_deg",
    "pitch_rate", "roll_rate", "yaw_rate",
    "base_x", "base_y", "base_z", "com_z",
    "h_left", "h_right",
    "tau_hip_l", "tau_hip_r", "tau_knee_l", "tau_knee_r",
    "tau_wheel_l", "tau_wheel_r",
    "contact", "airborne",
    "wheel_y_l", "wheel_z_l", "wheel_y_r", "wheel_z_r",
]

# 输出精度（小数位）。不限制的话 Python 会写出 17 位浮点，单个 CSV 会膨胀到几 MB。
CSV_PRECISION = {
    "t": 4, "cmd_v": 4, "cmd_yaw": 4, "cmd_h": 4, "v_fwd": 4, "v_wheel": 4,
    "pitch_deg": 3, "roll_deg": 3, "yaw_deg": 3,
    "pitch_rate": 4, "roll_rate": 4, "yaw_rate": 4,
    "base_x": 5, "base_y": 5, "base_z": 5, "com_z": 5, "h_left": 5, "h_right": 5,
    "tau_hip_l": 3, "tau_hip_r": 3, "tau_knee_l": 3, "tau_knee_r": 3,
    "tau_wheel_l": 3, "tau_wheel_r": 3,
    "contact": 0, "airborne": 0,
    "wheel_y_l": 5, "wheel_z_l": 5, "wheel_y_r": 5, "wheel_z_r": 5,
}


def _round_row(row: dict) -> dict:
    return {
        key: (round(float(value), CSV_PRECISION.get(key, 4)) if isinstance(value, float) else value)
        for key, value in row.items()
    }


# --------------------------------------------------------------------------- #
# 参数覆盖
# --------------------------------------------------------------------------- #
def _coerce(text: str):
    low = text.strip().lower()
    if low in ("true", "false"):
        return low == "true"
    for caster in (int, float):
        try:
            return caster(text)
        except ValueError:
            continue
    return text


def apply_override(params, dotted: str, value) -> None:
    """按点分路径写入嵌套参数，例如 vmc.stand_rate_ff_scale。"""
    parts = dotted.split(".")
    obj = params
    for part in parts[:-1]:
        if not hasattr(obj, part):
            raise KeyError(f"unknown parameter path: {dotted}")
        obj = getattr(obj, part)
    leaf = parts[-1]
    if not hasattr(obj, leaf):
        raise KeyError(f"unknown parameter: {dotted}")
    setattr(obj, leaf, value)


# --------------------------------------------------------------------------- #
# 逐步状态采样
# --------------------------------------------------------------------------- #
def _forward_velocity(model, data, state) -> float:
    """机身实际前向速度（本体 +Y 在水平面的投影）。与控制器内部口径一致。"""
    rotation = np.asarray(data.xmat[body_id(model, "base_link")]).reshape(3, 3)
    forward = rotation[:2, 1]
    norm = float(np.linalg.norm(forward))
    if norm < 1e-9:
        return 0.0
    return float(np.dot(state.base_linear_velocity[:2], forward / norm))


def _com_height(model, data) -> float:
    masses = np.asarray(model.body_mass, dtype=float)
    return float(np.sum(masses[:, None] * np.asarray(data.xipos), axis=0)[2] / masses.sum())


def _wheel_speed(state) -> float:
    values = list(state.wheel_velocities.values())
    return float(np.mean(values)) if values else 0.0


def _leg_heights(model, data) -> tuple[float, float]:
    base_z = float(data.xpos[body_id(model, "base_link"), 2])
    left = base_z - wheel_center_z(model, data, LEG_CLOSED_LOOP["left"].wheel_body)
    right = base_z - wheel_center_z(model, data, LEG_CLOSED_LOOP["right"].wheel_body)
    return float(left), float(right)


def _wheel_positions(model, data) -> tuple[np.ndarray, np.ndarray]:
    """左右轮心（轮轴中心）在世界系的位置。地形跟随实验直接看轮心的 (y, z)。"""
    return (
        wheel_center_world(model, data, LEG_CLOSED_LOOP["left"].wheel_body),
        wheel_center_world(model, data, LEG_CLOSED_LOOP["right"].wheel_body),
    )


def sample_row(scn, model, data, state, control, addresses, phase) -> dict:
    _roll, yaw = _rpy_from_quaternion(np.asarray(state.base_quaternion, dtype=float))
    h_left, h_right = _leg_heights(model, data)
    wheel_l, wheel_r = _wheel_positions(model, data)
    act = addresses.actuators

    def tau(joint: str) -> float:
        idx = act.get(joint)
        return float(control[idx]) if idx is not None else 0.0

    return _round_row({
        "t": float(data.time),
        "phase": phase.value,
        "cmd_v": float(scn.velocity(float(data.time))),
        "cmd_yaw": float(scn.yaw_rate(float(data.time))),
        "cmd_h": float(scn.height(float(data.time))),
        "v_fwd": _forward_velocity(model, data, state),
        "v_wheel": _wheel_speed(state),
        "pitch_deg": float(np.degrees(state.pitch)),
        "roll_deg": float(np.degrees(state.roll)),
        "yaw_deg": float(np.degrees(yaw)),
        "pitch_rate": float(state.pitch_rate),
        "roll_rate": float(state.roll_rate),
        "yaw_rate": float(state.base_angular_velocity[2]),
        "base_x": float(state.base_position[0]),
        "base_y": float(state.base_position[1]),
        "base_z": float(state.base_position[2]),
        "com_z": _com_height(model, data),
        "h_left": h_left,
        "h_right": h_right,
        "tau_hip_l": tau(LEG_CLOSED_LOOP["left"].hip_joint),
        "tau_hip_r": tau(LEG_CLOSED_LOOP["right"].hip_joint),
        "tau_knee_l": tau(LEG_CLOSED_LOOP["left"].knee_joint),
        "tau_knee_r": tau(LEG_CLOSED_LOOP["right"].knee_joint),
        "tau_wheel_l": tau("link_007_joint"),
        "tau_wheel_r": tau("link_004_joint"),
        "contact": int(state.contact_count),
        "airborne": int(state.contact_count == 0),
        "wheel_y_l": float(wheel_l[1]),
        "wheel_z_l": float(wheel_l[2]),
        "wheel_y_r": float(wheel_r[1]),
        "wheel_z_r": float(wheel_r[2]),
    })


# --------------------------------------------------------------------------- #
# 指标
# --------------------------------------------------------------------------- #
def _window_metrics(rows: list[dict], t0: float, t1: float) -> dict:
    seg = [r for r in rows if t0 <= r["t"] <= t1]
    if not seg:
        return {}
    roll = np.array([r["roll_deg"] for r in seg])
    pitch = np.array([r["pitch_deg"] for r in seg])
    v = np.array([r["v_fwd"] for r in seg])
    cmd = np.array([r["cmd_v"] for r in seg])
    return {
        "roll_p2p_deg": float(np.ptp(roll)),
        "max_abs_roll_deg": float(np.max(np.abs(roll))),
        "max_abs_pitch_deg": float(np.max(np.abs(pitch))),
        "v_mean": float(np.mean(v)),
        "v_min": float(np.min(v)),
        "speed_rms_error": float(np.sqrt(np.mean((cmd - v) ** 2))),
        "airborne_fraction": float(np.mean([r["airborne"] for r in seg])),
        "leg_torque_peak": float(max(
            max(abs(r["tau_hip_l"]), abs(r["tau_hip_r"]),
                abs(r["tau_knee_l"]), abs(r["tau_knee_r"])) for r in seg
        )),
    }


def compute_metrics(scn: Scenario, rows: list[dict], triggers: list[tuple[float, float]]) -> dict:
    pitch = np.array([r["pitch_deg"] for r in rows])
    roll = np.array([r["roll_deg"] for r in rows])
    base_z = np.array([r["base_z"] for r in rows])
    com_z = np.array([r["com_z"] for r in rows])
    yaw = np.array([r["yaw_deg"] for r in rows])
    times = np.array([r["t"] for r in rows])
    wheel_tau = np.array([max(abs(r["tau_wheel_l"]), abs(r["tau_wheel_r"])) for r in rows])
    leg_tau = np.array([max(abs(r["tau_hip_l"]), abs(r["tau_hip_r"]),
                            abs(r["tau_knee_l"]), abs(r["tau_knee_r"])) for r in rows])

    metrics: dict = {
        "duration": float(times[-1]),
        "max_abs_pitch_deg": float(np.max(np.abs(pitch))),
        "max_abs_roll_deg": float(np.max(np.abs(roll))),
        "roll_p2p_deg": float(np.ptp(roll)),
        "min_base_z": float(np.min(base_z)),
        "fell": bool(np.max(np.abs(pitch)) > np.degrees(FALL_PITCH_RAD) or np.min(base_z) < FALL_BASE_Z),
        "airborne_fraction": float(np.mean([r["airborne"] for r in rows])),
        "leg_torque_peak": float(np.max(leg_tau)),
        "wheel_torque_peak": float(np.max(wheel_tau)),
        "x_drift_m": float(rows[-1]["base_x"] - rows[0]["base_x"]),
        "y_drift_m": float(rows[-1]["base_y"] - rows[0]["base_y"]),
        "windows": {name: _window_metrics(rows, *win) for name, win in scn.windows.items()},
    }

    # 跳跃相关指标（只有在真的起跳后才有意义）
    for idx, (trigger_t, _amp) in enumerate(triggers):
        after = [r for r in rows if r["t"] >= trigger_t]
        flight = [r for r in after if r["phase"] == JumpPhase.FLIGHT.value]
        if not flight:
            metrics[f"jump{idx}_took_off"] = False
            continue
        takeoff_t = flight[0]["t"]
        before_touch = [r for r in rows if r["t"] <= takeoff_t and r["contact"] > 0]
        last_contact = before_touch[-1] if before_touch else flight[0]
        apex = max(after, key=lambda r: r["com_z"])
        landing = [r for r in after if r["t"] > takeoff_t + 0.3]
        flight_seg = [r for r in after if r["phase"] == JumpPhase.FLIGHT.value]

        def _wheel_z(row: dict) -> float:
            """轮心高度 = 机身原点到轮心的高度差（腿高）反推。"""
            return row["base_z"] - 0.5 * (row["h_left"] + row["h_right"])

        # 行程保持率：起跳后 2 s 的前向位移 / （起跳前速度 × 2 s）。
        # 这是"行驶中跳跃不减速"的核心指标，也是论文里最有说服力的那个数。
        pre = [r for r in rows if trigger_t - 0.5 <= r["t"] <= trigger_t]
        post = [r for r in rows if takeoff_t <= r["t"] <= takeoff_t + 2.0]
        v_before = float(np.mean([r["v_fwd"] for r in pre])) if pre else 0.0
        travel = float(np.trapezoid([r["v_fwd"] for r in post], [r["t"] for r in post])) if len(post) > 1 else 0.0
        retention = travel / (v_before * 2.0) if abs(v_before) > 0.05 else float("nan")

        # 该跳的观测终点：下一次触发之前（多连跳时不能都用"运行结束"，否则各跳读数会互相重叠）
        next_trigger = triggers[idx + 1][0] if idx + 1 < len(triggers) else times[-1]
        yaw_end = float(np.interp(next_trigger, times, yaw))

        metrics.update({
            f"jump{idx}_took_off": True,
            f"jump{idx}_trigger_t": trigger_t,
            f"jump{idx}_takeoff_t": float(takeoff_t),
            f"jump{idx}_ballistic_mm": float((apex["com_z"] - last_contact["com_z"]) * 1000.0),
            f"jump{idx}_apex_com_z": float(apex["com_z"]),
            f"jump{idx}_flight_pitch_peak_deg": float(max(abs(r["pitch_deg"]) for r in flight_seg)),
            f"jump{idx}_landing_pitch_peak_deg": float(
                max((abs(r["pitch_deg"]) for r in landing), default=0.0)),
            f"jump{idx}_yaw_drift_deg": float(yaw_end - float(
                np.interp(trigger_t, times, yaw))),
            f"jump{idx}_extend_leg_torque_peak": float(max(
                (max(abs(r["tau_hip_l"]), abs(r["tau_hip_r"]),
                     abs(r["tau_knee_l"]), abs(r["tau_knee_r"]))
                 for r in after if r["phase"] == JumpPhase.EXTEND.value), default=0.0)),
            f"jump{idx}_wheel_clearance_mm": float(
                (max(_wheel_z(r) for r in flight_seg) - _wheel_z(last_contact)) * 1000.0),
            f"jump{idx}_v_before": v_before,
            f"jump{idx}_v_min_after_takeoff": float(min(r["v_fwd"] for r in post)) if post else 0.0,
            f"jump{idx}_travel_retention_2s": float(retention),
        })
    return metrics


# --------------------------------------------------------------------------- #
# 运行
# --------------------------------------------------------------------------- #
def run_scenario(
    scn: Scenario,
    *,
    out_dir: Path,
    tag: str,
    sample_every: int,
    viewer: bool,
    quiet: bool,
    extra_overrides: list[str],
) -> dict:
    params = copy.deepcopy(STAND_PARAMS)
    for key, value in scn.overrides.items():
        apply_override(params, key, value)
    for item in extra_overrides:
        key, _, raw = item.partition("=")
        if not _:
            raise ValueError(f"--set expects key=value, got {item!r}")
        apply_override(params, key.strip(), _coerce(raw))

    # XML 缓存放在 paper/.cache 下（不放 data/ 里，避免和论文数据混在一起）
    cache_dir = _ROBOT_SIM / "paper" / ".cache" / _terrain_key(scn)
    model, data = build_controlled_model(
        Path(URDF_RELATIVE_PATH),
        cache_dir=cache_dir,
        terrain=scn.terrain,
        **scn.terrain_kwargs,
    )
    addresses = model_addresses(model)
    phase_machine = JumpPhaseMachine(MANUAL_JUMP_PHASE_PARAMS)
    controller = CombinedController(params, phase_machine=phase_machine)

    dt = float(model.opt.timestep)
    total_steps = int(round(scn.duration / dt))
    pending_times = list(scn.jump_times)
    pending_y = list(scn.jump_at_y)
    triggers: list[tuple[float, float]] = []
    rows: list[dict] = []

    context = None
    if viewer:
        # mujoco.viewer 是惰性子模块，必须显式导入才会挂到 mujoco 上。
        # 注意：这里必须用 `from mujoco import viewer`，不能用 `import mujoco.viewer`——
        # 后者会让 `mujoco` 变成本函数的局部名，遮蔽模块级导入，
        # 导致本函数前面的 mujoco.mj_step 等调用报 UnboundLocalError。
        from mujoco import viewer as mj_viewer
        context = mj_viewer.launch_passive(model, data)

    wall_start = time.perf_counter()
    last_control = np.zeros(model.nu)
    executed_steps = 0
    try:
        for step in range(total_steps):
            if context is not None and not context.is_running():
                break
            executed_steps += 1
            t = float(data.time)
            state = extract_sim_state(model, data)
            params.target_velocity = scn.velocity(t)
            params.target_yaw_rate = scn.yaw_rate(t)
            params.vmc.nominal_height = scn.height(t)

            if pending_times and t >= pending_times[0] - 0.5 * dt:
                trigger_t = pending_times.pop(0)
                if phase_machine.phase != JumpPhase.STAND:
                    if not quiet:
                        print(f"    [跳过] t={trigger_t:.2f}s 的跳跃触发被忽略：当前相位为 "
                              f"{phase_machine.phase.value}（需回到 STAND）")
                else:
                    _start_jump(phase_machine, params, scn)
                    triggers.append((trigger_t, scn.jump_amplitude))
            if pending_y and float(state.base_position[1]) >= pending_y[0]:
                trigger_y = pending_y.pop(0)
                if phase_machine.phase != JumpPhase.STAND:
                    if not quiet:
                        print(f"    [跳过] 到 y={trigger_y:.2f} m 时相位为 "
                              f"{phase_machine.phase.value}，本次跳跃忽略")
                else:
                    _start_jump(phase_machine, params, scn)
                    triggers.append((t, scn.jump_amplitude))
                    if not quiet:
                        print(f"    [触发] 到 y={trigger_y:.2f} m（实际 {state.base_position[1]:.2f}）起跳")

            control = np.asarray(controller(model, data, state), dtype=float)
            last_control = control
            data.ctrl[: model.nu] = control
            mujoco.mj_step(model, data)
            # 采样放在步进之后，与 test_jump.py / test_jump_moving.py 的口径一致
            # （接触数、质心高度都取"这一步走完"的值），否则弹道会差一个采样周期。
            if step % sample_every == 0 or step == total_steps - 1:
                state_after = extract_sim_state(model, data)
                rows.append(sample_row(
                    scn, model, data, state_after, control, addresses, phase_machine.phase,
                ))
            if context is not None and step % 5 == 0:
                context.sync()
                time.sleep(0.001)
    finally:
        if context is not None:
            context.close()

    # 末步补一条，保证指标覆盖到结束
    if rows and rows[-1]["t"] < float(data.time):
        state = extract_sim_state(model, data)
        rows.append(sample_row(scn, model, data, state, last_control, addresses, phase_machine.phase))

    wall = time.perf_counter() - wall_start
    metrics = compute_metrics(scn, rows, triggers)
    metrics.update({
        "name": scn.name,
        "group": scn.group,
        "desc": scn.desc,
        "terrain": scn.terrain,
        "terrain_kwargs": {k: str(v) for k, v in scn.terrain_kwargs.items()},
        "overrides": {**scn.overrides, **dict(i.split("=", 1) for i in extra_overrides)},
        "sim_timestep": dt,
        "sample_every_steps": sample_every,
        "wall_time_s": round(wall, 2),
        "realtime_factor": round(float(data.time) / wall, 2) if wall > 0 else None,
        "truncated": executed_steps < total_steps,
    })
    _write_outputs(scn, rows, metrics, out_dir, tag)

    if not quiet:
        note = f"（提前中断，仅跑了 {executed_steps}/{total_steps} 步）" if metrics["truncated"] else "完成"
        print(
            f"  {scn.name:<22s} {note}  "
            f"|pitch|max={metrics['max_abs_pitch_deg']:6.2f}°  "
            f"roll_p2p={metrics['roll_p2p_deg']:6.2f}°  "
            f"摔={'是' if metrics['fell'] else '否'}  "
            f"实时倍率={metrics['realtime_factor']}x"
        )
    return metrics


def _terrain_key(scn: Scenario) -> str:
    parts = [str(scn.terrain)] + [f"{k}-{v}" for k, v in sorted(scn.terrain_kwargs.items())]
    return "_".join(parts).replace("/", "-").replace(" ", "")


def _start_jump(phase_machine: JumpPhaseMachine, params, scn: Scenario) -> None:
    # JumpTrajectoryParams 是 frozen dataclass，只能整体替换（不能用 setattr）。
    traj_params = MANUAL_JUMP_TRAJECTORY_PARAMS
    if scn.jump_overrides:
        traj_params = replace(traj_params, **scn.jump_overrides)
    trajectory = JumpTrajectory(
        traj_params,
        h_start=float(params.vmc.nominal_height),
        cmd_jump_amplitude=float(scn.jump_amplitude),
    )
    phase_machine.start_jump(trajectory)


def _write_outputs(scn: Scenario, rows: list[dict], metrics: dict, out_dir: Path, tag: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    suffix = f"_{tag}" if tag else ""
    csv_path = out_dir / f"{scn.name}{suffix}.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    (out_dir / f"{scn.name}{suffix}.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    index_path = out_dir / "index.json"
    index = json.loads(index_path.read_text(encoding="utf-8")) if index_path.exists() else {}
    index[f"{scn.name}{suffix}"] = {
        "desc": scn.desc,
        "group": scn.group,
        "csv": csv_path.name,
        "samples": len(rows),
        "fell": metrics["fell"],
        "max_abs_pitch_deg": metrics["max_abs_pitch_deg"],
        "roll_p2p_deg": metrics["roll_p2p_deg"],
        "overrides": metrics["overrides"],
    }
    index_path.write_text(json.dumps(index, ensure_ascii=False, indent=2), encoding="utf-8")


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="按名字跑论文仿真场景（输出 CSV + JSON）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--list", action="store_true", help="列出所有场景后退出")
    parser.add_argument("--case", action="append", default=[], help="场景名（可重复）")
    parser.add_argument("--group", action="append", default=[], help="分组名（可重复）")
    parser.add_argument("--all", action="store_true", help="跑全部场景")
    parser.add_argument("--out", default="paper/data", help="输出目录（默认 paper/data）")
    parser.add_argument("--tag", default="", help="输出文件名后缀，例如 ff_off")
    parser.add_argument("--set", action="append", default=[], help="临时参数覆盖 key=value（可重复）")
    parser.add_argument("--sample-every", type=int, default=1, help="每 N 步记录一行（默认每步）")
    parser.add_argument("--viewer", action="store_true", help="打开可视化窗口")
    parser.add_argument("--quiet", action="store_true", help="只输出结果表")
    return parser.parse_args(argv)


def print_scenarios() -> None:
    from paper.scenarios import SCENARIOS

    print("=" * 96)
    print(f"共 {len(SCENARIOS)} 个场景，分组：{', '.join(groups())}")
    print("=" * 96)
    for group in groups():
        print(f"\n[{group}]")
        for scn in SCENARIOS.values():
            if scn.group != group:
                continue
            terrain = scn.terrain or "flat"
            jump = ""
            if scn.jump_times:
                jump = f"  jump@t={scn.jump_times}"
            elif scn.jump_at_y:
                jump = f"  jump@y={scn.jump_at_y}"
            print(f"  {scn.name:<24s} {scn.duration:5.1f}s  {terrain:<12s} {scn.desc}{jump}")
    print()


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.list:
        print_scenarios()
        return 0

    picked = select(
        names=args.case or None,
        group_names=args.group or None,
    ) if (args.case or args.group or args.all) else []
    if not picked:
        print("没有指定场景。用 --case <名字> / --group <分组> / --all，或 --list 查看。")
        return 1

    out_dir = Path(args.out)
    print("=" * 96)
    print(f"运行 {len(picked)} 个场景 → {out_dir.resolve()}")
    if args.set:
        print(f"临时参数覆盖：{args.set}")
    print("=" * 96)

    failures: list[str] = []
    for scn in picked:
        try:
            metrics = run_scenario(
                scn,
                out_dir=out_dir,
                tag=args.tag,
                sample_every=max(1, args.sample_every),
                viewer=args.viewer,
                quiet=args.quiet,
                extra_overrides=args.set,
            )
        except Exception as exc:  # noqa: BLE001 - 单场景失败不影响其余场景
            print(f"  {scn.name:<22s} 失败：{type(exc).__name__}: {exc}")
            failures.append(scn.name)
            continue
        if metrics["fell"]:
            failures.append(f"{scn.name}（摔倒）")

    print("-" * 96)
    if failures:
        print("需要关注的场景：")
        for item in failures:
            print(f"  - {item}")
    else:
        print("全部场景跑完，无摔倒。")

    if args.viewer:
        # 规避 Wayland 下关闭 GLFW 窗口时的段错误
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
