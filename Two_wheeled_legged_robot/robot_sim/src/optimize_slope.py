"""坡道专项参数优化器（2026-09-11）。

移植上游 wheel_legged_robot_sim 的调参流程（Optuna + 场景打分 + warm start，
见上游 src/optimize.py），但做三处针对本项目的适配：

1. 场景换成"上坡 → 平台上停车 → 驻车"的坡道场景，直接打目标是坡上余量；
2. 搜索空间换成本项目实际在用的参数（2 状态轮子增益、停车外环、VMC 找平），
   不再搜索 5D LQR 的 Q/R——STAND 走的是 wheel_balance_gain_2d 覆盖通道；
3. 前向轴用 +Y（本机沿 Y 行驶；上游 rollout.py 用的是 [0]，直接套会算错）。

用法：
    .venv/bin/python -m src.optimize_slope --trials 60 --workers 4
    .venv/bin/python -m src.optimize_slope --replay best_slope_params.json
"""
from __future__ import annotations

import argparse
import copy
import json
import tempfile
from dataclasses import dataclass
from math import atan2, cos, sin
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import optuna

from src.controllers.combined import CombinedController, CombinedParams
from src.controllers.default_params import STAND_PARAMS
from src.mjcf_builder import _terrain_box, prepare_controlled_mujoco_xml
from src.state import extract_sim_state

# 单轮梯形坡几何：与 test_slope_v2.py 保持一致（左轮车道 x≈0.01）
TERRAIN = dict(height=0.02, ramp_len=0.20, plat_len=0.50, y_start=0.25, width=0.16, x_lane=0.01)

# 全宽对称坡：坡面覆盖左右两轮（x≈0.01/0.39），平台加长到 1.5m，
# 避免停车后的滑行直接掉下平台尽头（那是场景伪影，不是坡上失稳）。
SYMMETRIC = dict(height=0.04, ramp_len=0.50, plat_len=1.50, y_start=0.40,
                 width=0.60, x_center=0.20)

# 场景：站 1s → 平滑加速到巡航速度上坡 → 到指定位置发停车指令 → 驻车。
# 用位置触发（而非固定时刻），因为本机指令跟随有明显滞后，按时序停车会停不到坡上。
@dataclass(frozen=True)
class ScenarioConfig:
    terrain: str = "symmetric"  # symmetric=全宽对称坡（可调参）；trapezoid=项目单轮梯形坡
    height: float | None = None  # 坡高 (m)，None 用该地形的默认值
    stop_y: float = 0.65        # 到达该位置（m）即发停车指令
    cruise: float = 0.20        # 巡航速度 (m/s)，与 test_slope_v2 一致
    accel: float = 0.05         # 指令加速率 (m/s²)，与 test_slope_v2 一致（平滑无阶跃）
    hold_time: float = 6.0      # 停车后驻车时长 (s)
    max_time: float = 30.0      # 场景总时长上限 (s)
    drive_through: bool = False  # True=不停车全程巡航（"通过性"场景，如单轮梯形坡）

_XML_CACHE: dict[float, str] = {}


def build_terrain_xml(height: float | None = None) -> str:
    """构造单轮梯形坡 XML（同 test_slope_v2 的几何）。"""
    h = float(TERRAIN["height"] if height is None else height)
    if h in _XML_CACHE:
        return _XML_CACHE[h]
    xml_path = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    tree = ET.parse(xml_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    ramp_len = float(TERRAIN["ramp_len"])
    plat_len = float(TERRAIN["plat_len"])
    y_start = float(TERRAIN["y_start"])
    width = float(TERRAIN["width"])
    x_lane = float(TERRAIN["x_lane"])
    span = (ramp_len * ramp_len + h * h) ** 0.5
    ang = atan2(h, ramp_len)
    thick = h / cos(ang) + 0.005
    _terrain_box(
        worldbody,
        name="slope_up",
        size=(0.5 * width, 0.5 * span, 0.5 * thick),
        pos=(x_lane, y_start + 0.5 * ramp_len + 0.5 * thick * sin(ang),
             0.5 * h - 0.5 * thick * cos(ang)),
        euler=(ang, 0.0, 0.0),
    )
    _terrain_box(
        worldbody,
        name="slope_plat",
        size=(0.5 * width, 0.5 * plat_len, 0.5 * h),
        pos=(x_lane, y_start + ramp_len + 0.5 * plat_len, 0.5 * h),
    )
    out = Path(tempfile.mkdtemp()) / "optimize_slope.xml"
    tree.write(out, encoding="utf-8")
    _XML_CACHE[h] = str(out)
    return str(out)


_SYM_CACHE: dict[float, str] = {}


def build_symmetric_slope_xml(height: float | None = None) -> str:
    """构造全宽对称坡 XML（两轮同时上坡，平台 1.5m）。"""
    h = float(SYMMETRIC["height"] if height is None else height)
    if h in _SYM_CACHE:
        return _SYM_CACHE[h]
    xml_path = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    tree = ET.parse(xml_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    ramp_len = float(SYMMETRIC["ramp_len"])
    plat_len = float(SYMMETRIC["plat_len"])
    y_start = float(SYMMETRIC["y_start"])
    width = float(SYMMETRIC["width"])
    x_center = float(SYMMETRIC["x_center"])
    span = (ramp_len * ramp_len + h * h) ** 0.5
    ang = atan2(h, ramp_len)
    thick = h / cos(ang) + 0.005
    _terrain_box(
        worldbody,
        name="sym_ramp",
        size=(0.5 * width, 0.5 * span, 0.5 * thick),
        pos=(x_center, y_start + 0.5 * ramp_len + 0.5 * thick * sin(ang),
             0.5 * h - 0.5 * thick * cos(ang)),
        euler=(ang, 0.0, 0.0),
    )
    _terrain_box(
        worldbody,
        name="sym_plat",
        size=(0.5 * width, 0.5 * plat_len, 0.5 * h),
        pos=(x_center, y_start + ramp_len + 0.5 * plat_len, 0.5 * h),
    )
    out = Path(tempfile.mkdtemp()) / "optimize_slope_sym.xml"
    tree.write(out, encoding="utf-8")
    _SYM_CACHE[h] = str(out)
    return str(out)


@dataclass(frozen=True)
class SlopeMetrics:
    fell: bool
    forward_distance: float      # 沿 +Y 的行驶距离 (m)
    max_abs_pitch: float
    max_abs_roll: float
    offground_fraction: float    # 单轮离地采样占比
    final_speed: float           # 末速 (m/s)
    hold_drift: float            # 驻车稳定后最大位移 (m)
    stop_y: float                # 指令回零时刻的位置 (m)
    stop_time: float             # 指令回零时刻 (s)
    reversed: bool               # 驻车期间是否朝反方向移动超过 30mm


def run_slope(params, config: ScenarioConfig = ScenarioConfig()) -> SlopeMetrics:
    """跑一次坡道场景并返回指标。"""
    if config.terrain == "symmetric":
        xml_path = build_symmetric_slope_xml(config.height)
    elif config.terrain == "trapezoid":
        xml_path = build_terrain_xml(config.height)
    else:
        raise ValueError(f"unsupported terrain: {config.terrain}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id < 0:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)

    controller = CombinedController(copy.deepcopy(params))
    y0 = float(data.qpos[1])
    fell = False
    max_pitch = 0.0
    max_roll = 0.0
    offground = 0
    samples = 0
    hold_y: list[float] = []
    stop_y: float | None = None
    stop_time: float | None = None
    final_speed = 0.0
    contact_bodies = ("link_007_collision_proxy", "link_004_collision_proxy")
    total_steps = max(1, int(np.ceil(config.max_time / model.opt.timestep)))

    for _ in range(total_steps):
        state = extract_sim_state(model, data)
        t = float(data.time)
        # 注意：控制器在构造时深拷贝参数，指令必须写到 controller.params 上
        if t < 1.0:
            command = 0.0
        elif config.drive_through or stop_time is None:
            command = min(config.cruise, config.accel * (t - 1.0))
        else:
            command = 0.0
        controller.params.target_velocity = command
        control = controller(model, data, state)
        data.ctrl[: model.nu] = control
        mujoco.mj_step(model, data)

        state = extract_sim_state(model, data)
        y = float(state.base_position[1]) - y0
        max_pitch = max(max_pitch, abs(float(state.pitch)))
        max_roll = max(max_roll, abs(float(state.roll)))
        if abs(float(state.pitch)) > 0.8 or float(state.base_position[2]) < 0.30:
            fell = True
            break
        if not config.drive_through and stop_time is None and y >= config.stop_y:
            stop_y, stop_time = y, t
        if t >= 2.0:
            # 统计左右轮是否同时接地（单轮坡上离地样本是余量的直接指标）
            left_contact = right_contact = False
            for c in range(data.ncon):
                names = {
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[c].geom1),
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[c].geom2),
                }
                if contact_bodies[0] in names:
                    left_contact = True
                if contact_bodies[1] in names:
                    right_contact = True
            samples += 1
            if not (left_contact and right_contact):
                offground += 1
        if stop_time is not None and t >= stop_time + 0.5:
            # 驻车窗口跳过 0.5s 收敛段，只统计真正的驻车保持质量
            hold_y.append(y)
        if stop_time is not None and t >= stop_time + config.hold_time:
            break
        final_speed = float(controller._project_forward_body_velocity(model, data, state))

    forward_distance = float(data.qpos[1]) - y0
    hold_drift = 0.0
    reversed_flag = False
    if hold_y:
        hold_drift = max(abs(v - hold_y[0]) for v in hold_y)
        reversed_flag = (max(hold_y) - min(hold_y)) > 0.03 and (min(hold_y) < hold_y[0] - 0.03)
    return SlopeMetrics(
        fell=fell,
        forward_distance=forward_distance,
        max_abs_pitch=max_pitch,
        max_abs_roll=max_roll,
        offground_fraction=(offground / samples) if samples else 0.0,
        final_speed=final_speed,
        hold_drift=hold_drift,
        # drive_through 场景无停车点，用前进距离占位以复用同一套评分
        stop_y=(
            stop_y if stop_y is not None
            else (forward_distance if config.drive_through else 0.0)
        ),
        stop_time=stop_time if stop_time is not None else float(data.time),
        reversed=reversed_flag,
    )


# 判定"确实上了坡"的最小位置（对称坡坡面 0.40~0.90）
MIN_PROGRESS = 0.50

# 调参用工况组：单条曲线容易被过拟合（本机早期实验中，巡航/停点微调会让结论翻转），
# 因此对一组工况取平均分。
EVAL_CONFIGS = (
    ScenarioConfig(cruise=0.15, stop_y=0.60),
    ScenarioConfig(cruise=0.15, stop_y=0.70),
    ScenarioConfig(cruise=0.20, stop_y=0.60),
    ScenarioConfig(cruise=0.20, stop_y=0.70),
    # 项目验收用例的通过性场景（单轮梯形坡 2cm，同 test_slope_v2 曲线）。
    # 不加这条时，调参会把对称坡调好却把单轮坡调翻（实测 |pitch| 19.9°→180°）。
    ScenarioConfig(terrain="trapezoid", height=0.02, cruise=0.20,
                   drive_through=True, max_time=18.0),
)


def score_metrics(metrics: SlopeMetrics) -> float:
    """坡道场景评分（越大越好）：先不翻车、再压姿态与离地、最后要求真的上去了。"""
    score = 0.0
    if metrics.fell:
        return -1000.0
    score -= 200.0 * metrics.max_abs_pitch
    score -= 150.0 * metrics.max_abs_roll
    score -= 100.0 * metrics.offground_fraction
    score -= 300.0 * abs(metrics.final_speed)
    score -= 200.0 * metrics.hold_drift
    if metrics.reversed:
        score -= 50.0          # 坡上驻车溜坡
    if metrics.stop_y >= MIN_PROGRESS:
        score += 50.0
    else:
        score -= 500.0 * (MIN_PROGRESS - metrics.stop_y)
    return score


def sample_params(trial: optuna.Trial, local: bool = False) -> CombinedParams:
    """搜索空间：只用本机 STAND 实际生效、且 headless 可调的参数。

    local=True 时把边界收窄到当前值附近 ±20%（局部精调）。宽边界下大部分采样
    会直接掉出稳定域（实测 60~80 轮一条都没超过基线），需要局部搜索才有信号。
    """
    params = copy.deepcopy(STAND_PARAMS)
    if local:
        bounds = LOCAL_BOUNDS
    else:
        bounds = {
            "wheel_gain_pitch": (-70.0, -25.0),
            "wheel_gain_pitch_rate": (-14.0, -3.0),
            "wheel_vel_balance_gain": (-8.0, 0.0),
            "pitch_lean_gain": (0.05, 0.30),
            "velocity_ki": (0.05, 0.60),
            "position_kp": (1.0, 6.0),
            "position_kd": (0.5, 3.0),
            "wheel_vel_damping": (0.0, 3.0),
            "roll_level_offset_limit": (0.01, 0.05),
        }
    params.wheel_balance_gain_2d = np.array([
        trial.suggest_float("wheel_gain_pitch", *bounds["wheel_gain_pitch"]),
        trial.suggest_float("wheel_gain_pitch_rate", *bounds["wheel_gain_pitch_rate"]),
    ])
    params.wheel_vel_balance_gain = trial.suggest_float(
        "wheel_vel_balance_gain", *bounds["wheel_vel_balance_gain"])
    params.pitch_lean_gain = trial.suggest_float(
        "pitch_lean_gain", *bounds["pitch_lean_gain"])
    params.velocity_ki = trial.suggest_float("velocity_ki", *bounds["velocity_ki"])
    params.position_kp = trial.suggest_float("position_kp", *bounds["position_kp"])
    params.position_kd = trial.suggest_float("position_kd", *bounds["position_kd"])
    params.wheel_vel_damping = trial.suggest_float(
        "wheel_vel_damping", *bounds["wheel_vel_damping"])
    params.vmc.roll_level_offset_limit = trial.suggest_float(
        "roll_level_offset_limit", *bounds["roll_level_offset_limit"])
    return params


# 局部精调边界：当前值 ±20%（基线增益来自试验台标定，只在其邻域内搜索）
LOCAL_BOUNDS = {
    "wheel_gain_pitch": (-54.0, -36.0),
    "wheel_gain_pitch_rate": (-8.4, -5.6),
    "wheel_vel_balance_gain": (-1.5, 0.0),
    "pitch_lean_gain": (0.12, 0.18),
    "velocity_ki": (0.24, 0.36),
    "position_kp": (2.4, 3.6),
    "position_kd": (1.2, 1.8),
    "wheel_vel_damping": (0.0, 0.5),
    "roll_level_offset_limit": (0.028, 0.042),
}


def evaluate(params, configs=EVAL_CONFIGS) -> float:
    """坡道工况组平均分 - 平地停车约束。

    只优化坡道会把"停车顿挫"换回来（实测调后平地轮力矩跳变 0.041→1.447 N.m），
    因此把平地停车作为约束并入目标：跳变超过 0.10 N.m 重罚，反向倒退直接重罚。
    """
    slope_score = float(np.mean([score_metrics(run_slope(params, config)) for config in configs]))
    flat = run_flat_stop(params)
    penalty = 500.0 * max(0.0, flat["wheel_step"] - FLAT_STEP_LIMIT)
    penalty += 200.0 * flat["reverse"] / 0.02
    return slope_score - penalty


def run_flat_stop(params) -> dict:
    """平地 前进0.5→停→后退0.5→停 场景（与 test_stop_response.py 同曲线）。"""
    model = mujoco.MjModel.from_xml_path(
        str(prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf")))
    )
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)
    controller = CombinedController(copy.deepcopy(params))
    rows: list[tuple[float, float, float, np.ndarray, np.ndarray | None]] = []
    previous: np.ndarray | None = None
    for _ in range(max(1, int(np.ceil(18.0 / model.opt.timestep)))):
        state = extract_sim_state(model, data)
        controller.params.target_velocity = _flat_profile(float(data.time))
        control = controller(model, data, state)
        rows.append((
            float(data.time), float(data.qpos[1]),
            controller._project_forward_body_velocity(model, data, state),
            control.copy(), None if previous is None else previous.copy(),
        ))
        data.ctrl[: model.nu] = control
        mujoco.mj_step(model, data)
        previous = control.copy()

    wheel_step = 0.0
    reverse = 0.0
    for start, end in ((7.25, 8.25), (13.25, 18.0)):
        window = [r for r in rows if start <= r[0] <= end]
        pos0 = window[0][1]
        direction = 1.0 if window[0][2] >= 0.0 else -1.0
        reverse = max(reverse, max(-direction * (r[1] - pos0) for r in window))
        edge = [r for r in window if r[4] is not None and r[0] <= start + 0.15]
        for row in edge:
            wheel_step = max(
                wheel_step,
                abs(float(row[3][0] - row[4][0])),
                abs(float(row[3][1] - row[4][1])),
            )
    return {"wheel_step": wheel_step, "reverse": reverse}


def _flat_profile(t: float) -> float:
    t -= 1.0
    if t < 0.0 or t >= 12.25:
        return 0.0
    if t < 1.25:
        return 0.4 * t
    if t < 5.0:
        return 0.5
    if t < 6.25:
        return max(0.0, 0.5 - 0.4 * (t - 5.0))
    if t < 7.25:
        return 0.0
    if t < 8.5:
        return -0.4 * (t - 7.25)
    if t < 11.0:
        return -0.5
    return max(-0.5, -0.5 + 0.4 * (t - 11.0))


FLAT_STEP_LIMIT = 0.10      # N.m，平地停车轮力矩单周期跳变上限（回归阈值 0.25 前留余量）

def objective(trial: optuna.Trial) -> float:
    return evaluate(sample_params(trial))


def baseline_params_dict() -> dict:
    """当前 STAND 参数在搜索空间中的坐标（warm start 用）。"""
    return {
        "wheel_gain_pitch": float(STAND_PARAMS.wheel_balance_gain_2d[0]),
        "wheel_gain_pitch_rate": float(STAND_PARAMS.wheel_balance_gain_2d[1]),
        "wheel_vel_balance_gain": float(STAND_PARAMS.wheel_vel_balance_gain),
        "pitch_lean_gain": float(STAND_PARAMS.pitch_lean_gain),
        "velocity_ki": float(STAND_PARAMS.velocity_ki),
        "position_kp": float(STAND_PARAMS.position_kp),
        "position_kd": float(STAND_PARAMS.position_kd),
        "wheel_vel_damping": float(STAND_PARAMS.wheel_vel_damping),
        "roll_level_offset_limit": float(STAND_PARAMS.vmc.roll_level_offset_limit),
    }


def apply_dict_to_params(params: CombinedParams, values: dict) -> CombinedParams:
    params.wheel_balance_gain_2d = np.array([
        float(values["wheel_gain_pitch"]),
        float(values["wheel_gain_pitch_rate"]),
    ])
    params.wheel_vel_balance_gain = float(values["wheel_vel_balance_gain"])
    params.pitch_lean_gain = float(values["pitch_lean_gain"])
    params.velocity_ki = float(values["velocity_ki"])
    params.position_kp = float(values["position_kp"])
    params.position_kd = float(values["position_kd"])
    params.wheel_vel_damping = float(values["wheel_vel_damping"])
    params.vmc.roll_level_offset_limit = float(values["roll_level_offset_limit"])
    return params


def describe(metrics: SlopeMetrics) -> str:
    return (
        f"fell={metrics.fell} 前进={metrics.forward_distance:+.2f}m 停点y={metrics.stop_y:+.2f}m"
        f"(t={metrics.stop_time:.1f}s) "
        f"|pitch|max={np.degrees(metrics.max_abs_pitch):.1f}deg "
        f"|roll|max={np.degrees(metrics.max_abs_roll):.1f}deg "
        f"离地={metrics.offground_fraction * 100:.0f}% 末速={metrics.final_speed:+.4f} "
        f"驻车漂移={metrics.hold_drift * 1000:.0f}mm"
        f"{' 溜坡' if metrics.reversed else ''}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="坡道专项参数优化（Optuna）")
    parser.add_argument("--trials", type=int, default=60)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, default=Path("best_slope_params.json"))
    parser.add_argument("--replay", type=Path, default=None, help="只评估给定参数文件")
    parser.add_argument("--height", type=float, default=None, help="坡高 (m)，默认 0.04")
    parser.add_argument("--terrain", choices=("symmetric", "trapezoid"), default="symmetric")
    parser.add_argument("--init-from", type=Path, default=None,
                        help="把这个参数文件也作为 warm start 入队（可复用上一轮结果）")
    parser.add_argument("--local", action="store_true",
                        help="局部精调：搜索边界收窄到当前值 ±20%")
    args = parser.parse_args()
    config = ScenarioConfig(terrain=args.terrain, height=args.height)

    if args.replay is not None:
        values = json.loads(Path(args.replay).read_text(encoding="utf-8"))
        params = apply_dict_to_params(copy.deepcopy(STAND_PARAMS), values)
        print("[replay]", describe(run_slope(params, config)))
        return 0

    baseline = copy.deepcopy(STAND_PARAMS)
    print("[baseline]", describe(run_slope(baseline, config)))

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(
        direction="maximize",
        sampler=optuna.samplers.TPESampler(seed=args.seed),
    )
    study.enqueue_trial(baseline_params_dict())  # warm start：第一轮就是当前参数
    if args.init_from is not None and Path(args.init_from).exists():
        study.enqueue_trial(json.loads(Path(args.init_from).read_text(encoding="utf-8")))
        print(f"[warm start] 已入队 {args.init_from}")
    study.optimize(
        lambda trial: evaluate(sample_params(trial, local=args.local)),
        n_trials=args.trials,
        n_jobs=args.workers,
    )

    best = study.best_trial
    print(f"[baseline score] {evaluate(baseline):.3f}  (工况组平均)")
    print(f"[best     score] {best.value:.3f}  (trial #{best.number})")
    print("[best params]")
    for key, value in sorted(best.params.items()):
        print(f"    {key} = {value:.4f}")
    best_params = apply_dict_to_params(copy.deepcopy(STAND_PARAMS), best.params)
    print("[best metrics]", describe(run_slope(best_params, config)))
    for extra in EVAL_CONFIGS:
        print(f"    cruise={extra.cruise:.2f} stop_y={extra.stop_y:.2f} ->", describe(run_slope(best_params, extra)))

    args.out.write_text(
        json.dumps(best.params, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"[saved] {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
