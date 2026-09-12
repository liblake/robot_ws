"""鲁棒性体检：把控制器放到"和标称不一样的世界"里，量余量还剩多少。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_robustness.py            # 全量
    .venv/bin/python test_robustness.py --quick    # 只跑关键几档

为什么做这个：现在所有结论（站立/加减速/转向/过障）都来自**固定标称场景**，
不知道离边界有多远。而且项目里已经发现两处"保护性设置基于过时现象"：
腿的目标角速度前馈被误关、高位包线被误压。这类问题靠零散复现找不全。

体检维度：
  1. 质量        —— 全机 ±30%（CAD 值多半不准；注意控制器读的是同一个 model，
                    所以这测的是"经验增益能否适应"，不是参数失配）
  2. 质心        —— 机身质心前后 ±5 cm（控制器会自动重算平衡角）
  3. 控制回路延迟 —— 10 / 20 / 40 ms（真机头号杀手）
  4. 传感          —— IMU 姿态噪声 + 编码器量化
  5. 地面摩擦      —— ×0.5 / ×2
  6. 增益敏感度    —— 平衡增益 / 速度环增益 ±20%（说明调参面有多尖）
  7. 外力扰动      —— 站立时随机水平推力 + 初始倾角

判据：见 THRESHOLDS，取"标称值的 3~8 倍"作为"还能用"的门槛，只为分档，不是
设计指标。摔了直接 FAIL。

已知局限（体检结果偏乐观的地方，报告里会再强调）：
  - 控制器有若干处直接读 MuJoCo 内部量（`data.xmat/xpos/xipos/qpos/qvel/ncon`），
    本脚本只对"姿态+关节"这两类做了噪声/量化注入，接触数 `ncon` 仍给真值。
    真机上接触检测本身就是一个未实现的模块。
  - 延迟按"控制器输出滞后 N 步"建模，未含电流环/通信抖动。
"""

from __future__ import annotations

import argparse
import copy
import dataclasses
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from typing import Callable

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers import vmc as vm  # noqa: E402
from src.controllers.combined import CombinedController  # noqa: E402
from src.controllers.default_params import STAND_PARAMS  # noqa: E402
from src.mjcf_builder import prepare_controlled_mujoco_xml  # noqa: E402
from src.state import SimState, body_id, extract_sim_state, model_addresses  # noqa: E402


# 每个场景：(指标名, 门槛)。超门槛记 WARN，摔倒记 FAIL。
THRESHOLDS = {
    "stand": {"pitch": 0.15, "roll": 0.06, "y_drift": 0.10},
    "drive": {"track_rms": 0.08, "pitch": 0.25, "roll": 0.06},
    "turn": {"yaw_err": 0.10, "roll": 0.06, "pitch": 0.25},
    "ramp": {"roll_p2p": 5.0, "pitch": 0.35},
}

LEG_JOINTS = [
    (side, joint)
    for side in ("left", "right")
    for joint in (vm.LEG_CLOSED_LOOP[side].hip_joint, vm.LEG_CLOSED_LOOP[side].knee_joint)
]
WHEEL_JOINTS = ("link_007_joint", "link_004_joint")


# --------------------------------------------------------------------------- #
# 世界扰动
# --------------------------------------------------------------------------- #
class Setting:
    def __init__(self, group: str, label: str, *, mass=1.0, base_mass=1.0, com_dy=0.0,
                 friction=1.0, delay_ms=0.0, imu_noise=0.0, encoder=False,
                 balance_gain_scale=1.0, lean_scale=1.0, push=0.0):
        self.group = group
        self.label = label
        self.mass = mass
        self.base_mass = base_mass
        self.com_dy = com_dy
        self.friction = friction
        self.delay_ms = delay_ms
        self.imu_noise = imu_noise
        self.encoder = encoder
        self.balance_gain_scale = balance_gain_scale
        self.lean_scale = lean_scale
        self.push = push


def _world_xml(setting: "Setting", terrain: bool) -> Path:
    """按设置改惯量/质心/摩擦，并写出 XML 再加载。

    不要在运行时改 `model.body_mass/inertia` 再 `mj_setConst`：实测这一步会对
    某些缩放系数（0.85 / 1.15）算出 NaN 的模型（0.70 / 1.00 / 1.30 却正常）。
    直接改 XML 里的 `<inertial>` 让 MuJoCo 重新编译，既干净又是物理上正确的
    "同形状、不同密度"。
    """
    if terrain:
        src = prepare_controlled_mujoco_xml(
            Path("src/robot/robot.urdf"), terrain="single_wheel_trapezoid",
            terrain_side="left", terrain_height=0.04,
        )
    else:
        src = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    tree = ET.parse(src)
    root = tree.getroot()
    if terrain:
        for worldbody in root.findall("worldbody"):
            for geom in list(worldbody.findall("geom")):
                name = geom.get("name") or ""
                if "trapezoid" in name and "ramp_up" not in name and "platform" not in name:
                    worldbody.remove(geom)

    def scale_inertial(body: ET.Element, factor: float, com_dy: float) -> None:
        inertial = body.find("inertial")
        if inertial is None:
            return
        if inertial.get("mass"):
            inertial.set("mass", repr(float(inertial.get("mass")) * factor))
        for key in ("diaginertia", "fullinertia"):
            if inertial.get(key):
                values = [float(v) * factor for v in inertial.get(key).split()]
                inertial.set(key, " ".join(repr(v) for v in values))
        if com_dy:
            pos = [float(v) for v in (inertial.get("pos") or "0 0 0").split()]
            pos[1] += com_dy
            inertial.set("pos", " ".join(repr(v) for v in pos))

    for body in root.iter("body"):
        name = body.get("name") or ""
        if setting.mass != 1.0:
            scale_inertial(body, setting.mass, 0.0)
        if setting.base_mass != 1.0 and name == "base_link":
            scale_inertial(body, setting.base_mass, 0.0)
        if setting.com_dy != 0.0 and name == "base_link":
            scale_inertial(body, 1.0, setting.com_dy)

    if setting.friction != 1.0:
        for geom in root.iter("geom"):
            name = geom.get("name") or ""
            if name == "floor" or "trapezoid" in name:
                parts = [float(v) for v in (geom.get("friction") or "1 0.005 0.0001").split()]
                parts[0] *= setting.friction
                geom.set("friction", " ".join(repr(v) for v in parts))

    out = Path(tempfile.mkdtemp(prefix="robust_")) / "world.xml"
    tree.write(out, encoding="utf-8")
    return out


def build(setting: Setting, terrain: bool):
    model = mujoco.MjModel.from_xml_path(str(_world_xml(setting, terrain)))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)
    return model, data


def make_params(setting: Setting):
    params = copy.deepcopy(STAND_PARAMS)
    if setting.balance_gain_scale != 1.0:
        params.wheel_balance_gain_2d = params.wheel_balance_gain_2d * setting.balance_gain_scale
    if setting.lean_scale != 1.0:
        params.pitch_lean_gain *= setting.lean_scale
    return params


# --------------------------------------------------------------------------- #
# 传感/延迟包装
# --------------------------------------------------------------------------- #
class SensorLoop:
    """把控制器包成"只看得见有噪声/量化的传感量、且输出带延迟"的样子。"""

    def __init__(self, controller, setting: Setting, seed: int = 0):
        self.controller = controller
        self.setting = setting
        self.rng = np.random.default_rng(seed)
        self.delay_steps = int(round(setting.delay_ms / 1000.0 / 0.002))
        self.buffer: deque[np.ndarray] = deque()
        self.enc_q_res = 2.0 * np.pi / 4096.0   # 12 位编码器
        self.enc_v_res = 0.01                    # rad/s

    def __call__(self, model, data, state: SimState) -> np.ndarray:
        addresses = model_addresses(model)
        q_idx = [addresses.joint_qpos[j] for _, j in LEG_JOINTS] + \
                [addresses.joint_qpos[j] for j in WHEEL_JOINTS]
        v_idx = [addresses.joint_qvel[j] for _, j in LEG_JOINTS] + \
                [addresses.joint_qvel[j] for j in WHEEL_JOINTS]
        # 传感误差只应影响"控制器看到的世界"，不能污染真值——整份 qpos/qvel 存盘，
        # 调用完原样恢复（否则姿态噪声会逐步随机游走把仿真带飞）。
        q_saved = np.array(data.qpos, copy=True)
        v_saved = np.array(data.qvel, copy=True)
        touched = False
        if self.setting.encoder:
            # 只改关节编码器对应的 qpos/qvel，不调 mj_forward：VMC 是直接读
            # data.qpos/qvel 的，这样它才看得到量化误差；而 mj_forward 会留下
            # 按扰动后状态解出的接触力/热启动项，把真值也污染掉。
            data.qpos[q_idx] = np.round(data.qpos[q_idx] / self.enc_q_res) * self.enc_q_res
            data.qvel[v_idx] = np.round(data.qvel[v_idx] / self.enc_v_res) * self.enc_v_res
            touched = True
        if self.setting.imu_noise > 0.0:
            # IMU 误差只改控制器"看到"的状态副本，绝不动 data：
            # 早期版本把姿态噪声写进 data 再 mj_forward，实测会让机器人产生
            # 13~15° 的稳态倾角（假象）；只用 state 副本注入时该偏置消失。
            sigma = self.setting.imu_noise
            state = dataclasses.replace(
                state,
                pitch=state.pitch + self.rng.normal(0.0, sigma),
                roll=state.roll + self.rng.normal(0.0, sigma),
                pitch_rate=state.pitch_rate + self.rng.normal(0.0, sigma * 4.0),
                roll_rate=state.roll_rate + self.rng.normal(0.0, sigma * 4.0),
            )

        control = self.controller(model, data, state)

        if touched:
            data.qpos[:] = q_saved
            data.qvel[:] = v_saved

        self.buffer.append(np.asarray(control, dtype=float).copy())
        if len(self.buffer) > self.delay_steps:
            return self.buffer.popleft()
        return self.buffer[0]


# --------------------------------------------------------------------------- #
# 场景
# --------------------------------------------------------------------------- #
def _contact_count(model, data) -> int:
    ground = set()
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if name == "floor" or "trapezoid" in name or "wavy" in name:
            ground.add(name)
    count = 0
    for i in range(data.ncon):
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[i].geom1)
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, data.contact[i].geom2)
        if (g1 in ground) or (g2 in ground):
            count += 1
    return count


def _run(model, data, loop: SensorLoop, t_end: float, cmd_fn, sample_fn):
    samples = []
    fell_at = None
    for _ in range(int(t_end / model.opt.timestep)):
        state = extract_sim_state(model, data)
        velocity, yaw_rate = cmd_fn(float(data.time))
        controller = loop.controller
        controller.params.target_velocity = velocity
        controller.params.target_yaw_rate = yaw_rate
        control = loop(model, data, state)
        data.ctrl[: model.nu] = control
        mujoco.mj_step(model, data)
        state = extract_sim_state(model, data)
        samples.append(sample_fn(float(data.time), state, data, control))
        if fell_at is None and (abs(state.pitch) > 1.0 or float(data.qpos[2]) < 0.25):
            fell_at = float(data.time)
            break
    return samples, fell_at


def scenario_stand(setting: Setting) -> dict:
    model, data = build(setting, terrain=False)
    loop = SensorLoop(CombinedController(make_params(setting)), setting)

    def cmd(_t):
        return 0.0, 0.0

    rng = np.random.default_rng(7)
    bid = body_id(model, "base_link")
    y0 = float(data.qpos[1])

    def sample(t, state, data_, control):
        push = setting.push * np.sum(model.body_mass) * 9.81
        phase = np.sin(2.0 * np.pi * t / 2.0)
        data_.xfrc_applied[bid, 1] = push * phase
        return (t, abs(state.pitch), abs(state.roll), abs(float(data_.qpos[1]) - y0))

    samples, fell = _run(model, data, loop, 10.0, cmd, sample)
    arr = np.array([s[1:] for s in samples])
    warm = arr[len(arr) // 5:]
    return {
        "fell": fell,
        "pitch": float(warm[:, 0].max()),
        "roll": float(warm[:, 1].max()),
        "y_drift": float(warm[:, 2].max()),
    }


def scenario_drive(setting: Setting) -> dict:
    model, data = build(setting, terrain=False)
    loop = SensorLoop(CombinedController(make_params(setting)), setting)
    v_max = 0.5

    def cmd(t):
        if t < 1.0:
            return 0.0, 0.0
        if t < 1.0 + v_max / 1.0:
            return (t - 1.0) * 1.0, 0.0
        return v_max, 0.0

    def sample(t, state, data_, control):
        actual = loop.controller._project_forward_body_velocity(model, data_, state)
        return (t, actual, state.pitch, state.roll)

    samples, fell = _run(model, data, loop, 10.0, cmd, sample)
    arr = np.array([s[1:] for s in samples])
    times = np.array([s[0] for s in samples])
    hold = (times > 3.0) & (times < 9.0)
    return {
        "fell": fell,
        "track_rms": float(np.sqrt(np.mean((arr[hold, 0] - v_max) ** 2))),
        "pitch": float(np.abs(arr[:, 1]).max()),
        "roll": float(np.abs(arr[:, 2]).max()),
    }


def scenario_turn(setting: Setting) -> dict:
    model, data = build(setting, terrain=False)
    loop = SensorLoop(CombinedController(make_params(setting)), setting)
    yaw = 0.5

    def cmd(t):
        return 0.0, (0.0 if t < 1.0 else min(yaw, (t - 1.0) * 1.0))

    def sample(t, state, data_, control):
        return (t, float(state.base_angular_velocity[2]), state.roll, state.pitch)

    samples, fell = _run(model, data, loop, 12.0, cmd, sample)
    arr = np.array([s[1:] for s in samples])
    times = np.array([s[0] for s in samples])
    hold = (times > 8.0) & (times < 11.5)
    return {
        "fell": fell,
        "yaw_err": float(abs(arr[hold, 0].mean() - yaw)),
        "roll": float(np.abs(arr[:, 1]).max()),
        "pitch": float(np.abs(arr[:, 2]).max()),
    }


def scenario_ramp(setting: Setting) -> dict:
    model, data = build(setting, terrain=True)
    loop = SensorLoop(CombinedController(make_params(setting)), setting)
    y0 = float(data.qpos[1])

    def cmd(t):
        if t < 2.0:
            return 0.0, 0.0
        return min(0.3, (t - 2.0) * 0.15), 0.0

    def sample(t, state, data_, control):
        return (t, state.roll, state.pitch, float(data_.qpos[1]) - y0, _contact_count(model, data_))

    samples, fell = _run(model, data, loop, 12.0, cmd, sample)
    arr = np.array([s[1:] for s in samples])
    on_ramp = (arr[:, 2] > 0.15) & (arr[:, 2] < 0.95)
    if on_ramp.sum() < 5:
        return {"fell": fell, "roll_p2p": float("nan"), "pitch": float(np.abs(arr[:, 1]).max())}
    return {
        "fell": fell,
        "roll_p2p": float(np.degrees(np.ptp(arr[on_ramp, 0]))),
        "pitch": float(np.abs(arr[:, 1]).max()),
    }


SCENARIOS: dict[str, Callable[[Setting], dict]] = {
    "stand": scenario_stand,
    "drive": scenario_drive,
    "turn": scenario_turn,
    "ramp": scenario_ramp,
}


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def settings(quick: bool) -> list[Setting]:
    full = [
        Setting("基线", "标称"),
        Setting("质量", "全机 ×0.70", mass=0.70),
        Setting("质量", "全机 ×0.85", mass=0.85),
        Setting("质量", "全机 ×1.15", mass=1.15),
        Setting("质量", "全机 ×1.30", mass=1.30),
        Setting("质量", "机身只 ×0.70", base_mass=0.70),
        Setting("质量", "机身只 ×1.30", base_mass=1.30),
        Setting("质心", "前移 5 cm", com_dy=+0.05),
        Setting("质心", "后移 5 cm", com_dy=-0.05),
        Setting("延迟", "5 ms", delay_ms=5.0),
        Setting("延迟", "10 ms", delay_ms=10.0),
        Setting("延迟", "20 ms", delay_ms=20.0),
        Setting("延迟", "40 ms", delay_ms=40.0),
        Setting("传感", "IMU 0.3°", imu_noise=np.radians(0.3)),
        Setting("传感", "IMU 1.0°", imu_noise=np.radians(1.0)),
        Setting("传感", "编码器 12bit 量化", encoder=True),
        Setting("传感", "IMU 0.3° + 编码器", imu_noise=np.radians(0.3), encoder=True),
        Setting("传感", "IMU 1.0° + 编码器", imu_noise=np.radians(1.0), encoder=True),
        Setting("地面", "摩擦 ×0.5", friction=0.5),
        Setting("地面", "摩擦 ×2", friction=2.0),
        Setting("增益", "平衡增益 ×0.8", balance_gain_scale=0.8),
        Setting("增益", "平衡增益 ×1.2", balance_gain_scale=1.2),
        Setting("增益", "速度环增益 ×0.8", lean_scale=0.8),
        Setting("增益", "速度环增益 ×1.2", lean_scale=1.2),
        Setting("扰动", "推力 ±15% 自重", push=0.15),
        Setting("组合", "质量×1.15 + 20 ms + 传感", mass=1.15, delay_ms=20.0,
                imu_noise=np.radians(0.3), encoder=True),
    ]
    if not quick:
        return full
    keep = {"标称", "全机 ×1.30", "全机 ×0.70", "机身只 ×1.30", "后移 5 cm", "前移 5 cm",
            "10 ms", "20 ms", "40 ms", "IMU 0.3°", "IMU 1.0°", "编码器 12bit 量化", "摩擦 ×0.5",
            "平衡增益 ×1.2", "速度环增益 ×1.2", "推力 ±15% 自重", "质量×1.15 + 20 ms + 传感"}
    return [s for s in full if s.label in keep]


def evaluate(scenario: str, metrics: dict) -> tuple[str, str]:
    if metrics.get("fell") is not None:
        return "FAIL", f"摔倒 @{metrics['fell']:.2f}s"
    worst = []
    for key, limit in THRESHOLDS[scenario].items():
        value = metrics.get(key)
        if value is None or not np.isfinite(value):
            continue
        ratio = value / limit
        if ratio > 1.0:
            worst.append(f"{key}={value:.3f} > {limit}")
    if worst:
        return "WARN", "; ".join(worst)
    return "OK", ""


def main() -> int:
    parser = argparse.ArgumentParser(description="控制器鲁棒性体检")
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--only", default=None, help="只跑某个场景")
    args = parser.parse_args()

    scenario_names = [args.only] if args.only else list(SCENARIOS)
    rows = []
    print("=" * 96)
    print("鲁棒性体检（每个格子：判据 + 最坏指标）")
    print("=" * 96)
    header = f"{'组':<6}{'设置':<26}" + "".join(f"{n:^17}" for n in scenario_names)
    print(header)
    print("-" * 96)
    for setting in settings(args.quick):
        cells = []
        for name in scenario_names:
            try:
                metrics = SCENARIOS[name](setting)
                verdict, detail = evaluate(name, metrics)
                if verdict == "FAIL" and metrics.get("fell") is None:
                    verdict, detail = "FAIL", detail or "指标异常"
            except Exception as exc:  # noqa: BLE001 - 体检脚本要能继续跑完
                metrics = {}
                verdict, detail = "ERROR", f"{type(exc).__name__}: {exc}"
            cells.append(f"{verdict}{'!' if verdict != 'OK' else ' '}")
            rows.append((setting.group, setting.label, name, verdict, detail, metrics))
        print(f"{setting.group:<6}{setting.label:<26}" + "".join(f"{c:^17}" for c in cells))

    print("-" * 96)
    problems = [r for r in rows if r[3] != "OK"]
    if problems:
        print("需要关注的组合：")
        for group, label, name, verdict, detail, _ in problems:
            print(f"  [{verdict}] {group} / {label} / {name}: {detail}")
    else:
        print("全部 OK：所测扰动范围内没有超门槛项。")

    print("\n各维度最坏值（相对门槛的倍率，>1 表示超门槛）：")
    for name in scenario_names:
        best = None
        for group, label, scen, verdict, _detail, metrics in rows:
            if scen != name:
                continue
            for key, limit in THRESHOLDS[name].items():
                value = metrics.get(key)
                if value is None or not np.isfinite(value):
                    continue
                ratio = value / limit
                if best is None or ratio > best[0]:
                    best = (ratio, key, value, limit, f"{group}/{label}")
        if best:
            print(f"  {name:<6} 最坏 {best[1]:<10} = {best[2]:.4f} (门槛 {best[3]}) "
                  f"= {best[0]:.2f}×  @ {best[4]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
