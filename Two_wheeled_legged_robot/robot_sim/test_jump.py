"""跳跃验收：原地垂直起跳（阶段 6 第一步）。

用法（务必用项目虚拟环境）：
    .venv/bin/python test_jump.py            # 幅度 0.4 / 0.7 / 1.0 三档
    .venv/bin/python test_jump.py 1.0        # 指定幅度
    .venv/bin/python test_jump.py --viewer

指标定义（容易搞错，这里写清楚）：
  - 跳跃高度用**质心**，不是机身原点。腾空时伸腿会把机身推高但质心不动，
    用机身原点会把"伸腿"当成"跳"。
  - "弹道升高" = 质心最高点 − 离地瞬间质心高度，这是真正的跳起来多高。
  - "质心总升" 还包含蹬地阶段被腿顶起来的部分，会比弹道高大得多。
  - EXTEND 期间 `h` 是**腿高**变化率，质心只跟约 2/3，所以实测弹道 ≈
    轨迹 `air_height_max` × 0.4。

验收门槛：
  - 必须真的出现 FLIGHT 相位、且弹道升高 > 20 mm；
  - 不摔（|pitch| 或 |roll| < 1.0 rad，机身不掉高）；
  - EXTEND 腿力矩峰值 < 执行器上限 40 N·m（留出未饱和余量，结论才可迁移）；
  - 落地后 1 s 内 |pitch|max < 25°。
"""

from __future__ import annotations

import argparse
import copy
import os
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController  # noqa: E402
from src.controllers.default_params import STAND_PARAMS  # noqa: E402
from src.controllers.jump_trajectory import JumpTrajectory  # noqa: E402
from src.controllers.phase import JumpPhaseMachine  # noqa: E402
from src.launch_mujoco import MANUAL_JUMP_PHASE_PARAMS, MANUAL_JUMP_TRAJECTORY_PARAMS  # noqa: E402
from src.mjcf_builder import prepare_controlled_mujoco_xml  # noqa: E402
from src.state import extract_sim_state, model_addresses  # noqa: E402

LEG_JOINTS = ("link_002_joint", "link_003_joint", "link_005_joint", "link_006_joint")
TRIGGER_TIME = 1.0
TOTAL_TIME = 3.0

MIN_BALLISTIC = 0.020          # m
MAX_EXTEND_TORQUE = 60.0       # N·m（执行器上限，饱和即说明结论不可迁移）
FALL_PITCH = 1.0               # rad
POST_LAND_PITCH_LIMIT = 25.0   # deg


def build():
    xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id == -1:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)
    return model, data


def com_height(model, data) -> float:
    total = float(np.sum(model.body_mass))
    return float(np.sum(model.body_mass[:, None] * data.xipos, axis=0)[2] / total)


def run_once(amplitude: float, viewer: bool = False) -> dict:
    model, data = build()
    addresses = model_addresses(model)
    phase_machine = JumpPhaseMachine(MANUAL_JUMP_PHASE_PARAMS)
    params = copy.deepcopy(STAND_PARAMS)
    controller = CombinedController(params, phase_machine=phase_machine)
    trajectory = JumpTrajectory(
        MANUAL_JUMP_TRAJECTORY_PARAMS,
        h_start=float(params.vmc.nominal_height),
        cmd_jump_amplitude=amplitude,
    )
    leg_actuators = [addresses.actuators[j] for j in LEG_JOINTS]

    samples: list[tuple] = []
    sim_dt = float(model.opt.timestep)
    total_steps = int(TOTAL_TIME / sim_dt)
    context = mujoco.viewer.launch_passive(model, data) if viewer else None
    try:
        step = 0
        while step < total_steps:
            if context is not None and not context.is_running():
                break
            t = float(data.time)
            if abs(t - TRIGGER_TIME) < 0.5 * sim_dt:
                phase_machine.start_jump(trajectory)
            state = extract_sim_state(model, data)
            params.target_velocity = 0.0
            params.target_yaw_rate = 0.0
            control = controller(model, data, state)
            data.ctrl[: model.nu] = control
            mujoco.mj_step(model, data)
            state = extract_sim_state(model, data)
            samples.append((
                t, phase_machine.phase.value, com_height(model, data), int(state.contact_count),
                float(state.pitch), float(state.roll), float(state.base_position[2]),
                max(abs(float(control[a])) for a in leg_actuators),
            ))
            step += 1
            if context is not None and step % 5 == 0:
                context.sync()
                time.sleep(0.001)
    finally:
        if context is not None:
            context.close()

    flight_times = [s[0] for s in samples if s[1] == "flight"]
    result: dict = {"amplitude": amplitude, "flight": bool(flight_times)}
    if not flight_times:
        result.update(phase_end=samples[-1][1], fell=True)
        return result

    takeoff_time = min(flight_times)
    # 离地点 = 时间上不晚于首个 flight 样本的最后一个"真接触"样本。
    # 不能只按 t<=takeoff_time 取：EXTEND→FLIGHT 需 5ms 持续离地确认，首个
    # flight 样本其实已离地几十毫米爬升，把它当接触点会把弹道低估 ~6mm。
    last_contact = max(
        (s for s in samples if s[3] > 0 and s[0] <= takeoff_time),
        key=lambda s: s[0],
    )
    apex = max(s[2] for s in samples)
    extend_torque = max(
        (s[7] for s in samples if s[1] == "extend"), default=0.0
    )
    after = [s for s in samples if s[0] > takeoff_time]
    landed = [s for s in samples if s[0] > takeoff_time + 0.5]
    result.update(
        ballistic=apex - last_contact[2],
        total_rise=apex - samples[0][2],
        z_takeoff=last_contact[2],
        apex=apex,
        extend_torque=extend_torque,
        peak_torque=max(s[7] for s in samples),
        fell=any(abs(s[4]) > FALL_PITCH or abs(s[5]) > FALL_PITCH or s[6] < 0.25 for s in after),
        post_land_pitch=float(np.degrees(max((abs(s[4]) for s in landed), default=0.0))),
        phases=sorted({s[1] for s in samples}),
        samples=samples,
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="跳跃验收")
    parser.add_argument("amplitudes", nargs="*", type=float, default=None)
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    amplitudes = args.amplitudes or [0.4, 0.7, 1.0]

    print("=" * 74)
    print("跳跃验收（原地垂直起跳）")
    print("=" * 74)
    print(f"{'幅度':>5} {'起跳':>5} {'弹道升高':>9} {'质心总升':>9} {'离地v':>8} "
          f"{'EXTEND力矩':>10} {'落地pitch':>10} {'摔':>5}")

    failures: list[str] = []
    for amplitude in amplitudes:
        result = run_once(amplitude, viewer=args.viewer)
        if not result["flight"]:
            print(f"{amplitude:5.2f} {'否':>5}  相位停在 {result.get('phase_end')}")
            failures.append(f"amplitude {amplitude}: never left the ground")
            continue
        print(f"{amplitude:5.2f} {'是':>5} {result['ballistic'] * 1000:8.1f}mm "
              f"{result['total_rise'] * 1000:8.1f}mm {'':>8} "
              f"{result['extend_torque']:9.1f}N {result['post_land_pitch']:9.1f}° "
              f"{str(result['fell']):>5}")
        if result["ballistic"] < MIN_BALLISTIC:
            failures.append(f"amplitude {amplitude}: ballistic {result['ballistic'] * 1000:.1f} mm "
                            f"< {MIN_BALLISTIC * 1000:.0f} mm")
        if result["fell"]:
            failures.append(f"amplitude {amplitude}: fell after landing")
        if result["extend_torque"] >= MAX_EXTEND_TORQUE - 0.05:
            failures.append(f"amplitude {amplitude}: EXTEND leg torque saturated "
                            f"({result['extend_torque']:.1f} N·m)")
        if result["post_land_pitch"] > POST_LAND_PITCH_LIMIT:
            failures.append(f"amplitude {amplitude}: post-landing pitch "
                            f"{result['post_land_pitch']:.1f}° > {POST_LAND_PITCH_LIMIT}°")

    print("-" * 74)
    if args.viewer:
        os._exit(0)
    if failures:
        print("验收：FAIL")
        for item in failures:
            print(f"  - {item}")
        return 1
    print("验收：PASS（出现 FLIGHT、弹道升高达标、腿力矩未饱和、落地不摔）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
