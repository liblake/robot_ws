"""波浪路（washboard）通过性验收：以梯形速度曲线驶过 y∈[1.05, 2.25] 的随机波浪路。

用法：.venv/bin/python test_wavy_road.py [speed ...]（默认 0.3 0.5 0.8）
指标：摔倒/通过、pitch/roll 峰值、腿电机软限幅饱和占比、单轮离地占比、
      机身垂向加速度 rms（舒适性/地形隔离度）。
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import extract_sim_state, model_addresses

WAVY_Y0, WAVY_Y1 = 1.05, 2.25  # 波浪路范围（与 WavyRoadTerrain 一致）


def run_once(speed: float, accel: float = 0.5, params=None) -> dict:
    xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, kid)
    mujoco.mj_forward(model, data)

    controller = CombinedController(params if params is not None else STAND_PARAMS)
    ad = model_addresses(model)
    leg_acts = [ad.actuators[j] for j in
                ("link_002_joint", "link_003_joint", "link_005_joint", "link_006_joint")]
    limit = float(controller.params.vmc.stand_torque_limit)

    y0 = float(data.qpos[1])
    z_prev = None
    max_pitch = max_roll = 0.0
    fall = False
    sat_steps = wavy_steps = 0
    airborne_steps = 0
    z_acc_sq = 0.0
    total_steps = int(40.0 / float(model.opt.timestep))

    for i in range(total_steps):
        t = float(data.time)
        controller.params.target_velocity = min(speed, speed * t / (speed / accel)) if t < speed / accel else speed
        st = extract_sim_state(model, data)
        u = controller(model, data, st)
        data.ctrl[: model.nu] = u

        # 统计（用 step 前状态）
        in_wavy = WAVY_Y0 - 0.05 < float(st.base_position[1]) < WAVY_Y1 + 0.05
        if in_wavy:
            wavy_steps += 1
            max_pitch = max(max_pitch, abs(st.pitch))
            max_roll = max(max_roll, abs(st.roll))
            if abs(float(st.pitch)) > 0.8 or abs(float(st.roll)) > 0.8:
                fall = True
            if st.contact_count < 2:
                airborne_steps += 1
            if any(abs(u[a]) >= limit - 0.01 for a in leg_acts):
                sat_steps += 1

        mujoco.mj_step(model, data)
        z = float(data.qpos[2])
        if z_prev is not None and in_wavy:
            dt = float(model.opt.timestep)
            z_acc_sq += ((z - 2 * z_prev + z_pprev) / dt**2) ** 2 if z_pprev is not None else 0.0
        z_pprev = z_prev
        z_prev = z

        if float(data.qpos[1]) - y0 > WAVY_Y1 - y0 + 0.6 or fall:
            break

    dist = float(data.qpos[1]) - y0
    return {
        "speed": speed,
        "fall": fall,
        "reached": dist >= WAVY_Y1 - y0 + 0.3,
        "dist": dist,
        "max_pitch": max_pitch,
        "max_roll": max_roll,
        "sat_pct": 100.0 * sat_steps / max(wavy_steps, 1),
        "airborne_pct": 100.0 * airborne_steps / max(wavy_steps, 1),
        "z_acc_rms": float(np.sqrt(z_acc_sq / max(wavy_steps, 1))),
        "wavy_steps": wavy_steps,
    }


def main() -> int:
    speeds = [float(s) for s in sys.argv[1:]] or [0.3, 0.5, 0.8]
    print(f"{'v(m/s)':>6} {'fall':>5} {'pass':>5} {'pitch':>7} {'roll':>7} {'sat%':>6} {'air%':>6} {'z_acc':>7}")
    for v in speeds:
        r = run_once(v)
        print(f"{v:>6.2f} {str(r['fall']):>5} {str(r['reached']):>5} "
              f"{r['max_pitch']:>7.3f} {r['max_roll']:>7.3f} {r['sat_pct']:>6.1f} "
              f"{r['airborne_pct']:>6.1f} {r['z_acc_rms']:>7.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
