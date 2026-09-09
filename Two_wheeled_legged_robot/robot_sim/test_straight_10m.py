"""阶段5验收：10 米直线行驶（0.4 m/s）。

用法：.venv/bin/python test_straight_10m.py
指标：前进距离、横向漂移、航向偏差、pitch/高度稳定性、双轮接地。
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import extract_sim_state


def main() -> int:
    xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id == -1:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)

    controller = CombinedController(STAND_PARAMS)
    x0 = float(data.qpos[0])
    y0 = float(data.qpos[1])
    z0 = float(data.qpos[2])

    def heading() -> float:
        rot = data.xmat[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")].reshape(3, 3)
        fh = rot[:2, 1]
        return float(np.arctan2(fh[1], fh[0]))

    yaw0 = heading()
    wp = 0.0
    bad_contacts = 0
    max_wheel = 0.0
    max_lateral = 0.0
    max_heading_err = 0.0
    z_min = z0
    total_steps = int(35.0 / float(model.opt.timestep))  # 超时保护
    reached = False

    for i in range(total_steps):
        t = float(data.time)
        # 梯形加速到 0.4 m/s，2s 内完成
        if t < 2.0:
            v_cmd = 0.2 * t
        else:
            v_cmd = 0.4
        controller.params.target_velocity = v_cmd
        st = extract_sim_state(model, data)
        u = controller(model, data, st)
        data.ctrl[: model.nu] = u
        mujoco.mj_step(model, data)

        dist = float(st.base_position[1] - y0)
        lateral = abs(float(st.base_position[0] - x0))
        heading_err = abs(heading() - yaw0)
        wp = max(wp, abs(st.pitch))
        max_lateral = max(max_lateral, lateral)
        max_heading_err = max(max_heading_err, heading_err)
        max_wheel = max(max_wheel, abs(st.wheel_velocities["left"]), abs(st.wheel_velocities["right"]))
        z_min = min(z_min, float(st.base_position[2]))
        if st.contact_count < 4:
            bad_contacts += 1
        if dist >= 10.0:
            reached = True
            print(f"到达 10 m：t={t:.2f}s")
            break

    st = extract_sim_state(model, data)
    dist = float(st.base_position[1] - y0)
    print("=" * 60)
    print("10m 直线验收")
    print("=" * 60)
    print(f"前进距离      = {dist:.2f} m  {'PASS' if reached else 'FAIL(超时)'}")
    print(f"横向漂移 max  = {max_lateral * 100:.1f} cm  (目标 < 50cm)")
    print(f"航向偏差 max  = {np.degrees(max_heading_err):.1f} deg  (目标 < 5deg)")
    print(f"|pitch|max    = {wp:.3f} rad  (目标 < 0.15)")
    print(f"机身高度 min  = {z_min:.3f} m (起始 {z0:.3f})")
    print(f"轮速 max      = {max_wheel:.1f} rad/s  (电机峰值 350RPM=36.7rad/s)")
    print(f"掉接触步数    = {bad_contacts}")
    passed = (
        reached
        and max_lateral < 0.5
        and np.degrees(max_heading_err) < 5.0
        and wp < 0.15
        and bad_contacts < 50
    )
    print("验收：" + ("PASS" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
