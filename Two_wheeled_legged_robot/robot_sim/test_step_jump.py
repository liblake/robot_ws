"""行驶中跳上台阶验收：前进中不减速起跳，跳上高度 H 的台阶并续跑。

用法：
    .venv/bin/python test_step_jump.py                # 全矩阵：H×速度×触发距离
    .venv/bin/python test_step_jump.py 0.10 0.4       # 指定台阶高与车速

原理：轮子离地高度 = 质心弹道 + 蜷腿收腿量（见归档 18）。上台阶的关键是
**越过台缘瞬间的轮子净空 ≥ H**——由起跳触发位置（距台缘 d_trigger）决定。
位置触发模拟"时机完美的按键"；交互使用时用户按此标定判断提前量。
"""

import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.controllers.jump_trajectory import JumpTrajectory
from src.controllers.phase import JumpPhaseMachine
from src.launch_mujoco import MANUAL_JUMP_PHASE_PARAMS, MANUAL_JUMP_TRAJECTORY_PARAMS
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import extract_sim_state

STEP_X_CENTER = 0.20   # 两轮 x 跨度 -0.01~0.35，箱体全宽覆盖
STEP_HALF_W = 0.30
STEP_PLAT_LEN = 1.60   # 台阶平台长度（m）


def build_step_xml(height: float, y_edge: float) -> str:
    xml_path = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    import xml.etree.ElementTree as ET
    tree = ET.parse(xml_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    ET.SubElement(worldbody, "geom", {
        "name": "step_platform",
        "type": "box",
        "size": f"{STEP_HALF_W} {STEP_PLAT_LEN / 2:.3f} {height / 2:.4f}",
        "pos": f"{STEP_X_CENTER} {y_edge + STEP_PLAT_LEN / 2:.3f} {height / 2:.4f}",
        "contype": "1", "conaffinity": "1", "rgba": "0.45 0.42 0.34 1",
    })
    out = Path(tempfile.mkdtemp()) / "step.xml"
    tree.write(out, encoding="utf-8")
    return str(out)


def run(height: float, speed: float, d_trigger: float, y_edge: float = 0.60,
        verbose: bool = False) -> dict:
    xml = build_step_xml(height, y_edge)
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(model, data)
    pm = JumpPhaseMachine(MANUAL_JUMP_PHASE_PARAMS)
    ctrl = CombinedController(STAND_PARAMS, phase_machine=pm)
    w_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_004")
    w_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_007")

    total = int(8.0 / float(model.opt.timestep))
    y_trigger = y_edge - d_trigger
    fired = False
    fell = False
    face_hit = False
    approach_established = False
    max_clear_at_edge = None
    on_platform_steps = 0
    edge_cross_clear = None
    prev_y = float(data.qpos[1])
    prev_wy = None
    landed_platform = False
    for i in range(total):
        st = extract_sim_state(model, data)
        y = float(st.base_position[1])
        STAND_PARAMS.target_velocity = speed
        STAND_PARAMS.target_yaw_rate = 0.0
        if not fired and y >= y_trigger:
            pm.start_jump(JumpTrajectory(MANUAL_JUMP_TRAJECTORY_PARAMS,
                                         h_start=float(STAND_PARAMS.vmc.nominal_height),
                                         cmd_jump_amplitude=1.0))
            fired = True
        u = ctrl(model, data, st)
        data.ctrl[: model.nu] = u
        mujoco.mj_step(model, data)

        y_new = float(data.qpos[1])
        wz_r = float(data.xpos[w_r, 2])
        wz_l = float(data.xpos[w_l, 2])
        wy_r = float(data.xpos[w_r, 1])
        wy_prev = float(data.xpos[w_r, 1]) if prev_wy is None else prev_wy
        # 越缘净空：以**轮子自身 y** 越过台缘的那一步计（轮子在机身后方 ~7.5cm，
        # 用机身 y 判会早 7.5cm）。轮底净空 = 轮心 z − 台阶高 − 轮半径。
        if edge_cross_clear is None and min(wy_r, wy_prev) <= y_edge <= max(wy_r, wy_prev):
            edge_cross_clear = wz_r - height - 0.07
        # 面撞检测：指令前进但 y 停滞（速度 < 5% 指令且已触发起跳前）
        vy = (y_new - prev_y) / float(model.opt.timestep)
        # 面撞检测仅在起步加速完成（速度曾达 70% 指令）后启用，
        # 否则起步阶段的低速会被误判为撞面停滞。
        if not approach_established and vy >= 0.7 * speed:
            approach_established = True
        if not fired and approach_established and vy < 0.05 * speed:
            face_hit = True
        prev_y = y_new
        prev_wy = wy_r
        # 平台判定：双轮在台阶面上滚（轮心 z ≈ H+0.07±0.02，且 y > y_edge）
        if (y_new > y_edge + 0.05 and abs(wz_r - height - 0.07) < 0.02
                and abs(wz_l - height - 0.07) < 0.02):
            on_platform_steps += 1
        if abs(float(st.pitch)) > 1.0:
            fell = True
        if verbose and i % 50 == 0:
            print(f"  t={float(data.time):5.2f} y={y_new:.3f} wz_r={wz_r:.3f} "
                  f"phase={pm.phase.value} vy={vy:+.2f}")

    on_platform = on_platform_steps > 0.25 * 1.0 / float(model.opt.timestep)  # ≥0.25s
    exit_y = float(data.qpos[1])
    kept_driving = exit_y > y_edge + 0.5
    ok = (not fell) and (not face_hit) and on_platform and kept_driving
    return dict(ok=ok, fell=fell, face_hit=face_hit, on_platform=on_platform,
                kept_driving=kept_driving, edge_clear=edge_cross_clear,
                exit_y=exit_y)


def main() -> int:
    args = [float(a) for a in sys.argv[1:]]
    heights = [args[0]] if len(args) >= 1 else [0.05, 0.10, 0.15, 0.20]
    speeds = [args[1]] if len(args) >= 2 else [0.3, 0.4, 0.5]
    # 触发距离扫描：越缘时刻 ≈ d/v；希望落在高净空窗口（0.10~0.25s）
    print(f"{'台阶H':>6} {'速度':>5} {'触距':>6} {'越缘净空':>8} {'上平台':>5} {'续跑':>5} {'结果':>4}")
    for height in heights:
        for speed in speeds:
            best = None
            for d in (0.05, 0.09, 0.13, 0.17, 0.21, 0.25, 0.29):
                r = run(height, speed, d)
                mark = "✓" if r["ok"] else "✗"
                if r["ok"]:
                    best = (d, r)
                print(f"{height:6.2f} {speed:5.2f} {d:6.2f} "
                      f"{('%.3f' % r['edge_clear']) if r['edge_clear'] is not None else '--':>8} "
                      f"{str(r['on_platform']):>5} {str(r['kept_driving']):>5} {mark:>4}")

            if best:
                print(f"  → H={height:.2f} v={speed:.2f} 可行：触距 {best[0]:.2f} m "
                      f"（净空 {best[1]['edge_clear']:+.3f} m）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
