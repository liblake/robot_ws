#!/usr/bin/env python3
"""双轮腿机器人：机身自平衡 + 前进/后退速度跟踪（LQI）。

把机器人降维成“轮式倒立摆（WIP）”：
    x   = 轮子滚动前进位移  (= 轮半径 * 轮子转角)
    v   = 前进速度          (= 轮半径 * 轮子角速度)
    th  = 机身俯仰角(相对平衡点)
    thd = 俯仰角速度
    u   = 前进力矩指令（左轮 +u，右轮 -u，两轮轴方向相反所以是前进）

线性化模型（平衡点附近用 MuJoCo 有限差分识别）：
    x'' = a_xθ * th + a_xu * u
    th''= a_θθ * th + a_θu * u

为消除速度跟踪稳态误差，加一个速度误差积分项（LQI）：
    状态 = [x, v, th, thd, ∫(v - v_ref) dt]
    控制 = u = -K @ 状态

执行器映射（robot.xml 已修正）：0 hip_left 1 knee_left 2 wheel_left
3 hip_right 4 knee_right 5 wheel_right。前进力矩 u: ctrl[2]=+u, ctrl[5]=-u。
腿部用 PD 锁在标称姿态（这里固定为 0，即直立）。
"""
import os
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mplconfig")
import numpy as np
import mujoco
from scipy.linalg import solve_continuous_are
from scipy.optimize import curve_fit

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

XML = os.path.join(os.path.dirname(__file__), "..", "assets", "robot_urdf", "robot.xml")
WHEEL_R = 0.0697
GROUND_Z = -0.4
LEGS = [(0, 7, 6, 0.0), (1, 8, 7, 0.0), (3, 10, 9, 0.0), (4, 11, 10, 0.0)]


def load():
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    return model, data


def pitch_of(data):
    q = data.qpos[3:7]
    return np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))


def equilibrium(model, data):
    """求平衡点：机身质心位于轮轴正上方，轮底触地。"""
    data.qpos[:] = 0
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    wl = data.body("link_004").xpos.copy()
    wr = data.body("link_007").xpos.copy()
    axle_y = (wl[1] + wr[1]) / 2
    axle_z = (wl[2] + wr[2]) / 2
    mass = float(np.sum(model.body_mass[1:]))
    com = np.zeros(3)
    for b in range(1, model.nbody):
        com += model.body_mass[b] * data.xipos[b]
    com /= mass
    th = np.arctan2(com[1] - axle_y, com[2] - axle_z)
    c, s = np.cos(th), np.sin(th)
    ry = -axle_y * c - (-axle_z) * s
    rz = -axle_y * s + (-axle_z) * c
    origin = np.array([0.0, axle_y + ry, axle_z + rz])
    arz = axle_y * s + axle_z * c
    origin[2] += GROUND_Z - (origin[2] + arz - WHEEL_R)
    quat = np.array([np.cos(th / 2), np.sin(th / 2), 0, 0])
    qpos_eq = np.zeros(model.nq)
    qpos_eq[:3] = origin
    qpos_eq[3:7] = quat
    return qpos_eq, th, origin[1]


def lock_legs(data):
    """腿部 PD 锁在标称姿态。"""
    for a, qp, qv, nom in LEGS:
        data.ctrl[a] = np.clip(-600 * (data.qpos[qp] - nom) - 40 * data.qvel[qv], -200, 200)


def identify(model, data, qpos_eq, th_eq):
    """识别降维 WIP 模型系数，返回 a_xθ, a_θθ, a_xu, a_θu。"""
    def reset_with_pitch(theta0):
        mujoco.mj_resetData(model, data)
        data.qpos[:] = qpos_eq.copy()
        data.qvel[:] = 0
        dpos = np.zeros(model.nv)
        dpos[3] = theta0
        mujoco.mj_integratePos(model, data.qpos, dpos, 1.0)
        mujoco.mj_forward(model, data)

    reset_with_pitch(0.02)
    ts, ths, xs = [], [], []
    while data.time < 0.15:
        lock_legs(data)
        data.ctrl[2] = data.ctrl[5] = 0
        mujoco.mj_step(model, data)
        ts.append(data.time)
        ths.append(pitch_of(data) - th_eq)
        xs.append(WHEEL_R * data.qpos[9])
    ts = np.array(ts)
    ths = np.array(ths)
    xs = np.array(xs)
    w, _ = curve_fit(lambda t, w: ths[0] * np.cosh(w * t), ts, ths, p0=[8.0], maxfev=10000)
    a_thth = abs(w[0]) ** 2
    xdd = np.gradient(np.gradient(xs, ts), ts)
    mask = np.abs(ths) > 0.005
    a_xth = float(np.median(xdd[mask] / ths[mask]))

    reset_with_pitch(0.0)
    thdd, xdd = [], []
    while data.time < 0.03:
        lock_legs(data)
        data.ctrl[2] = 2.0
        data.ctrl[5] = -2.0
        mujoco.mj_step(model, data)
        thdd.append(data.qacc[3])
        xdd.append(WHEEL_R * data.qacc[8])
    a_thu = float(np.mean(thdd) / 2.0)
    a_xu = float(np.mean(xdd) / 2.0)
    return a_xth, a_thth, a_xu, a_thu


def design_lqi(a_xth, a_thth, a_xu, a_thu, Qv, Qth, Qd, Qi, R=1.0):
    """在 [x, v, th, thd] 模型上增广速度积分项，求 LQI 增益 K。"""
    Ac = np.array([
        [0, 1, 0, 0],
        [0, 0, a_xth, 0],
        [0, 0, 0, 1],
        [0, 0, a_thth, 0],
    ])
    Bc = np.array([[0], [a_xu], [0], [a_thu]])
    Aaug = np.zeros((5, 5))
    Aaug[:4, :4] = Ac
    Aaug[4, 1] = 1.0
    Baug = np.zeros((5, 1))
    Baug[:4, 0] = Bc[:, 0]
    Q = np.diag([0.0, Qv, Qth, Qd, Qi])
    P = solve_continuous_are(Aaug, Baug, Q, np.array([[R]]))
    K = np.linalg.solve(np.array([[R]]), Baug.T @ P).ravel()
    return K


def run_demo(model, data, qpos_eq, th_eq, y_eq, K, T=10.0, theta0=0.06):
    """平衡 + 前进/后退速度跟踪演示。"""
    mujoco.mj_resetData(model, data)
    data.qpos[:] = qpos_eq.copy()
    data.qvel[:] = 0
    dpos = np.zeros(model.nv)
    dpos[3] = theta0
    mujoco.mj_integratePos(model, data.qpos, dpos, 1.0)
    mujoco.mj_forward(model, data)

    def vref(t):
        return 0.0 if t < 2.0 else (0.4 if t < 6.0 else -0.25)

    ts, ps, vs, refs = [], [], [], []
    e_v = 0.0
    while data.time < T:
        t = data.time
        vr = vref(t)
        v = WHEEL_R * data.qvel[8]
        e_v += (v - vr) * model.opt.timestep
        x = np.array([WHEEL_R * data.qpos[9], v - vr, pitch_of(data) - th_eq, data.qvel[3], e_v])
        u = -float(K @ x)
        data.ctrl[2] = np.clip(u, -8, 8)
        data.ctrl[5] = np.clip(-u, -8, 8)
        lock_legs(data)
        mujoco.mj_step(model, data)
        ts.append(data.time)
        ps.append(np.degrees(pitch_of(data) - th_eq))
        vs.append(v)
        refs.append(vr)
    return np.array(ts), np.array(ps), np.array(vs), np.array(refs)


def main():
    model, data = load()
    qpos_eq, th_eq, y_eq = equilibrium(model, data)
    print(f"平衡点机身俯仰角: {np.degrees(th_eq):.2f} deg")
    a_xth, a_thth, a_xu, a_thu = identify(model, data, qpos_eq, th_eq)
    print(f"识别模型: x''={a_xth:.2f}*th + {a_xu:.2f}*u, th''={a_thth:.2f}*th + {a_thu:.2f}*u")
    K = design_lqi(a_xth, a_thth, a_xu, a_thu, Qv=20, Qth=500, Qd=50, Qi=500)
    print(f"LQI 增益 K = {np.round(K, 3)}")
    ts, ps, vs, refs = run_demo(model, data, qpos_eq, th_eq, y_eq, K)
    print("演示完成: 平衡 + 前进(0.4m/s) + 后退(-0.25m/s) 速度跟踪")
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].plot(ts, ps, label="body pitch")
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].set_ylabel("pitch error (deg)")
    ax[0].set_title("wheel-legged robot: self-balancing + velocity tracking (LQI)")
    ax[0].grid(True)
    ax[1].plot(ts, vs, label="forward velocity")
    ax[1].plot(ts, refs, "--", label="reference")
    ax[1].set_xlabel("time (s)")
    ax[1].set_ylabel("velocity (m/s)")
    ax[1].legend()
    ax[1].grid(True)
    plt.tight_layout()
    out = os.path.join(os.path.dirname(__file__), "balance_velocity.png")
    plt.savefig(out, dpi=110)
    print(f"图已保存: {out}")


if __name__ == "__main__":
    main()
