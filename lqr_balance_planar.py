"""Planar (pitch-only) LQR balance with 3 states: [pitch, pitch_dot, wheel_vel].

Run:
  python3 lqr_balance_planar.py --xml assets/robot_urdf/robot.xml --viewer
"""

from __future__ import annotations

import argparse
import importlib
import math
from pathlib import Path

import numpy as np

try:
    import mujoco
except ImportError as exc:
    raise SystemExit("This script requires the `mujoco` Python package.") from exc

try:
    from scipy.linalg import solve_discrete_are
    from scipy.optimize import minimize, brentq
except ImportError as exc:
    raise SystemExit("This script requires `scipy`.") from exc


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


def quat_to_pitch(q: np.ndarray) -> float:
    """提取绕 X 轴的角度（俯仰角）"""
    w, x, y, z = q
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


class PlanarBalancer:
    def __init__(
        self,
        xml_path: str | Path,
        hip_angle: float = 0.0,
        knee_angle: float = 0.0,
        base_height: float = 0.0,
        body_height: float | None = None,
        eq_torque_limit: float = 100.0,
        no_u_eq: bool = False,
        feedback_sign: float = 1.0,
        leg_y_offset: float = 0.0,          # 髋关节共同偏置（右+，左-）
    ) -> None:
        self.xml_path = Path(xml_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)

        self.jnt_base = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_joint")
        self.jnt_wheel_r = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "link_004_joint")
        self.jnt_wheel_l = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "link_007_joint")
        self.body_wheel_r = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link_004")
        self.body_wheel_l = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link_007")

        self.act_wheel_r = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "link_004_joint_motor")
        self.act_wheel_l = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "link_007_joint_motor")

        # 腿关节
        self.leg_actuator_ids = []
        self.leg_joint_qpos_adr = []
        for leg_act, leg_joint in [
            ("link_002_joint_motor", "link_002_joint"),
            ("link_003_joint_motor", "link_003_joint"),
            ("link_005_joint_motor", "link_005_joint"),
            ("link_006_joint_motor", "link_006_joint"),
        ]:
            act_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, leg_act)
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, leg_joint)
            self.leg_actuator_ids.append(act_id)
            self.leg_joint_qpos_adr.append(self.model.jnt_qposadr[jnt_id])

        self.base_qpos_adr = self.model.jnt_qposadr[self.jnt_base]
        self.base_dof_adr = self.model.jnt_dofadr[self.jnt_base]
        self.wheel_l_qpos_adr = self.model.jnt_qposadr[self.jnt_wheel_l]
        self.wheel_r_qpos_adr = self.model.jnt_qposadr[self.jnt_wheel_r]
        self.wheel_l_dof_adr = self.model.jnt_dofadr[self.jnt_wheel_l]
        self.wheel_r_dof_adr = self.model.jnt_dofadr[self.jnt_wheel_r]

        # 状态维度：3 (pitch, pitch_dot, wheel_vel)
        self.nx = 3
        self.nu = 1
        self.x_eq = np.zeros(self.nx)
        self.u_eq = 0.0
        self.base_height = base_height
        self.leg_y_offset = leg_y_offset            # 保存偏置
        self.wheel_radius = 0.0703
        self.body_height_target = body_height

        self.left_wheel_sign = -1.0
        self.feedback_sign = feedback_sign

        if body_height is not None:
            self.leg_qpos_ref = self.solve_stance(body_height)
            self.base_height = 0.0
        else:
            self.leg_qpos_ref = np.array([hip_angle, knee_angle, -hip_angle, -knee_angle])

        self.find_equilibrium_torque(eq_torque_limit)
        if no_u_eq:
            print("Forcing u_eq = 0.0 (ignoring computed equilibrium)")
            self.u_eq = 0.0

    def wheel_contact_zs(self, right_hip, right_knee, left_hip, left_knee):
        qpos = self.model.qpos0.copy()
        qvel = np.zeros(self.model.nv)
        qpos[self.base_qpos_adr + 1] = 0.0          # Y 为 0
        qpos[self.base_qpos_adr + 2] = 0.0
        qpos[self.base_qpos_adr + 3 : self.base_qpos_adr + 7] = np.array([1.0, 0.0, 0.0, 0.0])
        for adr, value in zip(self.leg_joint_qpos_adr, [right_hip, right_knee, left_hip, left_knee]):
            qpos[adr] = value
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        mujoco.mj_forward(self.model, self.data)
        z_r = self.data.xpos[self.body_wheel_r][2]
        z_l = self.data.xpos[self.body_wheel_l][2]
        return np.array([z_r, z_l]) - self.wheel_radius

    def solve_stance(self, body_height: float) -> np.ndarray:
        def objective(vars_):
            right_hip, right_knee = vars_
            left_hip, left_knee = -right_hip, -right_knee
            z_right, z_left = self.wheel_contact_zs(right_hip, right_knee, left_hip, left_knee)
            height_error = 0.5 * (z_right + z_left) + body_height
            roll_error = z_right - z_left
            knee_penalty = 1e-4 * right_knee**2
            return float(1e4 * height_error**2 + 1e4 * roll_error**2 + knee_penalty)

        result = minimize(objective, x0=np.array([0.0, 0.0]), bounds=[(-1.57, 1.57), (-1.57, 1.57)],
                          method="L-BFGS-B", options={"ftol": 1e-14, "gtol": 1e-10, "maxiter": 500})
        if not result.success:
            print(f"WARNING: stance IK did not fully converge: {result.message}")
        right_hip, right_knee = result.x
        left_hip, left_knee = -right_hip, -right_knee
        z_right, z_left = self.wheel_contact_zs(right_hip, right_knee, left_hip, left_knee)
        print(f"IK stance: body_height={body_height:.4f} m, right_hip={right_hip:.6f}, right_knee={right_knee:.6f}, "
              f"left_hip={left_hip:.6f}, left_knee={left_knee:.6f}, z_right={z_right:.6f}, z_left={z_left:.6f}")
        height_error = 0.5 * (z_right + z_left) + body_height
        if abs(height_error) > 1e-4:
            print(f"WARNING: final stance height error={height_error:+.6e} m")
        return np.array([right_hip, right_knee, left_hip, left_knee])

    def pitch_acceleration_for_torque(self, u: float) -> float:
        self.set_full_state(self.x_eq)
        self.set_control(u, include_u_eq=False)
        mujoco.mj_forward(self.model, self.data)
        return float(self.data.qacc[self.base_dof_adr + 0])

    def find_equilibrium_torque(self, torque_limit: float = 100.0) -> float:
        limit = abs(float(torque_limit))
        if limit <= 0.0:
            self.u_eq = 0.0
            return self.u_eq
        f0 = self.pitch_acceleration_for_torque(0.0)
        fp = self.pitch_acceleration_for_torque(limit)
        fm = self.pitch_acceleration_for_torque(-limit)
        slope = (fp - fm) / (2.0 * limit)
        self.u_eq = float(np.clip(-f0 / slope, -limit, limit)) if abs(slope) > 1e-12 else 0.0
        try:
            flo = self.pitch_acceleration_for_torque(-limit)
            fhi = self.pitch_acceleration_for_torque(limit)
            if flo * fhi <= 0.0:
                self.u_eq = float(brentq(self.pitch_acceleration_for_torque, -limit, limit, xtol=1e-10, rtol=1e-10, maxiter=100))
        except Exception:
            pass
        final_acc = self.pitch_acceleration_for_torque(self.u_eq)
        print(f"Equilibrium wheel torque: u_eq={self.u_eq:.6f} Nm, pitch_accel={final_acc:+.3e} rad/s^2")
        return self.u_eq

    def get_state(self) -> np.ndarray:
        """返回状态向量 [pitch, pitch_dot, wheel_vel_avg] (3维)"""
        base_quat = self.data.qpos[self.base_qpos_adr + 3 : self.base_qpos_adr + 7]
        pitch = quat_to_pitch(base_quat)
        base_angvel = self.data.qvel[self.base_dof_adr : self.base_dof_adr + 3]
        pitch_dot = base_angvel[0]

        wheel_l_dot = self.data.qvel[self.wheel_l_dof_adr] * self.left_wheel_sign
        wheel_r_dot = self.data.qvel[self.wheel_r_dof_adr]
        wheel_vel_avg = 0.5 * (wheel_r_dot + wheel_l_dot)

        return np.array([pitch, pitch_dot, wheel_vel_avg])

    def set_full_state(self, x: np.ndarray) -> None:
        # x 是 3维：[pitch, pitch_dot, wheel_vel]
        self.data.qpos[:] = self.model.qpos0.copy()
        self.data.qvel[:] = 0.0
        # 基座位置：Y 保持为 0（不整体平移），只设置高度
        self.data.qpos[self.base_qpos_adr + 1] = 0.0
        self.data.qpos[self.base_qpos_adr + 2] = self.base_height
        self.data.qpos[self.base_qpos_adr + 3 : self.base_qpos_adr + 7] = euler_to_quat(x[0], 0.0, 0.0)
        self.data.qvel[self.base_dof_adr : self.base_dof_adr + 3] = np.array([x[1], 0.0, 0.0])

        # 轮子角度设为0（位置归零）
        self.data.qpos[self.wheel_r_qpos_adr] = 0.0
        self.data.qpos[self.wheel_l_qpos_adr] = 0.0

        # 轮子速度
        self.data.qvel[self.wheel_r_dof_adr] = x[2]
        self.data.qvel[self.wheel_l_dof_adr] = x[2] * self.left_wheel_sign

        # 腿关节角度：在参考姿态上加上 leg_y_offset（右髋 +offset，左髋 -offset）
        # 注意：leg_qpos_ref 是 [右髋, 右膝, 左髋, 左膝]
        self.data.qpos[self.leg_joint_qpos_adr[0]] = self.leg_qpos_ref[0] + self.leg_y_offset
        self.data.qpos[self.leg_joint_qpos_adr[1]] = self.leg_qpos_ref[1]
        self.data.qpos[self.leg_joint_qpos_adr[2]] = self.leg_qpos_ref[2] - self.leg_y_offset
        self.data.qpos[self.leg_joint_qpos_adr[3]] = self.leg_qpos_ref[3]

    def set_control(self, u: float | np.ndarray, include_u_eq: bool = True) -> None:
        u = float(np.asarray(u).reshape(-1)[0])
        wheel_torque = self.u_eq + u if include_u_eq else u
        self.data.ctrl[:] = 0.0
        for act_id, qpos in zip(self.leg_actuator_ids, [self.leg_qpos_ref[0] + self.leg_y_offset,
                                                         self.leg_qpos_ref[1],
                                                         self.leg_qpos_ref[2] - self.leg_y_offset,
                                                         self.leg_qpos_ref[3]]):
            self.data.ctrl[act_id] = qpos
        self.data.ctrl[self.act_wheel_r] = wheel_torque
        self.data.ctrl[self.act_wheel_l] = wheel_torque * self.left_wheel_sign

    def step_dynamics(self, x: np.ndarray, u: float) -> np.ndarray:
        self.set_full_state(x)
        self.set_control(u)
        mujoco.mj_forward(self.model, self.data)
        mujoco.mj_step(self.model, self.data)
        return self.get_state()

    def linearize(self, eps: float = 1e-4, u_eps: float = 1e-2):
        x0 = self.x_eq.copy()
        u0 = 0.0
        A = np.zeros((self.nx, self.nx))
        for i in range(self.nx):
            dx = np.zeros(self.nx)
            dx[i] = eps
            x_next_plus = self.step_dynamics(x0 + dx, u0)
            x_next_minus = self.step_dynamics(x0 - dx, u0)
            A[:, i] = (x_next_plus - x_next_minus) / (2.0 * eps)

        B = np.zeros((self.nx, 1))
        x_next_plus = self.step_dynamics(x0, u0 + u_eps)
        x_next_minus = self.step_dynamics(x0, u0 - u_eps)
        B[:, 0] = (x_next_plus - x_next_minus) / (2.0 * u_eps)
        return A, B

    @staticmethod
    def dlqr(A, B, Q, R):
        try:
            P = solve_discrete_are(A, B, Q, R)
            return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        except np.linalg.LinAlgError as e:
            print(f"LQR求解失败: {e}")
            print("尝试调整权重或使用默认参数")
            raise

    @staticmethod
    def controllability_rank(A, B):
        ctrb = B.copy()
        block = B.copy()
        for _ in range(A.shape[0] - 1):
            block = A @ block
            ctrb = np.hstack([ctrb, block])
        return np.linalg.matrix_rank(ctrb)

    def control_torque(self, K, ctrl_limit):
        x = self.get_state()
        delta_u = -self.feedback_sign * float(np.asarray(K @ (x - self.x_eq)).reshape(-1)[0])
        return float(np.clip(delta_u, -ctrl_limit, ctrl_limit))

    def scan_leg_y_offset(self, y_range=(-0.03, -0.02), steps=21, eq_torque_limit=100.0):
        """
        扫描髋关节共同偏置 leg_y_offset，找到 u_eq = 0 的值。
        """
        original_offset = self.leg_y_offset
        results = []
        print(f"Scanning leg_y_offset from {y_range[0]:.4f} to {y_range[1]:.4f} with {steps} steps")
        for off in np.linspace(y_range[0], y_range[1], steps):
            self.leg_y_offset = off
            # 重新计算平衡扭矩（会更新 self.u_eq）
            u_eq = self.find_equilibrium_torque(eq_torque_limit)
            results.append((off, u_eq))
            print(f"leg_y_offset = {off:.6f} rad  ->  u_eq = {u_eq:.6f} Nm")

        # 寻找过零点
        zero_cross = None
        for i in range(1, len(results)):
            o1, u1 = results[i-1]
            o2, u2 = results[i]
            if u1 * u2 < 0:
                # 线性插值
                zero_o = o1 - u1 * (o2 - o1) / (u2 - u1)
                zero_cross = zero_o
                break
        if zero_cross is not None:
            print(f"\n>>> 零扭矩平衡点 leg_y_offset ≈ {zero_cross:.6f} rad")
            print(f"建议运行时加上参数：--leg-y-offset {zero_cross:.6f}")
        else:
            print("\n未找到过零点，请扩大扫描范围或检查腿姿。")

        # 恢复原始偏移
        self.leg_y_offset = original_offset
        # 重新计算原始 u_eq 以防后续使用
        self.find_equilibrium_torque(eq_torque_limit)
        return zero_cross

    def run(self, K, steps, ctrl_limit, print_every, use_viewer, open_loop_torque=None):
        mujoco.mj_resetData(self.model, self.data)
        self.set_full_state(self.x_eq)
        self.set_control(0.0)
        mujoco.mj_forward(self.model, self.data)

        if use_viewer:
            try:
                viewer_module = importlib.import_module("mujoco.viewer")
            except Exception:
                print("Unable to import mujoco.viewer; falling back to headless.")
                use_viewer = False
            else:
                with viewer_module.launch_passive(self.model, self.data) as viewer:
                    step = 0
                    while viewer.is_running() and step < steps:
                        if open_loop_torque is not None:
                            u = open_loop_torque
                        else:
                            u = self.control_torque(K, ctrl_limit)
                        self.set_control(u)
                        mujoco.mj_step(self.model, self.data)
                        if step % print_every == 0:
                            x = self.get_state()
                            print(f"step={step:5d} pitch={x[0]: .4f} pitch_dot={x[1]: .4f} "
                                  f"wheel_vel={x[2]: .4f} "
                                  f"delta_tau={u: .4f} tau={self.u_eq + u: .4f}")
                        viewer.sync()
                        step += 1
        else:
            for step in range(steps):
                if open_loop_torque is not None:
                    u = open_loop_torque
                else:
                    u = self.control_torque(K, ctrl_limit)
                self.set_control(u)
                mujoco.mj_step(self.model, self.data)
                if step % print_every == 0:
                    x = self.get_state()
                    print(f"step={step:5d} pitch={x[0]: .4f} pitch_dot={x[1]: .4f} "
                          f"wheel_vel={x[2]: .4f}")


def main():
    parser = argparse.ArgumentParser(description="3-state LQR balance")
    parser.add_argument("--xml", type=Path, default=Path("assets/robot_urdf/robot.xml"))
    parser.add_argument("--steps", type=int, default=2000)
    parser.add_argument("--eps", type=float, default=1e-4)
    parser.add_argument("--u-eps", type=float, default=1e-2)
    parser.add_argument("--ctrl-limit", type=float, default=200.0)
    parser.add_argument("--eq-torque-limit", type=float, default=100.0)
    parser.add_argument("--print-every", type=int, default=100)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--hip-angle", type=float, default=0.0)
    parser.add_argument("--knee-angle", type=float, default=0.0)
    parser.add_argument("--base-height", type=float, default=0.0)
    parser.add_argument("--body-height", type=float, default=0.4)
    parser.add_argument("--feedback-sign", type=float, default=1.0)

    # LQR权重（3状态）
    parser.add_argument("--q-pitch", type=float, default=5000.0)
    parser.add_argument("--q-pitch-dot", type=float, default=2000.0)
    parser.add_argument("--q-wheel-vel", type=float, default=100.0)
    parser.add_argument("--r-torque", type=float, default=0.001)
    parser.add_argument("--no-u-eq", action="store_true")
    parser.add_argument("--open-loop", type=float, default=None)

    # 新增参数：腿的 Y 方向偏置（髋关节共同偏置）
    parser.add_argument("--leg-y-offset", type=float, default=0.0,
                        help="髋关节共同偏置（右+，左-），用于调整轮子与质心的水平距离")
    parser.add_argument("--scan-offset", action="store_true",
                        help="扫描 leg-y-offset 与 u_eq 的关系，找到零扭矩点后退出")

    args = parser.parse_args()

    balancer = PlanarBalancer(
        args.xml,
        hip_angle=args.hip_angle,
        knee_angle=args.knee_angle,
        base_height=args.base_height,
        body_height=args.body_height,
        eq_torque_limit=args.eq_torque_limit,
        no_u_eq=args.no_u_eq,
        feedback_sign=args.feedback_sign,
        leg_y_offset=args.leg_y_offset,
    )

    # 如果只做扫描，则扫描后退出
    if args.scan_offset:
        balancer.scan_leg_y_offset(eq_torque_limit=args.eq_torque_limit)
        return

    A, B = balancer.linearize(eps=args.eps, u_eps=args.u_eps)
    print("A =\n", A)
    print("B =\n", B)
    rank = balancer.controllability_rank(A, B)
    print("controllability rank =", rank)
    if rank < balancer.nx:
        print(f"警告: 可控性秩 ({rank}) 小于状态维度 ({balancer.nx})，但可尝试继续。")

    Q = np.diag([args.q_pitch, args.q_pitch_dot, args.q_wheel_vel])
    R = np.diag([args.r_torque])
    K = balancer.dlqr(A, B, Q, R)
    print("K =\n", K)

    balancer.run(K, args.steps, args.ctrl_limit, args.print_every, args.viewer,
                 open_loop_torque=args.open_loop)


if __name__ == "__main__":
    main()