"""Linearize the wheel-legged robot and compute LQR balance gains.

This script assumes:
  - the four leg joints are held at their initial angles by position actuators;
  - the two wheel joints are torque-controlled with `motor` actuators.

State vector:
  x = [pitch, pitch_dot, yaw, yaw_dot,
       wheel_left, wheel_right, wheel_left_dot, wheel_right_dot]

Control vector:
  u = [torque_left, torque_right]

Run:
  python lqr_balance.py --xml assets/robot_urdf/robot.xml
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

try:
    import mujoco
except ImportError as exc:  # pragma: no cover - only used at runtime
    raise SystemExit("This script requires the `mujoco` Python package.") from exc

try:
    from scipy.linalg import solve_discrete_are
except ImportError as exc:  # pragma: no cover - only used at runtime
    raise SystemExit("This script requires `scipy`.") from exc


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return a unit quaternion [w, x, y, z] for ZYX Euler angles."""
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


def quat_to_pitch_yaw(q: np.ndarray) -> tuple[float, float]:
    """Return pitch and yaw for a quaternion [w, x, y, z]."""
    w, x, y, z = q
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return pitch, yaw


class Balancer:
    def __init__(self, xml_path: str | Path) -> None:
        self.xml_path = Path(xml_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)

        self.jnt_base = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_joint")
        self.jnt_wheel_l = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "link_004_joint")
        self.jnt_wheel_r = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "link_007_joint")

        self.act_wheel_l = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "link_004_joint_motor"
        )
        self.act_wheel_r = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "link_007_joint_motor"
        )

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

        self.nx = 8
        self.nu = 2
        self.x_eq = np.zeros(self.nx)

    def get_state(self) -> np.ndarray:
        """Read the reduced state from data."""
        base_quat = self.data.qpos[self.base_qpos_adr + 3 : self.base_qpos_adr + 7]
        pitch, yaw = quat_to_pitch_yaw(base_quat)

        base_angvel = self.data.qvel[self.base_dof_adr : self.base_dof_adr + 3]
        pitch_dot = base_angvel[1]
        yaw_dot = base_angvel[2]

        wheel_l = self.data.qpos[self.wheel_l_qpos_adr]
        wheel_r = self.data.qpos[self.wheel_r_qpos_adr]
        wheel_l_dot = self.data.qvel[self.wheel_l_dof_adr]
        wheel_r_dot = self.data.qvel[self.wheel_r_dof_adr]

        return np.array(
            [pitch, pitch_dot, yaw, yaw_dot, wheel_l, wheel_r, wheel_l_dot, wheel_r_dot]
        )

    def set_full_state(self, x: np.ndarray) -> None:
        """Set MuJoCo state from the reduced state."""
        self.data.qpos[:] = self.model.qpos0.copy()
        self.data.qvel[:] = 0.0

        self.data.qpos[self.base_qpos_adr + 3 : self.base_qpos_adr + 7] = euler_to_quat(
            0.0, x[0], x[2]
        )
        self.data.qvel[self.base_dof_adr : self.base_dof_adr + 3] = np.array(
            [0.0, x[1], x[3]]
        )

        self.data.qpos[self.wheel_l_qpos_adr] = x[4]
        self.data.qpos[self.wheel_r_qpos_adr] = x[5]
        self.data.qvel[self.wheel_l_dof_adr] = x[6]
        self.data.qvel[self.wheel_r_dof_adr] = x[7]

    def set_control(self, u: np.ndarray) -> None:
        """Write leg hold commands and wheel torques."""
        self.data.ctrl[:] = 0.0

        for act_id, leg_qpos_adr in zip(self.leg_actuator_ids, self.leg_joint_qpos_adr):
            self.data.ctrl[act_id] = self.model.qpos0[leg_qpos_adr]

        self.data.ctrl[self.act_wheel_l] = float(u[0])
        self.data.ctrl[self.act_wheel_r] = float(u[1])

    def step_dynamics(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Set state/control, advance one MuJoCo step, and return next reduced state."""
        self.set_full_state(x)
        self.set_control(u)
        mujoco.mj_forward(self.model, self.data)
        mujoco.mj_step(self.model, self.data)
        return self.get_state()

    def linearize(
        self,
        eps: float = 1e-6,
        u_eps: float = 1e-6,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Finite-difference discrete-time linearization around x_eq."""
        x0 = self.x_eq.copy()
        u0 = np.zeros(self.nu)
        x_next0 = self.step_dynamics(x0, u0)

        A = np.zeros((self.nx, self.nx))
        for i in range(self.nx):
            dx = np.zeros(self.nx)
            dx[i] = eps
            x_next = self.step_dynamics(x0 + dx, u0)
            A[:, i] = (x_next - x_next0) / eps

        B = np.zeros((self.nx, self.nu))
        for j in range(self.nu):
            du = np.zeros(self.nu)
            du[j] = u_eps
            x_next = self.step_dynamics(x0, u0 + du)
            B[:, j] = (x_next - x_next0) / u_eps

        return A, B

    @staticmethod
    def dlqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
        """Discrete-time infinite-horizon LQR gain."""
        P = solve_discrete_are(A, B, Q, R)
        return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)

    @staticmethod
    def controllability_rank(A: np.ndarray, B: np.ndarray) -> int:
        ctrb = B
        for _ in range(A.shape[0] - 1):
            ctrb = np.hstack([ctrb, A @ ctrb])
        return np.linalg.matrix_rank(ctrb)

    def run(
        self,
        K: np.ndarray,
        steps: int,
        ctrl_limit: float,
        print_every: int,
    ) -> None:
        """Run a closed-loop simulation and print state snapshots."""
        mujoco.mj_resetData(self.model, self.data)

        for step in range(steps):
            x = self.get_state()
            u = -K @ (x - self.x_eq)
            u = np.clip(u, -ctrl_limit, ctrl_limit)
            self.set_control(u)
            mujoco.mj_step(self.model, self.data)

            if step % print_every == 0:
                print(
                    f"step={step:5d} pitch={x[0]: .4f} pitch_dot={x[1]: .4f} "
                    f"yaw={x[2]: .4f} wheel_L={x[4]: .4f} wheel_R={x[5]: .4f}"
                )


def main() -> None:
    parser = argparse.ArgumentParser(description="LQR balance gains for MuJoCo robot")
    parser.add_argument(
        "--xml",
        type=Path,
        default=Path("assets/robot_urdf/robot.xml"),
        help="Path to robot MJCF/XML model",
    )
    parser.add_argument("--steps", type=int, default=2000, help="Number of simulation steps")
    parser.add_argument("--eps", type=float, default=1e-6, help="Finite-difference step for A")
    parser.add_argument("--u-eps", type=float, default=1e-6, help="Finite-difference step for B")
    parser.add_argument("--ctrl-limit", type=float, default=10.0, help="Wheel torque clamp")
    parser.add_argument("--print-every", type=int, default=100)
    parser.add_argument(
        "--q-pitch",
        type=float,
        default=1000.0,
        help="Q weight on pitch angle",
    )
    parser.add_argument(
        "--q-pitch-dot",
        type=float,
        default=10.0,
        help="Q weight on pitch rate",
    )
    parser.add_argument(
        "--q-yaw",
        type=float,
        default=100.0,
        help="Q weight on yaw angle",
    )
    parser.add_argument(
        "--q-yaw-dot",
        type=float,
        default=10.0,
        help="Q weight on yaw rate",
    )
    parser.add_argument(
        "--q-wheel-pos",
        type=float,
        default=100.0,
        help="Q weight on each wheel angle",
    )
    parser.add_argument(
        "--q-wheel-vel",
        type=float,
        default=1.0,
        help="Q weight on each wheel angular velocity",
    )
    parser.add_argument(
        "--r-torque",
        type=float,
        default=0.1,
        help="R weight on each wheel torque",
    )
    args = parser.parse_args()

    balancer = Balancer(args.xml)
    A, B = balancer.linearize(eps=args.eps, u_eps=args.u_eps)

    print("A =\n", A)
    print("B =\n", B)
    print("controllability rank =", balancer.controllability_rank(A, B))

    Q = np.diag(
        [
            args.q_pitch,
            args.q_pitch_dot,
            args.q_yaw,
            args.q_yaw_dot,
            args.q_wheel_pos,
            args.q_wheel_pos,
            args.q_wheel_vel,
            args.q_wheel_vel,
        ]
    )
    R = np.diag([args.r_torque, args.r_torque])
    K = balancer.dlqr(A, B, Q, R)

    print("K =\n", K)
    balancer.run(K, steps=args.steps, ctrl_limit=args.ctrl_limit, print_every=args.print_every)


if __name__ == "__main__":
    main()
