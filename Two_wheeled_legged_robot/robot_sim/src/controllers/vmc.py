from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from src.controllers.phase import JumpPhase, JumpPhaseMachine
from src.controllers.serial_leg_ik import SerialLegIk
from src.geometry import wheel_center_jacobian_z, wheel_center_z
from src.state import SimState, body_id, model_addresses

BASE_BODY_NAME = "base_link"
LEG_JACOBIAN_SINGULAR_VALUE_EPS = 1e-8


@dataclass(frozen=True)
class SerialLegGeometry:
    side: str
    hip_joint: str
    knee_joint: str
    wheel_body: str
    axis_sign: float  # +1: 右腿(+X); -1: 左腿(-X)（仅存档说明，IK 已处理符号）


LEG_CLOSED_LOOP: dict[str, SerialLegGeometry] = {
    "left": SerialLegGeometry(
        side="left",
        hip_joint="link_005_joint",
        knee_joint="link_006_joint",
        wheel_body="link_007",
        axis_sign=-1.0,
    ),
    "right": SerialLegGeometry(
        side="right",
        hip_joint="link_002_joint",
        knee_joint="link_003_joint",
        wheel_body="link_004",
        axis_sign=1.0,
    ),
}


@dataclass
class VmcParams:
    """VMC leg-height + jump-phase controller parameters.

    调参顺序 (实机/headless 都建议从上往下冻结):
      1. nominal_height            — stand 站姿高度 h_base (m), 必须在 ik 范围内
      2. kp_motor / kd_motor       — joint-space PD on hip/knee motor angles
                                     (STAND/CROUCH/EXTEND 通用)
      3. kp_land  / kd_land        — LAND 阶段独立 PD, 比 STAND 软 P 硬 D 吸冲击
      4. flight_pitch_kd           — FLIGHT 期间对称腿 motor 上的 pitch_rate 阻尼
      5. max_height_rate           — nominal_height 步进上限 (m/s)
      6. roll_level_*              — STAND 斜坡找平偏置 (默认 0, 仅 ramp 实验启用)
      7. stand_torque_limit        — STAND/CROUCH/LAND 的腿电机软限幅
    """

    # nominal_height is runtime-writable so the cmd_height slider can drive VMC.
    # 单位与 _leg_height() 一致：h_base = base_link.z - wheel.z。
    nominal_height: float
    # Joint-space PID gains on hip/knee motor angles (per 关节, N·m/rad, N·m·s/rad)。
    # 与串行 IK 的目标角配合，替代原四连杆的任务空间高度弹簧。
    kp_motor: float
    kd_motor: float
    # LAND 阶段独立的 PD 增益 (绝对值, 不是 STAND 增益的缩放). 落地需要软 P + 强 D
    # 吸收冲击, 跟 STAND 跟踪目标的需求不同. 用绝对值消除"调 STAND 同时改 LAND"
    # 的隐式耦合.
    kp_land: float = 15.0
    kd_land: float = 3.5
    max_height_rate: float = 0.1
    # FLIGHT 期间对称腿 motor 上的 pitch_rate 阻尼增益 (N·m per rad/s).
    # 见 _control() FLIGHT 分支注释 — 仅 D 项, 加 P 会与 leg-gravity 偏置形成正反馈.
    flight_pitch_kd: float = 1.5
    # STAND 斜坡找平: 测左右轮高差做前馈, 调左右腿高差把 base 调平 (纯前馈, kp 默认 0).
    # roll_level_offset_limit 是左右腿高差的单侧上限 (m); 0 = off.
    roll_level_kp_height: float = 0.0
    roll_level_kd_height: float = 0.0
    roll_level_offset_limit: float = 0.0
    # 上坡降站高: 找平需要大高差时降低共模站高, 给"伸长"那条腿留出 ik 上限余量.
    # 纯前馈, 由当前 roll_level 偏置量驱动. 值 = 伸长腿离 h_max 至少保留的余量 (m); 0 = off.
    slope_squat_margin: float = 0.0
    # 串行腿解析 IK（站姿律：轮心在髋正下方；h 单位同 nominal_height）
    ik: SerialLegIk = field(default_factory=SerialLegIk)
    # STAND/CROUCH/LAND 腿电机软限幅（N·m/关节）；EXTEND 允许拉到 actuator 上限。
    stand_torque_limit: float = 30.0
    # 接触逆动力学前馈（ground FF）：用 MuJoCo mj_inverse（qacc=0、含地面接触）
    # 求"撑住当前姿势每个关节需要的静态力矩"，直接作为前馈。
    # 只在轮子着地（ncon>=2）时启用；悬空测试请保持 False（会推空）。
    # 实机移植时需替换为标定好的重力/负载模型，不能依赖 MuJoCo。
    gravity_ff_enabled: bool = False
    # 腿自身重力前馈（= MuJoCo qfrc_bias 的腿关节分量），
    # 用于"固定机身/悬空"这类只测腿的场合；站立/整车支撑时不完整，需配 gravity_ff_enabled。
    leg_gravity_ff_enabled: bool = False


class VmcController:
    JACOBIAN_REFRESH_PERIOD = 10  # recompute leg motor jacobian every N control steps
    GROUND_FF_REFRESH_PERIOD = 10  # recompute contact inverse-dynamics FF every N steps

    def __init__(self, params: VmcParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.phase_machine = phase_machine
        self._neutral_height_offsets: dict[int, dict[str, float]] = {}
        self._height_jacobian_rows_cache: dict[str, np.ndarray] | None = None
        self._contact_ff_cache: dict[str, float] | None = None
        self._motor_jacobian_step_count: int = 0
        self._height_filtered: float | None = None
        self._crouch_start_height: float | None = None
        self._target_motor_angle_prev: dict[str, dict[str, float]] = {}
        self.last_target_motor_rate: dict[str, float] = {side: 0.0 for side in LEG_CLOSED_LOOP}
        self.last_target_heights: dict[str, float] = {side: 0.0 for side in LEG_CLOSED_LOOP}

    def __call__(self, model: mujoco.MjModel, data: mujoco.MjData, state: SimState) -> np.ndarray:
        return self._control(model, data, state, update_phase=True)

    def preview_control(self, model: mujoco.MjModel, data: mujoco.MjData, state: SimState) -> np.ndarray:
        """Compute VMC output without advancing the jump phase machine."""
        return self._control(model, data, state, update_phase=False)

    def leg_height_jacobian(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        """Return dh/dq for hip and knee motors of each side (串行腿).

        返回形如 {"left": {"link_005_joint": v, "link_006_joint": v}, ...}。
        h 定义同 _leg_height()：h_base = base_link.z - wheel.z。
        """
        addresses = model_addresses(model)
        rows = _leg_height_jacobian_rows(model, data)
        return {
            side: {
                joint: float(rows[side][addresses.joint_qvel[joint]])
                for joint in (geometry.hip_joint, geometry.knee_joint)
            }
            for side, geometry in LEG_CLOSED_LOOP.items()
        }

    def _control(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        state: SimState,
        *,
        update_phase: bool,
    ) -> np.ndarray:
        params_values = np.array([
            self.params.nominal_height,
            self.params.kp_motor,
            self.params.kd_motor,
            self.params.kp_land,
            self.params.kd_land,
            self.params.max_height_rate,
            self.params.flight_pitch_kd,
            self.params.roll_level_kp_height,
            self.params.roll_level_kd_height,
            self.params.roll_level_offset_limit,
            self.params.slope_squat_margin,
        ])
        if not np.all(np.isfinite(params_values)):
            raise ValueError("VMC params must be finite")
        if not np.all(np.isfinite(state.base_position)) or not np.all(np.isfinite(state.base_linear_velocity)):
            raise ValueError("VMC state must be finite")

        phase = self.phase_machine.phase if self.phase_machine is not None else JumpPhase.STAND
        current_leg_height = _average_leg_height(model, data)
        if self.phase_machine is not None and update_phase:
            self.phase_machine.update(
                dt=float(model.opt.timestep),
                leg_height=current_leg_height,
                vz=float(state.base_linear_velocity[2]),
                contact_count=state.contact_count,
                pitch=float(state.pitch),
            )
            phase = self.phase_machine.phase

        # FLIGHT: 用对称腿电机做 pitch_rate 反作用阻尼。
        # 物理依据: 电机力矩 τ 给腿 → Newton 3rd 给 base 反作用 -τ (绕 pitch 轴)。
        # 左右两个电机同向施加 τ → base 受 -2τ 的 pitch 力矩,可主动减速 pitch_rate。
        # (旧实现返回 0 是"干净"但起跳后 pitch_rate 累积 -4 rad/s,250ms 飞行翻 1 rad)
        #
        # 注意: 必须严格对称 (左右同 τ),否则会产生 roll 力矩。
        # 不对 height/位置做反馈 (legs 在空中无 ground reaction,kp 没意义);
        # 只做 pitch_rate 一维阻尼。
        # FALLEN: 完全 0,不再控制。
        if phase == JumpPhase.FALLEN:
            self.last_target_motor_rate = {side: 0.0 for side in LEG_CLOSED_LOOP}
            return np.zeros(model.nu)
        if phase == JumpPhase.FLIGHT:
            self.last_target_motor_rate = {side: 0.0 for side in LEG_CLOSED_LOOP}
            trajectory = self.phase_machine.trajectory if self.phase_machine is not None else None
            lock_height = float(trajectory.h_high) if trajectory is not None else float(self.params.nominal_height)
            self.last_target_heights = {side: lock_height for side in LEG_CLOSED_LOOP}
            # 空中姿态: 仅 pitch_rate 阻尼 (PD 中不加 K_p 位置项)。
            # τ_motor = -K_d * pitch_rate, 两腿对称同号 → 通过 four-bar 给 base 反向
            # pitch 力矩。flight_pitch_kd 默认 1.5, clip ±3.5 N·m 是实测稳定点。
            #
            # 为什么不加 K_p:
            # FLIGHT 期间 leg gravity 摆动 + EXTEND 末期 leg 角动量回流给 base 一个
            # 恒定 ~+2.3 N·m 的偏置 pitch 力矩, 让 pitch_rate 在 ~-1.5 rad/s 形成
            # 稳态平衡. 若加位置项 P, 与该偏置形成正反馈环, 实测 pitch 发散到 ±0.6 rad.
            # 因此只用 D 项, 接受残留稳态 pitch_rate (落地由 LAND 阶段吸收)。
            pitch_kd = float(self.params.flight_pitch_kd)
            attitude_torque = float(np.clip(
                -pitch_kd * float(state.pitch_rate), -3.5, 3.5,
            ))
            addresses = model_addresses(model)
            control = np.zeros(model.nu)
            # 串行腿 TODO(阶段6 跳跃)：空中姿态阻尼先只驱动髋，膝保持 0。
            for geometry in LEG_CLOSED_LOOP.values():
                act_idx = addresses.actuators[geometry.hip_joint]
                lo, hi = model.actuator_ctrlrange[act_idx]
                control[act_idx] = float(np.clip(attitude_torque, lo, hi))
            return control

        target_height = self._filtered_nominal_height(float(model.opt.timestep))
        target_h_dot = 0.0   # 期望 CoM 垂直速度,用于 motor velocity FF
        target_h_ddot = 0.0  # 期望 CoM 垂直加速度,用于动态 thrust FF
        kp_motor = self.params.kp_motor
        kd_motor = self.params.kd_motor
        trajectory = self.phase_machine.trajectory if self.phase_machine is not None else None

        if phase == JumpPhase.CROUCH and trajectory is not None:
            t = self.phase_machine.time_in_phase  # type: ignore[union-attr]
            target_height = trajectory.crouch.height(t)
            target_h_dot = trajectory.crouch.velocity(t)
            target_h_ddot = trajectory.crouch.acceleration(t)
        elif phase == JumpPhase.EXTEND and trajectory is not None:
            t = self.phase_machine.time_in_phase  # type: ignore[union-attr]
            extend_duration = float(trajectory.extend.duration)
            target_height = trajectory.extend.height(t)
            target_h_dot = trajectory.extend.velocity(t)
            target_h_ddot = trajectory.extend.acceleration(t)
            if t <= extend_duration:
                # 跟轨迹: 1.5x kp/kd 跟踪轨迹位置。不能更高: 3x kp + 50ms 内
                # 快速变化的 target → motor 振荡 ±20 N·m 把执行器顶到饱和。
                kp_motor *= 1.5
                kd_motor *= 1.5
            else:
                # 轨迹跑完但相位机还在 EXTEND (弹跳确认期/能量补推): 关闭位置 kp,
                # 只保留速度跟踪 + ff_torque (= m*(g+a)/N * dh/dθ)。
                #
                # 为什么必须关 kp: ConstantAccelerationTrajectory 在 t>duration 后
                # height 被 clip 到 h_high (常数)。但 motor θ 因为 ff_torque 持续推
                # + body 上升动量,会过冲到对应 h>h_high 的角度。kp*(θ_target-θ_current)
                # 变成负的 → PID 主动把 motor 拉回 → body vz 从 1.5 m/s 在 2ms 内
                # 掉到 0.9 (实测),起跳能量被 PID 自己浪费在 ground bounce 上。
                # ff_torque 是恒定加速度 profile 的正确推力,留它继续推就行。
                kp_motor = 0.0
                kd_motor *= 1.5
        elif phase == JumpPhase.LAND and trajectory is not None:
            # 入 LAND 瞬间生成 land 轨迹。h_target 传入当前 nominal_height
            # 而不是默认 0.142,这样落地不会强行伸腿到中位然后立刻收回。
            if trajectory.land is None:
                trajectory.setup_land(
                    h_contact=current_leg_height,
                    v_contact=float(state.base_linear_velocity[2]),
                    h_target=float(self.params.nominal_height),
                )
            t = self.phase_machine.time_in_phase  # type: ignore[union-attr]
            assert trajectory.land is not None
            target_height = trajectory.land.height(t)
            target_h_dot = trajectory.land.velocity(t)
            target_h_ddot = trajectory.land.acceleration(t)
            # LAND 用独立的绝对 PD 增益 (kp_land/kd_land), 与 STAND 解耦.
            # 软 P + 硬 D 吸收落地冲击, 不再以 STAND kp_motor 的倍数表达,
            # 避免"调 STAND 同时改 LAND"的隐式耦合.
            kp_motor = float(self.params.kp_land)
            kd_motor = float(self.params.kd_land)
        # STAND: 用 filtered nominal_height,target_h_ddot=0 (静态保持)

        control = np.zeros(model.nu)
        addresses = model_addresses(model)
        total_mass = float(np.sum(model.body_mass))
        gravity = abs(float(model.opt.gravity[2]))
        if self.params.leg_gravity_ff_enabled:
            # 刷新 qfrc_bias 到当前位形（mj_forward 不改变状态）
            mujoco.mj_forward(model, data)
        use_contact_ff = bool(self.params.gravity_ff_enabled and data.ncon >= 2)
        if use_contact_ff and self._motor_jacobian_step_count % self.GROUND_FF_REFRESH_PERIOD == 0:
            # qacc=0 的逆动力学：得到"让当前状态保持静止"每个关节需要的力矩（含地面反力）
            qacc_save = np.array(data.qacc, copy=True)
            data.qacc[:] = 0.0
            mujoco.mj_inverse(model, data)
            self._contact_ff_cache = {
                joint_name: float(data.qfrc_inverse[addresses.joint_qvel[joint_name]])
                for geometry in LEG_CLOSED_LOOP.values()
                for joint_name in (geometry.hip_joint, geometry.knee_joint)
            }
            data.qacc[:] = qacc_save
        if self._motor_jacobian_step_count % self.JACOBIAN_REFRESH_PERIOD == 0:
            self._height_jacobian_rows_cache = _leg_height_jacobian_rows(model, data)
        rows = self._height_jacobian_rows_cache
        assert rows is not None
        self._motor_jacobian_step_count += 1

        neutral_offsets = self._neutral_offsets(model, data)
        roll_level_offsets = self._roll_level_height_offsets(model, data, phase, state)
        common_height = self._slope_squat_height(target_height, roll_level_offsets, phase)

        for side, geometry in LEG_CLOSED_LOOP.items():
            side_target_height = common_height + neutral_offsets[side] + roll_level_offsets[side]
            side_target_height = self.params.ik.clamp_height(side_target_height)
            self.last_target_heights[side] = side_target_height

            target_hip, target_knee = self.params.ik.angles_from_base_height(side_target_height, side)
            dt = float(model.opt.timestep)
            previous = self._target_motor_angle_prev.get(side)
            if previous is None:
                target_rates = {geometry.hip_joint: 0.0, geometry.knee_joint: 0.0}
                self.last_target_motor_rate[side] = 0.0
            else:
                target_rates = {
                    geometry.hip_joint: (target_hip - previous[geometry.hip_joint]) / dt,
                    geometry.knee_joint: (target_knee - previous[geometry.knee_joint]) / dt,
                }
                self.last_target_motor_rate[side] = 0.5 * (
                    abs(target_rates[geometry.hip_joint])
                    + abs(target_rates[geometry.knee_joint])
                )
            self._target_motor_angle_prev[side] = {
                geometry.hip_joint: target_hip,
                geometry.knee_joint: target_knee,
            }

            height_row = rows[side]
            joint_specs = (
                (geometry.hip_joint, target_hip, target_rates[geometry.hip_joint]),
                (geometry.knee_joint, target_knee, target_rates[geometry.knee_joint]),
            )
            for joint_name, target_angle, target_rate in joint_specs:
                act_idx = addresses.actuators[joint_name]
                qpos_idx = addresses.joint_qpos[joint_name]
                qvel_idx = addresses.joint_qvel[joint_name]
                theta_current = float(data.qpos[qpos_idx])
                theta_rate = float(data.qvel[qvel_idx])
                pid_torque = (
                    kp_motor * (target_angle - theta_current)
                    + kd_motor * (target_rate - theta_rate)
                )

                ff_torque = 0.0
                if use_contact_ff and self._contact_ff_cache is not None:
                    # 接触逆动力学静态前馈（撑住整车，轮子着地时使用）
                    ff_torque = self._contact_ff_cache.get(joint_name, 0.0)
                elif phase != JumpPhase.FLIGHT and self.params.leg_gravity_ff_enabled:
                    # 腿/轮自身重力的精确关节前馈（固定机身测试）。
                    ff_torque = float(data.qfrc_bias[addresses.joint_qvel[joint_name]])
                control[act_idx] = ff_torque + pid_torque

        clipped = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        if phase in (JumpPhase.STAND, JumpPhase.CROUCH, JumpPhase.LAND):
            limit = float(self.params.stand_torque_limit)
            for geometry in LEG_CLOSED_LOOP.values():
                for joint_name in (geometry.hip_joint, geometry.knee_joint):
                    act_idx = addresses.actuators[joint_name]
                    clipped[act_idx] = float(np.clip(clipped[act_idx], -limit, limit))
        # EXTEND: 不做二次 clip,允许动态 FF 拉满到 actuator 上限推起跳。
        # FLIGHT: 已在函数顶部早返回,不会进入此分支。
        if not np.all(np.isfinite(clipped)):
            raise ValueError("VMC control must be finite")
        return clipped

    def _filtered_nominal_height(self, dt: float) -> float:
        target = float(self.params.nominal_height)
        if self._height_filtered is None:
            self._height_filtered = target
            return target
        max_step = max(float(self.params.max_height_rate), 0.0) * dt
        delta = float(np.clip(target - self._height_filtered, -max_step, max_step))
        self._height_filtered += delta
        return self._height_filtered

    def _roll_level_height_offsets(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        phase: JumpPhase,
        state: SimState,
    ) -> dict[str, float]:
        if phase != JumpPhase.STAND:
            return {side: 0.0 for side in LEG_CLOSED_LOOP}
        limit = max(float(self.params.roll_level_offset_limit), 0.0)
        if limit <= 0.0:
            return {side: 0.0 for side in LEG_CLOSED_LOOP}
        left_wheel_z = wheel_center_z(model, data, LEG_CLOSED_LOOP["left"].wheel_body)
        right_wheel_z = wheel_center_z(model, data, LEG_CLOSED_LOOP["right"].wheel_body)
        terrain_offset = -0.5 * (left_wheel_z - right_wheel_z)
        offset = terrain_offset + (
            float(self.params.roll_level_kp_height) * float(state.roll)
            + float(self.params.roll_level_kd_height) * float(state.roll_rate)
        )
        offset = float(np.clip(offset, -limit, limit))
        return {"left": offset, "right": -offset}

    def _slope_squat_height(
        self,
        target_height: float,
        roll_level_offsets: dict[str, float],
        phase: JumpPhase,
    ) -> float:
        """上坡降站高: 找平指令大高差时降低共模站高, 让"伸长"腿不顶 LUT 上限。

        cap = h_max - margin - 最大单侧偏置。纯前馈 (由 roll_level 偏置驱动),
        不引入 roll 闭环。平地 (偏置≈0) 时 cap > nominal → 原样返回, 不影响平地;
        非 STAND (跳跃) 时直接返回 target_height。
        """
        margin = float(self.params.slope_squat_margin)
        if margin <= 0.0 or phase != JumpPhase.STAND:
            return target_height
        max_offset = max((abs(v) for v in roll_level_offsets.values()), default=0.0)
        cap = float(self.params.ik.h_max) - margin - max_offset
        return float(min(target_height, cap))

    def _neutral_offsets(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
        offsets = self._neutral_height_offsets.get(id(model))
        if offsets is None:
            average_height = _average_leg_height(model, data)
            offsets = {
                side: _leg_height(model, data, geometry.wheel_body) - average_height
                for side, geometry in LEG_CLOSED_LOOP.items()
            }
            self._neutral_height_offsets[id(model)] = offsets
        return offsets


def _average_leg_height(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    heights = [
        _leg_height(model, data, geometry.wheel_body)
        for geometry in LEG_CLOSED_LOOP.values()
    ]
    return float(np.mean(heights))


def _leg_height(model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str) -> float:
    base_id = body_id(model, BASE_BODY_NAME)
    wheel_id = body_id(model, wheel_body)
    # 用机身 body 原点 (xpos) 而不是质心 (xipos)：IK 的 h_base 定义
    # 是"机身原点 - 轮心"。base_link 的 ipos z=+0.011，若用 xipos 会
    # 系统性偏高 11mm（早期阶段3测试误差 +12~15mm 即此原因）。
    return float(data.xpos[base_id, 2] - wheel_center_z(model, data, wheel_body))


def _leg_height_jacobian_rows(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, np.ndarray]:
    return {
        side: _leg_height_jacobian_row(model, data, geometry.wheel_body)
        for side, geometry in LEG_CLOSED_LOOP.items()
    }


def _leg_height_jacobian_row(model: mujoco.MjModel, data: mujoco.MjData, wheel_body: str) -> np.ndarray:
    base_id = body_id(model, BASE_BODY_NAME)
    base_jac = np.zeros((3, model.nv))
    wheel_id = body_id(model, wheel_body)
    mujoco.mj_jac(model, data, base_jac, None, data.xpos[base_id], base_id)
    wheel_jacobian_z = wheel_center_jacobian_z(model, data, wheel_body)
    jacobian = base_jac[2] - wheel_jacobian_z
    if not np.all(np.isfinite(jacobian)):
        raise ValueError("VMC leg height jacobian must be finite")
    return jacobian


def _leg_height_velocity_from_row(data: mujoco.MjData, jacobian: np.ndarray) -> float:
    velocity = jacobian @ data.qvel
    if not np.isfinite(velocity):
        raise ValueError("VMC leg height velocity must be finite")
    return float(velocity)
