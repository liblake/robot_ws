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
    # FLIGHT 膝关节位置保持 PD（N·m/rad, N·m·s/rad）。空中姿态反作用力偶由髋
    # 独自承担（flight_pitch_kd 分支），膝用小刚度 PD 锁在入 FLIGHT 瞬间的角度
    # （= 起跳蹬直位）：落地初始姿态可控、膝不被空中反作用力慢慢甩弯。
    # 0 = 旧行为（膝力矩 0，腿靠惯性保持）。
    flight_knee_kp: float = 80.0
    flight_knee_kd: float = 3.0
    # 空中姿态驱动方式。True（默认）= 左右髋给反号关节力矩（世界系叠加，有效）。
    # False = 复现镜像修正之前的旧行为（两侧同号，世界系力矩互相抵消，只摆腿）。
    # 只用于论文里的 "修正前 / 后" 对照实验，正常运行保持 True。
    flight_attitude_diff_enable: bool = True
    # 蜷腿跳（tuck）：腾空时收腿抬高轮子（轮子离地 = 质心弹道 + 收腿量），
    # 落地前重新展开。flight_tuck_height 为收腿目标腿高（h_base）。
    # 滞空 <0.25 s 的小跳跃自动退回膝锁存（时间不够完成收放）。
    flight_tuck_enable: bool = True
    flight_tuck_height: float = 0.35
    # 蜷腿跟踪 PD（D 项带目标角速度前馈：kd×(θ̇_target − θ̇)，否则阻尼项
    # 对抗收腿运动本身，蜷缩深度损失一半）。力矩上限由 ctrlrange 兜底。
    flight_tuck_kp: float = 200.0
    flight_tuck_kd: float = 8.0
    # 动态轮偏置覆盖（m）。由 CombinedController 按相位驱动：跳跃蹬伸时 →0
    # （推力垂直化，消除水平分量导致的前向加速与落地拖拽），STAND 缓慢回到
    # 标称值。None = 用 IK 标称偏置。
    dynamic_wheel_y_offset: float | None = None
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
    # STAND/CROUCH 腿电机软限幅（N·m/关节）；EXTEND 允许拉到 actuator 上限。
    stand_torque_limit: float = 30.0
    # LAND 阶段独立的腿电机软限幅（N·m/关节），2026-09-13 从"与 STAND 共用 30"
    # 拆出来。落地缓冲需要远大于静态保持的力矩（触地 v≈1.1~1.5 m/s 时缓冲环节
    # 峰值 ~40-50 N·m，其中大部分由动态推力前馈承担），30 的 STAND 限幅在触地步
    # 顶死、缓冲变形。60 = 执行器 ctrlrange（2026-09-13 随跳跃增高从 40 上调），
    # 即 LAND 不做额外软限幅、允许满量程缓冲（EXTEND 本来就不软限幅，逻辑对齐）。
    land_torque_limit: float = 60.0
    # 地面支撑前馈（support FF）：解析式计算"撑住整车每个关节需要的静态力矩"。
    #
    #   τ_j = (∂h/∂q_j)·(m_total·g)/N_legs  +  Σ_{i∈腿链} m_i·g·(∂z_i/∂q_j)
    #         \_______整车重量经轮子传导_______/   \____腿连杆自重(关节重力矩)____/
    #
    # 两项都只是刚体几何量（腿高雅可比 + 各连杆质心竖直雅可比），实机上用
    # IK/正运动学就能算，不依赖任何仿真器内部量。
    # 2026-09-12 替换掉旧的 mj_inverse（qacc=0 接触逆动力学）方案：实测两者
    # 在站立位形差 <0.05%（14 N·m 量级上差 0.008 N·m），差别只是旧方案还多
    # 补了一项速度相关项（Coriolis），静止时≈0。
    # 只在轮子着地（ncon>=2）时启用；悬空测试请保持 False（会推空）。
    gravity_ff_enabled: bool = False
    # 支撑前馈里是否叠加"腿连杆自重"那一项。默认 True。
    # 开源车（2.2 kg，四连杆空心杆）腿链只有零点几 kg，该项可忽略，所以它只
    # 留了第一项；本机腿链 3.50 kg/腿（总质量的 21%），只算第一项会在膝上差
    # 5.5 N·m、髋上差 3.6 N·m（实测），靠 kp=200 反推会让静立腿高偏 7.2 mm。
    # 置 False 即退化成开源车那条"只算载荷传导"的公式。
    support_ff_include_leg_weight: bool = True
    # 腿自身重力前馈（= MuJoCo qfrc_bias 的腿关节分量），
    # 用于"固定机身/悬空"这类只测腿的场合；站立/整车支撑时不完整，需配 gravity_ff_enabled。
    leg_gravity_ff_enabled: bool = False
    # STAND 相位"目标角速度前馈"的比例系数（1.0 = 完整，0 = 关闭）。
    # 上游的实现是 kd_motor·(ḣ_target/J − θ̇)：让关节阻尼项跟随目标角速度而不是
    # 单纯刹车。本机 2026-09-11 曾把它整个置 0：当时找平/高度指令变化快时目标角
    # 速率可达 0.5~0.94 rad/s，kd·rate 单独就把腿电机打到 ±30 软限幅，过障时腿
    # 跟不动目标并翻车。
    #
    # 2026-09-12 复测后恢复为 1.0。当时的失稳真因是**支撑前馈用 mj_inverse**
    # （动态下不可靠，见 RESULTS_ARCHIVE 第 9 节）而不是这条速度前馈；前馈换成
    # 解析式之后，本条前馈反而成了"腿主动去追地形"的关键手段——关掉它腿只能靠
    # 位置误差被动跟随，地形变化快时轮子就离地/车身侧倾。
    # 实测（左轮单侧梯形坡，过坡段 roll 峰峰 / 轮子离地占比 / 腿电机饱和占比）：
    #   65 mm @0.3 m/s：9.31° / 0.3% / 0.9%  →  0.30° / 0.0% / 0.0%
    #   65 mm @0.45    ：10.97° / 4.6% / 4.0% →  0.27° / 0.0% / 0.4%
    #   40 mm @0.45    ：5.73° / 0.3% / 0.0%  →  0.14° / 0.0% / 0.0%
    #   波浪路 @0.8    ：离地 9.4% → 6.5%（1.0 m/s：20.4% → 18.9%）
    #   平地 0.8 m/s   ：逐位不变（该前馈只在腿高目标变化时起作用）
    # 恢复后本机 65 mm 坡 roll 峰峰 0.27° 已优于开源对照车的 3.76°。
    stand_rate_ff_scale: float = 1.0
    # 跳跃相位（CROUCH/EXTEND）的关节 PD 乘数：kp 照上游放大 1.5×，
    # 但 **kd 必须压小**（60 → 9）。
    #
    # 实测（EXTEND，膝关节角度在 0.19 s 内变化 ≈0.47 rad、比例项 kp=300）：
    #   kd=90（60×1.5）：髋关节角速度逐拍交替 ±0.5 rad/s（98 步里 89 次变号），
    #                    D 项把这个数值自激放大成 ±40 N·m，腿电机全程顶在执行器
    #                    饱和上，起跳能量被抖振吃掉 → **完全跳不起来**。
    #   kd=9（60×0.15）：变号 1/98，EXTEND 力矩峰 36 N·m（不饱和），
    #                    弹道高度反而是最好的一组（60.9 mm vs 关掉速度前馈的 44.9）。
    # 机理：D 项在本机被当作"速度前馈"用（kd·(θ̇_target − θ̇)），增益大时
    # 与关节-地面接触这条刚度环形成 2 步极限环；kp 不需要动，位置跟踪靠
    # 动态推力前馈 + 比例项就够。
    jump_kp_scale: float = 1.5
    jump_kd_scale: float = 0.15
    # 跳跃相位（CROUCH/EXTEND/LAND）"目标角速度前馈"的乘数（1.0 = 完整）。
    # 这条前馈是必要的：本机每腿 2 个自由度，"轮心保持在髋正下方"的站姿律让
    # 髋角在伸腿过程中摆动 ≈0.47 rad，这一段姿态运动不在腿高方向、动态推力前馈
    # 管不到，必须由关节速度前馈来驱动。实测关掉它弹道高度从 60.9 mm 掉到 44.9 mm。
    jump_rate_ff_scale: float = 1.0


class VmcController:
    JACOBIAN_REFRESH_PERIOD = 10  # recompute leg motor jacobian every N control steps
    SUPPORT_FF_REFRESH_PERIOD = 10  # recompute analytic support FF every N steps

    def __init__(self, params: VmcParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.phase_machine = phase_machine
        self._neutral_height_offsets: dict[int, dict[str, float]] = {}
        self._height_jacobian_rows_cache: dict[str, np.ndarray] | None = None
        self._support_ff_cache: dict[str, float] | None = None
        self._motor_jacobian_step_count: int = 0
        self._height_filtered: float | None = None
        self._crouch_start_height: float | None = None
        self._target_motor_angle_prev: dict[str, dict[str, float]] = {}
        self._flight_knee_hold: dict[str, float] | None = None
        # 动态轮偏置覆盖（由 CombinedController 按相位写入；None=用 IK 标称值）。
        self.dynamic_wheel_y_offset: float | None = None
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
        phase_before_update = phase
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
            self._flight_knee_hold = None
            return np.zeros(model.nu)
        if phase == JumpPhase.FLIGHT:
            self.last_target_motor_rate = {side: 0.0 for side in LEG_CLOSED_LOOP}
            trajectory = self.phase_machine.trajectory if self.phase_machine is not None else None
            lock_height = float(trajectory.h_high) if trajectory is not None else float(self.params.nominal_height)
            addresses = model_addresses(model)
            control = np.zeros(model.nu)

            # 蜷腿（tuck）策略：起跳后收腿（大腿上摆+小腿折叠），轮子相对机身
            # 再抬高（tuck 收腿量），落地前重新展开到位吸收冲击。轮子离地高度
            # = 质心弹道 + 收腿量 —— 12 cm 质心跳 + 0.13 m 收腿 ≈ 轮子 23 cm。
            # 收腿/展腿由对称 IK 目标 + 关节 PD 驱动；空中俯仰反作用由镜像髋
            # 阻尼器（下方）对抗。
            t_flight = 0.0
            if trajectory is not None and trajectory.v_takeoff > 1e-6:
                # 滞空估计：质心离地速度 ≈ 0.63×腿高变化率（2026-09-12 标定）
                t_flight = 2.0 * 0.63 * float(trajectory.v_takeoff) / 9.81
            tuck_active = (
                self.params.flight_tuck_enable
                and trajectory is not None
                and t_flight >= 0.25
            )
            if tuck_active:
                t_fly = float(self.phase_machine.time_in_phase) \
                    if self.phase_machine is not None else 0.0
                h_target = _flight_tuck_height_profile(
                    t_fly, t_flight, lock_height, float(self.params.flight_tuck_height),
                )
            else:
                h_target = lock_height
            self.last_target_heights = {side: h_target for side in LEG_CLOSED_LOOP}

            # 非 tuck 场景（小跳跃/禁用）：保留膝位置保持——入 FLIGHT 瞬间锁存
            # 膝角（起跳蹬直位），小刚度 PD 锁住，落地初始膝角可控。
            if not tuck_active and self._flight_knee_hold is None:
                hold_addresses = model_addresses(model)
                self._flight_knee_hold = {
                    geometry.knee_joint: float(
                        data.qpos[hold_addresses.joint_qpos[geometry.knee_joint]]
                    )
                    for geometry in LEG_CLOSED_LOOP.values()
                }
            knee_kp = float(self.params.flight_knee_kp)
            knee_kd = float(self.params.flight_knee_kd)
            if not tuck_active and knee_kp > 0.0 and self._flight_knee_hold is not None:
                for geometry in LEG_CLOSED_LOOP.values():
                    act_idx = addresses.actuators[geometry.knee_joint]
                    qpos_idx = addresses.joint_qpos[geometry.knee_joint]
                    qvel_idx = addresses.joint_qvel[geometry.knee_joint]
                    tau = knee_kp * (
                        self._flight_knee_hold[geometry.knee_joint] - float(data.qpos[qpos_idx])
                    ) + knee_kd * (0.0 - float(data.qvel[qvel_idx]))
                    lo, hi = model.actuator_ctrlrange[act_idx]
                    control[act_idx] = float(np.clip(tau, lo, hi))
            if tuck_active:
                # 收腿/展腿跟踪 PD + 目标角速度前馈（D 项朝目标速度收敛而非
                # 朝零速刹车——后者会对抗收腿运动本身，深度损失一半）。
                tuck_kp = float(self.params.flight_tuck_kp)
                tuck_kd = float(self.params.flight_tuck_kd)
                dt = float(model.opt.timestep)
                h_prev = self._flight_tuck_h_prev
                for geometry in LEG_CLOSED_LOOP.values():
                    target_hip, target_knee = self.params.ik.angles_from_base_height(
                        h_target, geometry.side, self.dynamic_wheel_y_offset,
                    )
                    if h_prev is not None:
                        prev_hip, prev_knee = self.params.ik.angles_from_base_height(
                            h_prev, geometry.side, self.dynamic_wheel_y_offset,
                        )
                        hip_rate = (target_hip - prev_hip) / dt
                        knee_rate = (target_knee - prev_knee) / dt
                    else:
                        hip_rate = knee_rate = 0.0
                    for joint_name, target_angle, target_rate in (
                        (geometry.hip_joint, target_hip, hip_rate),
                        (geometry.knee_joint, target_knee, knee_rate),
                    ):
                        act_idx = addresses.actuators[joint_name]
                        qpos_idx = addresses.joint_qpos[joint_name]
                        qvel_idx = addresses.joint_qvel[joint_name]
                        tau = tuck_kp * (target_angle - float(data.qpos[qpos_idx])) \
                            + tuck_kd * (target_rate - float(data.qvel[qvel_idx]))
                        lo, hi = model.actuator_ctrlrange[act_idx]
                        control[act_idx] = float(np.clip(tau, lo, hi))
                self._flight_tuck_h_prev = h_target
            # 空中姿态: 仅 pitch_rate 阻尼 (PD 中不加 K_p 位置项)。
            # τ_motor = -K_d * pitch_rate, 两腿对称同号 → 给 base 反向
            # pitch 力矩。flight_pitch_kd 默认 1.5, clip ±3.5 N·m 是实测稳定点。
            #
            # 为什么不加 K_p:
            # FLIGHT 期间 leg gravity 摆动 + EXTEND 末期 leg 角动量回流给 base 一个
            # 恒定 ~+2.3 N·m 的偏置 pitch 力矩, 让 pitch_rate 在 ~-1.5 rad/s 形成
            # 稳态平衡. 若加位置项 P, 与该偏置形成正反馈环, 实测 pitch 发散到 ±0.6 rad.
            # 因此只用 D 项, 接受残留稳态 pitch_rate (落地由 LAND 阶段吸收)。
            pitch_kd = float(self.params.flight_pitch_kd)
            # 2026-09-13 关键修复：左右髋给**反号**关节力矩。左右髋关节轴镜像
            # （左 -X / 右 +X），同号关节力矩在世界系里互相抵消——旧阻尼器
            # 实际上只摆腿、不俯仰机身（调 flight_pitch_kd 无效的原因）。
            # 反号力矩的世界系合力 = 2×τ，才真正俯仰机身（腿对为配重）。
            attitude_torque = float(np.clip(
                -pitch_kd * float(state.pitch_rate), -6.0, 6.0,
            ))
            for geometry in LEG_CLOSED_LOOP.values():
                act_idx = addresses.actuators[geometry.hip_joint]
                lo, hi = model.actuator_ctrlrange[act_idx]
                # 右髋(+X 轴)取 +τ，左髋(−X 轴)取 −τ：世界系同向叠加
                # （符号经实测确定，取反成正反馈 52.8°）
                sign = 1.0 if geometry.side == "right" else -1.0
                if not bool(self.params.flight_attitude_diff_enable):
                    # 论文对照用：复现镜像修正之前的写法（两侧同号关节力矩 →
                    # 世界系力矩互相抵消，只摆腿、不调机身姿态）。
                    sign = 1.0
                control[act_idx] = float(np.clip(sign * attitude_torque, lo, hi))
            return control

        # 离开 FLIGHT（落地/回到地面相位）后清掉膝锁存与蜷腿差分状态。
        self._flight_knee_hold = None
        self._flight_tuck_h_prev = None
        # 同时清掉"上一拍目标角"：FLIGHT 分支早返回、从不更新它，落地第一拍
        # 拿到的 previous 还是 EXTEND 末尾的目标 (h_high)。落地目标由 setup_land
        # 从实际触地腿高接管，两者相差可达 0.1 m → 关节目标角差 ~0.35 rad，
        # 除以 dt=2 ms 得到 ~180 rad/s 的假目标角速度，kd_land(8)·rate 直接把
        # 触地瞬间的腿电机推到 ±60 N·m 饱和（一次 2 ms 的反作用力矩冲击）。
        # 清空后落地第一拍的 rate FF 为 0，力矩由位置项和动态推力前馈给出。
        if phase_before_update == JumpPhase.FLIGHT:
            self._target_motor_angle_prev = {}
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
                kp_motor *= float(self.params.jump_kp_scale)
                kd_motor *= float(self.params.jump_kd_scale)
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
                kd_motor *= float(self.params.jump_kd_scale)
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
        # 跳跃相位腿动得很快（EXTEND 0.15~0.3 s 内走完整个行程），雅可比每步刷新；
        # STAND 静态保持按固定周期刷新即可。
        fast_leg_motion = phase != JumpPhase.STAND
        if fast_leg_motion or self._motor_jacobian_step_count % self.JACOBIAN_REFRESH_PERIOD == 0:
            self._height_jacobian_rows_cache = _leg_height_jacobian_rows(model, data)
        rows = self._height_jacobian_rows_cache
        assert rows is not None
        use_support_ff = bool(self.params.gravity_ff_enabled and data.ncon >= 2)
        if use_support_ff and self._motor_jacobian_step_count % self.SUPPORT_FF_REFRESH_PERIOD == 0:
            self._support_ff_cache = self._analytic_support_feedforward(
                model, data, rows, total_mass, gravity,
            )
        # 跳跃的"发动机"：把轨迹的期望竖向加速度折进支撑前馈
        #   总推力 F = m·(g + ḧ_target)，每腿分摊 F/N，关节力矩 = (∂h/∂q_j)·F/N
        # （腿连杆自重项同样按 g+ḧ 计）。STAND 时 ḧ=0，与静态前馈逐位相同。
        # 这一项不按接触门控：EXTEND 蹬到最后会瞬间离地，正是最需要推力的时候。
        dynamic_ff: dict[str, float] | None = None
        if self.params.gravity_ff_enabled and phase in (
            JumpPhase.CROUCH, JumpPhase.EXTEND, JumpPhase.LAND,
        ):
            dynamic_ff = self._analytic_support_feedforward(
                model, data, rows, total_mass, gravity, vertical_accel=target_h_ddot,
            )
        self._motor_jacobian_step_count += 1

        neutral_offsets = self._neutral_offsets(model, data)
        roll_level_offsets = self._roll_level_height_offsets(model, data, phase, state)
        common_height = self._slope_squat_height(target_height, roll_level_offsets, phase)

        for side, geometry in LEG_CLOSED_LOOP.items():
            side_target_height = common_height + neutral_offsets[side] + roll_level_offsets[side]
            side_target_height = self.params.ik.clamp_height(side_target_height)
            self.last_target_heights[side] = side_target_height

            target_hip, target_knee = self.params.ik.angles_from_base_height(
                side_target_height, side, self.dynamic_wheel_y_offset,
            )
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

            if phase == JumpPhase.STAND:
                # STAND 的"目标角速度前馈"按 stand_rate_ff_scale 缩放
                # （默认 0 = 旧行为，只保留阻尼项；见 VmcParams 注释）。
                # 上游 STAND 时 target_h_dot=0、但目标角仍随找平偏置变化，
                # 所以 rate = Δtarget_angle/dt 是"找平需要的关节速度"。
                rate_scale = float(self.params.stand_rate_ff_scale)
                target_rates = {
                    geometry.hip_joint: target_rates[geometry.hip_joint] * rate_scale,
                    geometry.knee_joint: target_rates[geometry.knee_joint] * rate_scale,
                }
            else:
                # CROUCH/EXTEND/LAND：见 VmcParams.jump_rate_ff_scale 的说明。
                jump_scale = float(self.params.jump_rate_ff_scale)
                target_rates = {
                    geometry.hip_joint: target_rates[geometry.hip_joint] * jump_scale,
                    geometry.knee_joint: target_rates[geometry.knee_joint] * jump_scale,
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
                if dynamic_ff is not None:
                    # 跳跃相位：带期望加速度的动态推力前馈（CROUCH/EXTEND/LAND）
                    ff_torque = dynamic_ff.get(joint_name, 0.0)
                elif use_support_ff and self._support_ff_cache is not None:
                    # 解析式支撑前馈（整车重量经轮子传导 + 腿连杆自重）
                    ff_torque = self._support_ff_cache.get(joint_name, 0.0)
                elif phase != JumpPhase.FLIGHT and self.params.leg_gravity_ff_enabled:
                    # 腿/轮自身重力的精确关节前馈（固定机身测试）。
                    ff_torque = float(data.qfrc_bias[addresses.joint_qvel[joint_name]])
                control[act_idx] = ff_torque + pid_torque

        clipped = np.clip(control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
        if phase in (JumpPhase.STAND, JumpPhase.CROUCH, JumpPhase.LAND):
            # LAND 用独立的软限幅：缓冲 PD 需要的力矩远大于静态保持（见 VmcParams）。
            limit = float(
                self.params.land_torque_limit if phase == JumpPhase.LAND
                else self.params.stand_torque_limit
            )
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

    def _analytic_support_feedforward(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        rows: dict[str, np.ndarray],
        total_mass: float,
        gravity: float,
        vertical_accel: float = 0.0,
    ) -> dict[str, float]:
        """解析式支撑前馈：撑住整车时每个腿关节需要的静态力矩（N·m）。

            τ_j = (∂h/∂q_j)·(m_total·g)/N_legs + Σ_{i∈腿链} m_i·g·(∂z_i/∂q_j)

        第一项：整车重量经轮子传到地面的那条载荷路径。由虚功原理，
        作用在轮心上的竖直支承力 F = m·g/N 对应的关节力矩就是
        τ_j = F·(∂h/∂q_j)（h = 机身原点 − 轮心高度，腿伸长时 ∂h/∂q > 0）。
        这一项就是开源车 `vmc.py` 里 `jacobian[side] * total_mass * gravity / N`
        的等价形式。

        第二项：腿链（大腿/小腿/轮）自身重量的关节重力矩
        τ_g,j = Σ m_i·g·(∂z_i/∂q_j)。开源车的四连杆腿很轻可以省掉；
        本机腿链 3.5 kg/腿，省掉会在膝上差 5.5 N·m、髋上差 3.6 N·m。
        实机实现时这一项就是标准的"关节重力补偿 τ_g(q)"。

        两项都只用刚体几何（腿高雅可比 + 各连杆质心竖直雅可比），
        换成实机就替换成 IK/正运动学给出的解析雅可比，不含任何仿真器内部量。

        vertical_accel：把 g 换成 (g + vertical_accel)。跳跃时传入轨迹的期望竖向
        加速度 ḧ_target，两项一起放大/缩小 —— 这就是把整机蹬起来的那条推力
        路径（上游对应 `jacobian·m_total·(g+ḧ)/N`，本机多一项腿连杆自重）。
        ḧ=0 时与纯静态支撑前馈逐位相同，所以 STAND 的行为不受影响。
        """
        addresses = model_addresses(model)
        n_legs = len(LEG_CLOSED_LOOP)
        include_leg_weight = bool(self.params.support_ff_include_leg_weight)
        g_eff = gravity + float(vertical_accel)
        jac = np.zeros((3, model.nv))
        feedforward: dict[str, float] = {}
        for side, geometry in LEG_CLOSED_LOOP.items():
            row = rows[side]
            chain = _leg_body_chain(model, geometry.wheel_body) if include_leg_weight else ()
            for joint_name in (geometry.hip_joint, geometry.knee_joint):
                qvel_idx = addresses.joint_qvel[joint_name]
                torque = float(row[qvel_idx]) * total_mass * g_eff / n_legs
                for body_id_ in chain:
                    jac[:] = 0.0
                    mujoco.mj_jac(model, data, jac, None, data.xipos[body_id_], body_id_)
                    torque += float(model.body_mass[body_id_]) * g_eff * float(jac[2, qvel_idx])
                feedforward[joint_name] = torque
        if not all(np.isfinite(value) for value in feedforward.values()):
            raise ValueError("analytic support feedforward must be finite")
        return feedforward


def _leg_body_chain(model: mujoco.MjModel, wheel_body: str) -> list[int]:
    """沿 parent 链从轮 body 回溯到 base_link 的所有腿 link body id（不含 base/world）。

    对应"轮 → 小腿 → 大腿"这条串联链，用于把腿连杆自重算进支撑前馈。
    实机实现时不需要这棵树：直接按腿的连杆参数列 τ_g(q) 即可。
    """
    base = body_id(model, BASE_BODY_NAME)
    chain: list[int] = []
    current = body_id(model, wheel_body)
    while current not in (0, base):
        chain.append(current)
        current = int(model.body_parentid[current])
    return chain


def _flight_tuck_height_profile(t: float, t_flight: float, h_high: float, h_tuck: float) -> float:
    """腾空收腿-展腿高度计划（按滞空时间归一化）。

    τ<0.15 保持蹬直（起跳 settle）；0.15~0.45 smoothstep 收到 h_tuck；
    0.45~0.62 保持蜷缩；0.62~0.92 展回 h_high（落地吸收行程就位）；>0.92 保持。
    """
    if t_flight <= 0.0:
        return h_high
    tau = float(np.clip(t / t_flight, 0.0, 1.0))

    def smooth(x: float) -> float:
        x = float(np.clip(x, 0.0, 1.0))
        return x * x * (3.0 - 2.0 * x)

    if tau < 0.15:
        return h_high
    if tau < 0.45:
        return h_high + (h_tuck - h_high) * smooth((tau - 0.15) / 0.30)
    if tau < 0.62:
        return h_tuck
    if tau < 0.92:
        return h_tuck + (h_high - h_tuck) * smooth((tau - 0.62) / 0.30)
    return h_high


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
