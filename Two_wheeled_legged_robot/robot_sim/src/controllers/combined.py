from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from src.controllers.balance_lqr import LEG_ROLL_DIFF_SIGNS, equilibrium_pitch_from_geometry
from src.controllers.balance_state import balance_tangent_state_5d
from src.controllers.lqr import LqrController
from src.controllers.phase import JumpPhaseMachine, JumpPhase
from src.controllers.vmc import LEG_CLOSED_LOOP, VmcController, VmcParams
from src.geometry import wheel_center_z
from src.model_semantics import MODEL_SEMANTICS, WHEEL_FORWARD_SIGNS
from src.state import SimState, body_id, model_addresses


JUMP_PHASES = (JumpPhase.CROUCH, JumpPhase.EXTEND, JumpPhase.FLIGHT, JumpPhase.LAND)


def _wrap_to_pi(angle: float) -> float:
    """把角度规整到 (-pi, pi], 用于航向误差的最短路径回正。"""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass
class CombinedParams:
    """LQR+VMC 组合控制器参数。

    架构: 相位独占 (phase-authoritative)。LQR 和 VMC 在腿 motor 上的冲突
    (height PD + roll_diff 求和饱和) 通过相位限制 LQR 各通道的输出消除。

    - STAND: LQR 全开 (轮 forward + 腿 roll 差分 + yaw) + VMC 高度 PD。
    - CROUCH/EXTEND/LAND: LQR 轮 forward + yaw 维持本体平衡 (不动腿),
      VMC 轨迹独占腿。LQR 轮 forward 阻止 pitch 在 EXTEND 期间因水平失稳放大,
      但因为不写腿 motor,leg actuator 量程完全留给 VMC 动态 FF。
    - FLIGHT: 全 0。腿 motor = 0 → 空中无 reaction torque,base 姿态由角动量
      保持;轮 = 0 → 不积累轮速,避免落地反扭。
    - FALLEN: 全 0。

    调参顺序 (实机/headless 通用, 详见 default_params.py 顶部说明):
      VMC PD → LQR balance → forward (pitch_lean) → yaw → heading_hold → jump.

    Attributes:
        vmc: VMC 高度控制参数。
        q_diag: LQR 状态权重对角线 [pitch, pitch_rate, roll, roll_rate, wheel_vel]。
            注意:Q[wheel_vel] 不应过大,否则 LQR 与外环 pitch_lean PI 争抢速度
            跟踪权限 (见 default_params.py STAND_PARAMS 注释)。
        r_diag: LQR 虚拟控制权重对角线 [forward_wheel, roll_diff_leg]。
        target_velocity: 目标前进速度 (m/s)。可运行时修改。
        pitch_lean_gain: 速度误差到目标 pitch 偏移的增益 (P)。
        velocity_ki: 速度积分补偿增益 (I)。
        position_kp: 位置外环 P 增益 (1/s)。pos_err [m] → vel_ref 修正 [m/s]。
            仅 target_velocity≈0 时启用。
        position_kd: 位置外环 D 增益。base 水平速度反馈进 vel_ref,
            提供位置-pitch 二阶环的阻尼。
        position_velocity_limit: 位置外环输出 vel_ref 修正幅度限制 (m/s)。
        yaw_damping: yaw 角速度比例阻尼增益 (P)。
        yaw_ki: yaw 角速度积分增益 (I)。
        target_yaw_rate: 目标 yaw 角速度 (rad/s)。
        heading_hold_kp: 航向保持外环 P 增益。航向误差 [rad] → yaw-rate 参考 [rad/s]。
            仅 target_yaw_rate≈0 (未发转向指令) 且站立接地时启用。与位置外环
            (_position_outer_loop) 同构: 锁定当前航向, 把航向误差串级成 yaw-rate
            参考喂进 yaw 阻尼内环, 抵抗外部扰动保持方向不变。0 = 关闭 (默认, 保持旧行为)。
        heading_hold_rate_limit: 航向保持外环输出 yaw-rate 参考的幅度上限 (rad/s),
            防止大航向误差时猛回正干扰平衡。
        fixed_height: True 时 LQR gain 锁定在 _initialize_lqr 那一刻的值,
            后续不再随几何变化重算 (适合 cmd_height 固定的场景, 节省 CPU)。
            False (默认) 动态更新 LQR gain: 静止平地高度变化时沿 height
            轴线性插值；移动或 roll-leveling 场景下只切换当前 (height_bin, roll_bin)
            缓存，避免混合不同地形姿态的线性化结果。
        lqr_height_bin_size: gain 缓存的 height bin 宽度 (m)。同一 bin 内插值
            参数确定。默认 0.02 m, 对应 LUT 操作范围内 ~5 个 bin。
        ff_gain: 高度变化引起的 wheel velocity FF 系数 (默认 0 = 禁用)。
            仅 cmd_height 快速阶跃 pitch peak 不达标时启用; 启用后又把
            height_rate 信号注入 LQR target[4], 跟 pitch_lean 路径有轻微耦合。
    """
    vmc: VmcParams
    q_diag: np.ndarray
    r_diag: np.ndarray
    target_velocity: float = 0.0
    pitch_lean_gain: float = 0.02
    velocity_ki: float = 0.1
    position_kp: float = 1.5
    position_kd: float = 1.5
    position_velocity_limit: float = 0.3
    yaw_damping: float = 0.5
    yaw_ki: float = 0.0
    target_yaw_rate: float = 0.0
    # 航向保持: 默认关闭 (kp=0), 保持旧 yaw-rate 阻尼行为不变。见 Attributes 说明。
    heading_hold_kp: float = 0.0
    heading_hold_rate_limit: float = 1.0
    fixed_height: bool = True
    lqr_height_bin_size: float = 0.02
    ff_gain: float = 0.0
    # 试验台校准的 2 状态轮子 LQR 增益（状态 [pitch, pitch_rate]）。
    # 非 None 时 STAND 的轮子 forward 通道改用 u = -K @ [pitch-eq, pitch_rate]，
    # 替代尚未标定的 5D 简化 LQR wheel 通道（2026-09-09 试验台验证增益
    # K=[-38.2, -6.3]，轮力矩按实机峰值 ±9 N·m 限幅）。
    wheel_balance_gain_2d: np.ndarray | None = None
    # 轮子前向力矩的软件上限（N·m），默认取实机峰值 ±9（见 mjcf_builder 657 行）。
    # 2026-09-11 扫描结论（2 cm 单轮坡 × 8 档速度 0.10~0.45 m/s，通过档数）：
    #   9 → 7/8（仅 0.27 m/s 翻车）; 8 → 6/8; 7 → 4/8; 6 → 4/8; 5 → 1/8。
    # 即"限扭防弹射"并不成立——降限幅反而吃掉低速爬坡能力，保持 9 最优。
    wheel_torque_limit: float = 9.0
    # 内环轮速反馈项（对齐上游 5D LQR 的 K[0,4] 结构；默认关闭）。
    # 上游 forward_torque = ff − K·(x − target)，轮速系数 K[0,4] 作用在
    # (ω − target[4]) 上，本机等效 −1.203 N·m/(m/s)。
    # 2026-09-11 实测结论（扫描 ±12，含 2/4/6cm 对称坡与单轮梯形坡）：
    #   平地停车收敛 0.94/0.84s → -4 时 0.53/0.52s、-6 时 0.17/0.17s，确有改善；
    #   但所有非零取值都会侵蚀坡上余量：-4 时 test_slope_v2 直接翻车（|pitch| 135°），
    #   -0.5/-2/-3 同样翻车，-1/-1.2 虽通过但 |pitch|max 19.9°→26°/38°。
    # 根因：本机 ω 取左右轮平均前向速度，在单轮梯形坡上两轮行程差异很大，该信号
    # 不能代表机身速度，注入平衡环会引入偏置。故默认 0，保留字段供后续标定使用。
    wheel_vel_balance_gain: float = 0.0
    # 指令斜坡限速（2026-09-09）：手柄松开瞬间 target 从 0.5 阶跃到 0，
    # 外环来不及平滑会造成大幅前后摆动。这里限制指令变化率。
    # 2026-09-11 手柄实驾日志（run_20260911_182645）显示平地刹车距离 471~861mm、
    # 平均减速度仅 0.17~0.25 m/s²，松杆后刹不住。实测 0.4→1.2 的效果：
    #   刹车距离 794→524mm（-34%）；单轮坡余量逐位不变（|pitch|max 19.9°）；
    #   10m 直行不变；停车瞬态与停稳时间也逐位不变（轮跳 0.051/0.041 N.m、0.94/0.84s）。
    # 2026-09-11 第二轮（配合 pitch_lean_gain=0.35）：1.2 → 2.5。
    # 该限速只决定"摇杆打到满量程后目标速度多久到位"；1.2 m/s² 让 0→0.8 m/s
    # 要 0.67 s，手感偏钝。提到 2.5 后 0→0.8 m/s 只需 0.32 s，实测 0.5/0.8/1.0 m/s
    # 三档速度跟踪 rms 均 ≤0.019、松杆刹车 0.77~1.00 s，单轮坡 |pitch|max
    # 0.146→0.074，无回落。
    max_linear_accel: float = 2.5    # m/s²
    # 转向同样偏钝：0.15 rad/s² 下 0→0.5 rad/s 要 3.3 s（实测验收到 90% 用时 3.23 s）。
    # 提到 1.0 后只需 1.2 s，且稳态速差与峰值几乎不变（峰值 0.506 vs 目标 0.5）。
    max_yaw_accel: float = 1.0       # rad/s²
    # 停车时把位置保持和速度积分释放成连续过渡，避免外环接管造成顿挫。
    position_hold_blend_tau: float = 0.2       # s
    velocity_integral_release_tau: float = 0.25  # s
    velocity_integral_release_speed: float = 0.10  # m/s
    position_hold_velocity_gate: float = 0.03   # m/s
    # 高位限速联动保护（2026-09-09 手柄实测失稳后添加）：
    # 高站姿 > high_height_threshold 时动态余量小，限制速度/转向上限，
    # 防止"0.48m/s + 0.50m 高站"这类危险组合。
    high_height_threshold: float = 0.45
    high_height_velocity_limit: float = 0.3
    high_height_yaw_limit: float = 0.2
    # 轮速阻尼（2026-09-09 高位极限环调试）：2 状态 LQR 不含 wheel_vel 状态，
    # 轮子会来回转造成机身晃动；这里直接对轮子前向速度加阻尼力矩。
    wheel_vel_damping: float = 0.0
    # 转向通道总输出上限（N·m）。yaw_damping=8 在"机身被外力偏航"时能瞬间输出
    # 8·Δω 的差动力矩；站立 / 单轮垫高这类场景里 Δω 可以很大，输出会顶到轮子
    # ±9 N·m 峰值，把本该留给平衡通道的量程吃光（实测单轮垫高场景 pitch 因此
    # 从 0.10 rad 抖到 0.37 rad、单轮离地 42% 的步数）。加这道硬上限后转向只
    # 借用有限的轮力矩，平衡永远留有余量。0 = 关闭（恢复旧的无限幅行为）。
    # 2.0 N·m 的依据：正常行驶转向只需 ~0.8 N·m（见 yaw_ki 注释）；0.6 rad/s
    # 档和 0.8 m/s + 0.6 rad/s 复合在 1.0/2.0/4.0 三档限幅下曲线完全一致；
    # 给机身一个 15 N·m·0.1 s 的偏航冲量，2.0 与无限幅的航向回正曲线也一致
    # （25 N·m 冲量下峰值 0.19 vs 0.07 rad，仍能回正）。
    yaw_correction_limit: float = 2.0
    # yaw 积分上限（|积分| 的绝对值上限）。另有"积分只能占用输出上限里
    # 比例项没用完的余量"的条件积分（见 _compute_yaw_correction），两者取小。
    # 实测正常转向稳态积分仅 ~0.14。
    yaw_integral_limit: float = 1.0


class CombinedController:
    """相位独占控制器: STAND 用 LQR+VMC,跳跃全程 VMC 独占。"""

    def __init__(self, params: CombinedParams, phase_machine: JumpPhaseMachine | None = None) -> None:
        self.params = params
        self.vmc_controller = VmcController(params.vmc, phase_machine)
        self._lqr_controller: LqrController | None = None
        self._velocity_integral: float = 0.0
        self._yaw_integral: float = 0.0
        # Gain 表键 = (height_bin, roll_bin)。每个 roll_bin 第一次出现时一次性填满
        # height 轴，避免 cmd_height 扫描时在控制循环里反复求解 DARE。
        self._lqr_gain_cache: dict[tuple[int, int], np.ndarray] = {}
        self._wheel_ff_gain_cache: dict[tuple[int, int], float] = {}
        self._prewarmed_roll_bins: set[int] = set()
        self._active_wheel_ff_gain: float = 0.0
        self._height_wheel_velocity_ff: float = 0.0
        self._last_phase = JumpPhase.STAND
        self._position_anchor: np.ndarray | None = None
        self._heading_anchor: float | None = None
        self._equilibrium_pitch: float = 0.0
        self._last_cmd_velocity: float | None = None
        self._last_cmd_yaw_rate: float | None = None
        # 外部速度指令回中时，禁止上一段行驶留下的速度积分继续累积，
        # 但要按时间常数释放，避免停车瞬间改变平衡点。
        self._zero_velocity_request = False
        # 上面那个布尔量的连续渐入版本 (0→1, 时间常数 = velocity_integral_release_tau)。
        # 见 _update_lqr_target 里的说明: 直接用布尔量会在"外部指令刚跨过 0"那一
        # 个周期把积分门控从 1 跳到 smoothstep(|v|/release_speed)，在目标倾角上
        # 产生一次阶跃 (K=45 的轮 LQR 会把它放大成 45·Δθ 的轮力矩跳变)。
        self._zero_request_blend: float = 0.0
        self._position_hold_blend: float = 0.0
        self._slope_pitch_bias: float = 0.0
        self._slope_bias_tau = 0.30  # 爬坡参考跟随时间常数 (s)

    def _smooth_command_targets(self, dt: float) -> None:
        """把外部写入的 target_velocity / target_yaw_rate 做斜坡限速。

        手柄摇杆从满量程突然回中 => 期望值阶跃到 0，直接喂给外环会前后猛摆；
        这里按 max accel 限幅，控制器内部使用平滑后的值。
        """
        desired_v = float(self.params.target_velocity)
        self._zero_velocity_request = abs(desired_v) <= 1e-6
        # 把"是否在停车"这个开关也按时间常数平滑成连续量。积分门控/释放全部用它，
        # 于是外部指令跨越零点的那一步不会在控制量里留下台阶。
        zero_blend_tau = max(float(self.params.velocity_integral_release_tau), 1e-6)
        zero_blend_alpha = 1.0 - float(np.exp(-dt / zero_blend_tau))
        self._zero_request_blend += zero_blend_alpha * (
            (1.0 if self._zero_velocity_request else 0.0) - self._zero_request_blend
        )
        self._zero_request_blend = float(np.clip(self._zero_request_blend, 0.0, 1.0))
        current_h = float(self.params.vmc.nominal_height)
        if current_h >= self.params.high_height_threshold:
            v_lim = float(self.params.high_height_velocity_limit)
            yaw_lim = float(self.params.high_height_yaw_limit)
            desired_v = float(np.clip(desired_v, -v_lim, v_lim))
            self.params.target_yaw_rate = float(
                np.clip(self.params.target_yaw_rate, -yaw_lim, yaw_lim)
            )
        if self._last_cmd_velocity is None:
            self._last_cmd_velocity = desired_v
        else:
            step = max(float(self.params.max_linear_accel), 0.0) * dt
            self._last_cmd_velocity += float(
                np.clip(desired_v - self._last_cmd_velocity, -step, step)
            )
        self.params.target_velocity = self._last_cmd_velocity
        # 位置保持只在停车锚点已经锁定后渐入。锚点由
        # _handle_stand_entry 在实际速度接近零时锁定，避免停车过程中的
        # 位置误差被误认为需要反向修正。
        blend_tau = max(float(self.params.position_hold_blend_tau), 1e-6)
        blend_alpha = 1.0 - float(np.exp(-dt / blend_tau))
        # 内部速度目标尚在斜坡减速时，速度环仍在主动制动；位置环此时
        # 不应提前叠加一个第二套制动指令。等内部目标真正到零后再渐入。
        blend_target = 1.0 if (
            self._zero_velocity_request
            and abs(self.params.target_velocity) <= 1e-6
            and self._position_anchor is not None
        ) else 0.0
        if self._position_anchor is None:
            # 锚点尚未建立时，位置环完全不参与，不能提前积累渐入权重。
            self._position_hold_blend = 0.0
        else:
            self._position_hold_blend += blend_alpha * (blend_target - self._position_hold_blend)
        self._position_hold_blend = float(np.clip(self._position_hold_blend, 0.0, 1.0))

        desired_yaw = float(self.params.target_yaw_rate)
        if self._last_cmd_yaw_rate is None:
            self._last_cmd_yaw_rate = desired_yaw
        else:
            step = max(float(self.params.max_yaw_accel), 0.0) * dt
            self._last_cmd_yaw_rate += float(
                np.clip(desired_yaw - self._last_cmd_yaw_rate, -step, step)
            )
        self.params.target_yaw_rate = self._last_cmd_yaw_rate

    @property
    def lqr_controller(self) -> LqrController | None:
        return self._lqr_controller

    # ---------- LQR gain management ----------

    def _bin_size(self) -> float:
        return max(float(self.params.lqr_height_bin_size), 1e-6)

    def _roll_bin(self, state: SimState) -> int:
        return int(np.floor(float(state.roll) / 0.02))

    def _height_bin(self, state: SimState) -> int:
        return int(np.floor(float(state.base_position[2]) / self._bin_size()))

    def _height_table_bins(self) -> range:
        ik = self.params.vmc.ik
        bin_size = self._bin_size()
        lo = int(np.floor(float(ik.h_min) / bin_size)) - 2
        hi = int(np.ceil(float(ik.h_max) / bin_size)) + 2
        return range(lo, hi + 1)

    def _prewarm_lqr_height_table(self, model: Any, data: Any, h_bin: int, r_bin: int) -> None:
        if r_bin in self._prewarmed_roll_bins:
            return
        gain = self._gain_for_bin(model, data, h_bin, r_bin)
        ff = self._wheel_ff_for_bin(model, data, h_bin, r_bin)
        for table_h_bin in self._height_table_bins():
            self._lqr_gain_cache.setdefault((table_h_bin, r_bin), gain)
            self._wheel_ff_gain_cache.setdefault((table_h_bin, r_bin), ff)
        self._prewarmed_roll_bins.add(r_bin)

    def _gain_for_bin(
        self, model: Any, data: Any, h_bin: int, r_bin: int,
    ) -> np.ndarray:
        cached = self._lqr_gain_cache.get((h_bin, r_bin))
        if cached is not None:
            return cached
        from src.controllers.balance_lqr import compute_balance_lqr_gain_5d
        gain = compute_balance_lqr_gain_5d(
            model, data, self.params.q_diag, self.params.r_diag,
        )
        self._lqr_gain_cache[(h_bin, r_bin)] = gain
        return gain

    def _wheel_ff_for_bin(
        self, model: Any, data: Any, h_bin: int, r_bin: int,
    ) -> float:
        cached = self._wheel_ff_gain_cache.get((h_bin, r_bin))
        if cached is not None:
            return cached
        from src.controllers.balance_lqr import wheel_ff_gain_for_leg_common
        ff = wheel_ff_gain_for_leg_common(model, data)
        self._wheel_ff_gain_cache[(h_bin, r_bin)] = ff
        return ff

    def _interpolated_lqr_inputs(
        self, model: Any, data: Any, state: SimState,
    ) -> tuple[np.ndarray, float]:
        h_bin = self._height_bin(state)
        r_bin = self._roll_bin(state)
        if not self._should_interpolate_height_gain(state):
            return (
                self._gain_for_bin(model, data, h_bin, r_bin),
                self._wheel_ff_for_bin(model, data, h_bin, r_bin),
            )

        self._prewarm_lqr_height_table(model, data, h_bin, r_bin)
        h = float(state.base_position[2])
        center_position = h / self._bin_size() - 0.5
        h_lo = int(np.floor(center_position))
        h_hi = h_lo + 1
        alpha = float(np.clip(center_position - h_lo, 0.0, 1.0))

        gain_lo = self._gain_for_bin(model, data, h_lo, r_bin)
        gain_hi = self._gain_for_bin(model, data, h_hi, r_bin)
        ff_lo = self._wheel_ff_for_bin(model, data, h_lo, r_bin)
        ff_hi = self._wheel_ff_for_bin(model, data, h_hi, r_bin)
        return (
            (1.0 - alpha) * gain_lo + alpha * gain_hi,
            (1.0 - alpha) * ff_lo + alpha * ff_hi,
        )

    def _should_interpolate_height_gain(self, state: SimState) -> bool:
        if self.params.fixed_height:
            return False
        if abs(float(self.params.target_velocity)) > 1e-6:
            return False
        if abs(float(state.roll)) >= 0.015 or abs(float(state.roll_rate)) >= 0.2:
            return False
        vmc = self.params.vmc
        return not (
            abs(float(vmc.roll_level_kp_height)) > 1e-12
            or abs(float(vmc.roll_level_kd_height)) > 1e-12
            or abs(float(vmc.roll_level_offset_limit)) > 1e-12
        )

    def _initialize_lqr(self, model: Any, data: Any, state: SimState) -> None:
        if not np.isfinite(self.params.target_velocity):
            raise ValueError("target_velocity must be finite")
        gain, ff_gain = self._interpolated_lqr_inputs(model, data, state)
        self._active_wheel_ff_gain = ff_gain
        self._equilibrium_pitch = equilibrium_pitch_from_geometry(model, data)
        target = balance_tangent_state_5d(model, data, state).copy()
        target[0] = self._equilibrium_pitch
        target[1] = 0.0
        target[2] = 0.0
        target[3] = 0.0
        target[4] = self.params.target_velocity
        self._lqr_controller = LqrController(gain, target, np.zeros(2), balance_tangent_state_5d)

    def _ensure_lqr_height_bin(self, model: Any, data: Any, state: SimState) -> None:
        if self._lqr_controller is None:
            return
        gain, ff_gain = self._interpolated_lqr_inputs(model, data, state)
        self._lqr_controller.gain = gain
        self._active_wheel_ff_gain = ff_gain

    # ---------- Height feedforward ----------

    def _average_leg_height_and_wheel_mid_z(self, model: Any, data: Any) -> tuple[float, float]:
        base_id = body_id(model, "base_link")
        wheel_mid_z = float(np.mean([
            wheel_center_z(model, data, geometry.wheel_body)
            for geometry in LEG_CLOSED_LOOP.values()
        ]))
        # 与 vmc._leg_height 保持一致：腿高用机身原点 xpos（xipos 会含 ipos z 偏移）
        return float(data.xpos[base_id, 2] - wheel_mid_z), wheel_mid_z

    def _height_wheel_velocity_feedforward(self, current_leg_height: float) -> float:
        if self.params.fixed_height or abs(float(self.params.ff_gain)) < 1e-12:
            return 0.0
        # 串行腿暂无标量 dh/dθ 与 dy_wheel/dh 曲线；ff_gain 默认 0 已提前返回。
        # 若将来启用该前馈，需要按串行腿两关节重做映射（TODO 阶段5）。
        return 0.0

    # ---------- Position / velocity outer loops ----------

    def _position_outer_loop(self, model: Any, data: Any, state: SimState) -> float:
        """位置 PD 外环。速度指令回零后按 blend 权重渐入。"""
        if self._position_anchor is None:
            return 0.0
        if self._position_hold_blend <= 1e-9:
            return 0.0
        base_id = body_id(model, "base_link")
        rotation = np.asarray(data.xmat[base_id]).reshape(3, 3)
        forward_horiz = rotation[:2, 1]
        forward_norm = float(np.linalg.norm(forward_horiz))
        if forward_norm < 1e-6:
            return 0.0
        forward_horiz = forward_horiz / forward_norm
        delta = self._position_anchor - state.base_position[:2]
        forward_error = float(np.dot(delta, forward_horiz))
        forward_velocity = float(np.dot(state.base_linear_velocity[:2], forward_horiz))
        vel_correction = self.params.position_kp * forward_error - self.params.position_kd * forward_velocity
        limit = max(float(self.params.position_velocity_limit), 0.0)
        return self._position_hold_blend * float(np.clip(vel_correction, -limit, limit))

    def _base_heading(self, model: Any, data: Any) -> float:
        """本体前向 (+Y) 在世界 XY 平面投影的航向角 (rad)。

        与 _position_outer_loop 的 forward 定义一致 (rotation[:,1] = 本体 Y 轴)。
        投影到水平面后取 atan2, 对俯仰倾角不敏感 (pitch 只改变投影长度不改方位角)。
        """
        base_id = body_id(model, "base_link")
        rotation = np.asarray(data.xmat[base_id]).reshape(3, 3)
        forward_horiz = rotation[:2, 1]
        if float(np.linalg.norm(forward_horiz)) < 1e-6:
            return 0.0
        return float(np.arctan2(forward_horiz[1], forward_horiz[0]))

    def _heading_outer_loop(self, model: Any, data: Any) -> float:
        """航向保持 P 外环。返回 yaw-rate 参考 (rad/s), 串级进 _compute_yaw_correction。

        仅在: 有 anchor + 未发转向指令 (target_yaw_rate≈0) + heading_hold_kp>0 时输出。
        error = wrap(anchor - heading); rate_ref = clip(kp*error, ±rate_limit)。
        sign: 实际 yaw_rate>0 (本体绕 +Z, 上升航向角) 会减小 error, 故 rate_ref 与
        error 同号即构成稳定回正 (内环把 yaw_rate 拉向 rate_ref)。仅 STAND 接地时调用。
        """
        if self._heading_anchor is None:
            return 0.0
        if abs(self.params.target_yaw_rate) > 1e-6:
            return 0.0
        kp = float(self.params.heading_hold_kp)
        if kp <= 0.0:
            return 0.0
        error = _wrap_to_pi(self._heading_anchor - self._base_heading(model, data))
        limit = max(float(self.params.heading_hold_rate_limit), 0.0)
        return float(np.clip(kp * error, -limit, limit))

    def _update_lqr_target(self, model: Any, data: Any, state: SimState, dt: float) -> None:
        """STAND 阶段的 LQR 目标更新。equilibrium_pitch 跟随当前几何重算。

        5D state: [pitch, pitch_rate, roll, roll_rate, wheel_vel]。
        target[0] = equilibrium_pitch + pitch_lean (速度 PI 输出);
        target[4] = target_velocity + position_outer_loop (位置 P 输出) + height_ff。
        """
        if self._lqr_controller is None:
            return
        # 速度反馈统一用"机身实际前向速度"而不是轮速（2026-09-09 斜坡根因）：
        # 轮速在爬坡/打滑时会虚高，会让外环误判"超速"而倒拉轮子。
        current_wheel_vel = self._project_forward_body_velocity(model, data, state)

        self._equilibrium_pitch = equilibrium_pitch_from_geometry(model, data)
        position_vel_correction = self._position_outer_loop(model, data, state)

        velocity_target = self.params.target_velocity + self._height_wheel_velocity_ff + position_vel_correction
        velocity_error = velocity_target - current_wheel_vel

        # 停车时释放前进/后退阶段累积的速度误差，让位置外环逐步处理残余运动。
        # 外部回零与内部 target_velocity 的斜坡是两个不同事件，不能瞬时清积分。
        max_lean = 0.2
        pitch_p = self.params.pitch_lean_gain * velocity_error
        release_tau = max(float(self.params.velocity_integral_release_tau), 1e-6)
        # 行驶阶段累积的偏置需要释放，但不能在停车边沿瞬时清零，否则 LQR 目标倾角
        # 和轮力矩会同时发生阶跃。用 _zero_request_blend (0→1 连续渐入) 调制：停车
        # 那一刻它仍为 0，控制量连续；随后按时间常数渐入，并以实际速度做 C1 平滑
        # 门控，零速处积分输出也连续归零，高速制动阶段仍保留原有积分制动力。
        release_speed = max(float(self.params.velocity_integral_release_speed), 1e-6)
        speed_ratio = float(np.clip(abs(current_wheel_vel) / release_speed, 0.0, 1.0))
        speed_gate = speed_ratio * speed_ratio * (3.0 - 2.0 * speed_ratio)
        release = float(self._zero_request_blend)
        integral_blend = 1.0 - release * (1.0 - speed_gate)
        # 积分本体也按同一个连续量释放（原来是在布尔开关上直接指数衰减）。
        if release > 1e-9:
            self._velocity_integral *= float(np.exp(-dt / release_tau * release))
        pitch_i = self.params.velocity_ki * self._velocity_integral * integral_blend
        pitch_lean = pitch_p + pitch_i
        # Anti-windup: 只在停车释放尚未接管、且 pitch_lean 未饱和时累积。
        if release < 1.0 - 1e-9 and -max_lean < pitch_lean < max_lean:
            self._velocity_integral += velocity_error * dt
        pitch_lean = float(np.clip(pitch_lean, -max_lean, max_lean))

        self._lqr_controller.target[0] = self._equilibrium_pitch + pitch_lean
        self._lqr_controller.target[1] = 0.0
        self._lqr_controller.target[2] = 0.0
        self._lqr_controller.target[3] = 0.0
        self._lqr_controller.target[4] = velocity_target

    def _project_forward_body_velocity(self, model: Any, data: Any, state: SimState) -> float:
        """机身实际前向速度（本体 +Y 在水平面的投影 × 机身水平速度）。"""
        base_id = body_id(model, "base_link")
        rotation = np.asarray(data.xmat[base_id]).reshape(3, 3)
        forward_horiz = rotation[:2, 1]
        norm = float(np.linalg.norm(forward_horiz))
        if norm < 1e-6:
            return 0.0
        forward_horiz = forward_horiz / norm
        return float(np.dot(state.base_linear_velocity[:2], forward_horiz))

    def _update_slope_pitch_bias(self, model: Any, data: Any, state: SimState, dt: float) -> None:
        """爬坡自适应倾角参考。

        检测：两轮 z 差 >8mm 且机身前向速度 >0.03m/s → 判定正在上坡。
        上坡时地面把车身姿态顶起来（pitch 偏离平地平衡角），LQR 若立刻把
        它当成"失稳"就会倒拉轮子 → 坡沿打滑。这里让平衡参考以一个时间常数
        缓慢跟随地形引起的姿态偏移，出坡后平滑衰减回 0。
        """
        z_left = wheel_center_z(model, data, LEG_CLOSED_LOOP["left"].wheel_body)
        z_right = wheel_center_z(model, data, LEG_CLOSED_LOOP["right"].wheel_body)
        fwd_v = self._project_forward_body_velocity(model, data, state)
        climbing = abs(z_left - z_right) > 0.008 and fwd_v > 0.03
        if climbing:
            eq = float(self._equilibrium_pitch)
            target_bias = float(state.pitch) - eq
            alpha = dt / max(self._slope_bias_tau, 1e-6)
            self._slope_pitch_bias += alpha * (target_bias - self._slope_pitch_bias)
            self._slope_pitch_bias = float(np.clip(self._slope_pitch_bias, -0.12, 0.12))
        else:
            # 指数衰减回 0
            self._slope_pitch_bias *= max(0.0, 1.0 - dt / max(self._slope_bias_tau, 1e-6))

    def _compute_yaw_correction(self, state: SimState, dt: float, heading_rate_ref: float = 0.0) -> float:
        """yaw 角速度阻尼内环。effective_target = target_yaw_rate + 航向保持外环参考。

        航向保持把航向角误差转成 yaw-rate 参考 (heading_rate_ref) 串级进来; 不发转向
        指令时它驱动本环把实际 yaw_rate 拉向"回正所需角速度", 从而把航向拉回 anchor。
        heading_rate_ref=0 时退化为原始 yaw-rate 阻尼 (跳跃相位即走此路径)。
        """
        yaw_rate = float(state.base_angular_velocity[2])
        effective_target_yaw_rate = self.params.target_yaw_rate + heading_rate_ref
        yaw_error = yaw_rate - effective_target_yaw_rate
        yaw_damping = float(self.params.yaw_damping)
        yaw_ki = float(self.params.yaw_ki)
        limit = float(self.params.yaw_correction_limit)
        damping_term = yaw_damping * yaw_error

        # 条件积分抗饱和（两个约束取小）：
        #   1) 绝对值上限 yaw_integral_limit；
        #   2) 输出余量 = (总输出上限 − |比例项|) / yaw_ki。
        # "被顶住转不动"（轮子卡住、贴墙、外力强扭机身）时积分不会风紧成一个大
        # 偏置，既不会松开后久久吐不干净，也不会让转向通道吃掉平衡用的轮力矩。
        absolute_limit = max(float(self.params.yaw_integral_limit), 0.0)
        if limit > 0.0 and yaw_ki > 1e-12:
            headroom_limit = max(limit - abs(damping_term), 0.0) / yaw_ki
            integral_limit = min(absolute_limit, headroom_limit)
        else:
            integral_limit = absolute_limit
        candidate = self._yaw_integral + yaw_error * dt
        if integral_limit > 0.0:
            # 只在"继续往外涨"时冻结；往范围内收的方向照常累积。
            if abs(candidate) > integral_limit and abs(candidate) > abs(self._yaw_integral):
                candidate = self._yaw_integral
            candidate = float(np.clip(candidate, -integral_limit, integral_limit))
        else:
            candidate = 0.0
        self._yaw_integral = candidate

        correction = damping_term + yaw_ki * self._yaw_integral
        if limit > 0.0:
            correction = float(np.clip(correction, -limit, limit))
        return correction

    # ---------- Control allocation ----------

    def _allocate_balance_control(
        self,
        model: Any,
        forward_torque: float,
        roll_torque: float,
        yaw_torque: float,
        addresses: Any,
    ) -> np.ndarray:
        """STAND 模式: 把 LQR 虚拟力矩 + yaw 修正分配到物理轮/腿执行器。"""
        control = np.zeros(model.nu)
        virtual_wheel_torques = (forward_torque - yaw_torque, forward_torque + yaw_torque)
        for joint_name, virtual_torque in zip(MODEL_SEMANTICS.wheel_joints, virtual_wheel_torques):
            actuator_index = addresses.actuators[joint_name]
            control[actuator_index] = WHEEL_FORWARD_SIGNS[joint_name] * virtual_torque
        for joint_name, sign in LEG_ROLL_DIFF_SIGNS.items():
            control[addresses.actuators[joint_name]] = sign * roll_torque
        return control

    def _merge_vmc_and_clip(
        self,
        model: Any,
        control: np.ndarray,
        vmc_control: np.ndarray,
        addresses: Any,
        phase: JumpPhase,
    ) -> np.ndarray:
        """合并 VMC 腿力矩 (共模高度控制) 到 control,然后按相位 clip。

        只 clip 物理执行器 (轮 + 腿 motor),不动 cmd_* slider 这些非物理 actuator
        (它们由 launch_mujoco 在 step 前后单独 read/restore)。
        """
        for joint_name in MODEL_SEMANTICS.leg_motor_joints:
            actuator_index = addresses.actuators[joint_name]
            control[actuator_index] += vmc_control[actuator_index]

        clipped = control.copy()
        physical_joints = MODEL_SEMANTICS.wheel_joints + MODEL_SEMANTICS.leg_motor_joints
        for joint_name in physical_joints:
            act_idx = addresses.actuators[joint_name]
            clipped[act_idx] = float(np.clip(
                control[act_idx],
                model.actuator_ctrlrange[act_idx, 0],
                model.actuator_ctrlrange[act_idx, 1],
            ))
        if phase in (JumpPhase.STAND, JumpPhase.CROUCH, JumpPhase.LAND):
            limit = float(self.params.vmc.stand_torque_limit)
            for joint_name in MODEL_SEMANTICS.leg_motor_joints:
                act_idx = addresses.actuators[joint_name]
                clipped[act_idx] = float(np.clip(clipped[act_idx], -limit, limit))
        # EXTEND: 允许动态 FF 拉满到 actuator 极限 (±12.5 N·m) 推起跳。
        # FLIGHT: VMC 早返回 0,本路径在 _jump_control 下也只会得到 0 + 0 = 0。

        if not np.all(np.isfinite(clipped)):
            raise ValueError("combined control must be finite")
        return clipped

    # ---------- State management ----------

    def _reset_balance_state(self) -> None:
        self._velocity_integral = 0.0
        self._yaw_integral = 0.0
        self._height_wheel_velocity_ff = 0.0
        self._lqr_controller = None
        self._position_anchor = None
        self._heading_anchor = None
        self._last_cmd_velocity = None
        self._last_cmd_yaw_rate = None
        self._zero_velocity_request = False
        self._zero_request_blend = 0.0
        self._position_hold_blend = 0.0
        self._slope_pitch_bias = 0.0

    def _handle_stand_entry(self, model: Any, data: Any, state: SimState) -> None:
        """STAND 进入瞬间: 清积分,锁位置 anchor + 航向 anchor。"""
        if self._last_phase != JumpPhase.STAND:
            self._velocity_integral = 0.0
            self._position_anchor = np.array(state.base_position[:2], dtype=float)
            self._heading_anchor = self._base_heading(model, data)
        zero_target = self._zero_velocity_request and abs(self.params.target_velocity) <= 1e-6
        if zero_target:
            velocity_gate = max(float(self.params.position_hold_velocity_gate), 0.0)
            actual_velocity = abs(self._project_forward_body_velocity(model, data, state))
            if self._position_anchor is None and actual_velocity <= velocity_gate:
                # 先完成减速，再把当前位置作为停车点；这样位置环从零位置误差接管。
                self._position_anchor = np.array(state.base_position[:2], dtype=float)
                # 该状态可能已经在等待低速期间积累了 blend，锁定新锚点时
                # 必须从零开始渐入，否则会在同一周期突然启用位置环。
                self._position_hold_blend = 0.0
        else:
            self._position_anchor = None
        # 航向 anchor 与位置 anchor 独立: 发转向指令时丢弃, 松开 (≈0) 时重新锁定当前航向。
        # 注意只看 target_yaw_rate, 与 target_velocity 无关 — 直线行驶 (有速度无转向)
        # 时仍保持航向, 抵抗偏航漂移。
        if abs(self.params.target_yaw_rate) > 1e-6:
            self._heading_anchor = None
        elif self._heading_anchor is None:
            self._heading_anchor = self._base_heading(model, data)

    def _handle_jump_entry(self) -> None:
        """跳跃序列开始: 清积分,丢 anchor。LQR controller 本身保留 (gain 只依赖几何)。"""
        if self._last_phase not in JUMP_PHASES:
            self._velocity_integral = 0.0
            self._yaw_integral = 0.0
            self._position_anchor = None
            self._heading_anchor = None
        self._height_wheel_velocity_ff = 0.0

    # ---------- Phase-dispatched control ----------

    def _stand_control(
        self,
        model: Any,
        data: Any,
        state: SimState,
        dt: float,
        vmc_control: np.ndarray,
        addresses: Any,
    ) -> np.ndarray:
        """STAND: LQR 全开 (轮 forward + 腿 roll 差分 + yaw) + VMC 高度 PD。"""
        if self._lqr_controller is None:
            self._initialize_lqr(model, data, state)
        if self._lqr_controller is None:
            raise ValueError("failed to initialize LQR controller")
        if not self.params.fixed_height:
            self._ensure_lqr_height_bin(model, data, state)

        current_leg_height, _ = self._average_leg_height_and_wheel_mid_z(model, data)
        self._height_wheel_velocity_ff = self._height_wheel_velocity_feedforward(current_leg_height)

        self._update_lqr_target(model, data, state, dt)
        # Feedforward 抵消 VMC 共模 leg torque 引起的 pitch 扰动:
        # VMC 在 leg motor 上输出 τ_L, τ_R (vmc_control), 共模 = (τ_L + τ_R)/2.
        # 这个 common torque 通过 hip motor 反作用产生 pitch_rate 扰动
        # (实测系数 ≈ -0.15 rad/s² per N·m·leg, 见 _leg_common_to_pitch_coupling).
        # 用 wheel 提前抵消, LQR 反馈只处理残差。
        leg_motor_names = MODEL_SEMANTICS.leg_motor_joints
        leg_torques = [float(vmc_control[addresses.actuators[name]]) for name in leg_motor_names]
        vmc_common_leg = float(np.mean(leg_torques))
        wheel_ff = self._active_wheel_ff_gain * vmc_common_leg
        self._lqr_controller.feedforward = np.array([wheel_ff, 0.0])
        lqr_control = self._lqr_controller(model, data, state)
        if lqr_control.shape != (2,):
            raise ValueError("LQR virtual control must have shape (2,)")
        if state.contact_count >= len(LEG_CLOSED_LOOP):
            if self.params.wheel_balance_gain_2d is not None:
                # 覆盖通道：用试验台标定的 2 状态轮 LQR，但跟踪的是
                # _update_lqr_target 算出的目标倾角（= 平衡角 + 速度 PI/位置外环
                # 的 pitch_lean）。只锁平衡角会让机器人匀速/加速前冲，必须把
                # 位置/速度外环接回来。
                target_pitch = float(self._lqr_controller.target[0])
                self._update_slope_pitch_bias(model, data, state, dt)
                target_pitch += self._slope_pitch_bias
                balance_err = [
                    float(state.pitch) - target_pitch,
                    float(state.pitch_rate),
                ]
                gains = np.asarray(self.params.wheel_balance_gain_2d, dtype=float)
                if float(self.params.wheel_vel_balance_gain) != 0.0:
                    # 对齐上游：增益向量第三项作用于轮速状态误差
                    # (前向轮速 − target[4])，与 pitch/pitch_rate 两项同源同符号约定。
                    wheel_vel = float(balance_tangent_state_5d(model, data, state)[4])
                    balance_err.append(
                        wheel_vel - float(self._lqr_controller.target[4])
                    )
                    gains = np.concatenate(
                        [gains, [float(self.params.wheel_vel_balance_gain)]]
                    )
                forward_torque = float(
                    np.clip(
                        -(gains @ np.asarray(balance_err)),
                        -self.params.wheel_torque_limit,
                        self.params.wheel_torque_limit,
                    )
                )
                if self.params.wheel_vel_damping > 0.0:
                    # 物理前向轮速（与 balance_state 同约定）
                    wf = 0.5 * (
                        state.wheel_velocities["left"] - state.wheel_velocities["right"]
                    ) * 0.07
                    forward_torque -= float(self.params.wheel_vel_damping) * wf
                    forward_torque = float(np.clip(
                        forward_torque,
                        -self.params.wheel_torque_limit,
                        self.params.wheel_torque_limit,
                    ))
                # 转向/航向保持外环（与 LQR 路径一致）：无转向指令时锁航向，
                # 有 target_yaw_rate 时跟踪转向角速度。
                heading_rate_ref = self._heading_outer_loop(model, data)
                # 实测（2026-09-09）：左右轮轴相反 → 纯转向在执行器坐标中同号。
                # _compute_yaw_correction 的符号按开源同向轮轴约定，这里取反。
                yaw_correction = -self._compute_yaw_correction(state, dt, heading_rate_ref)
            else:
                forward_torque = float(lqr_control[0])
                heading_rate_ref = self._heading_outer_loop(model, data)
                yaw_correction = self._compute_yaw_correction(state, dt, heading_rate_ref)
        else:
            forward_torque = 0.0
            yaw_correction = 0.0
        # Roll leveling is handled by VMC as differential leg-height targets.
        # Do not also write LQR roll torque into the same leg actuators.
        roll_torque = 0.0

        if self.params.wheel_balance_gain_2d is not None:
            # 覆盖通道的自定义轮子分配：
            #   前进：ctrlL=+fwd, ctrlR=-fwd（已实测）
            #   转向：左右同号 ctrlL=ctrlR=-yaw_corr（左右轮轴相反，实测映射）
            left_wheel = MODEL_SEMANTICS.wheel_joints[0]   # 左轮 link_007
            right_wheel = MODEL_SEMANTICS.wheel_joints[1]  # 右轮 link_004
            control = np.zeros(model.nu)
            control[addresses.actuators[left_wheel]] = forward_torque - yaw_correction
            control[addresses.actuators[right_wheel]] = -forward_torque - yaw_correction
        else:
            control = self._allocate_balance_control(model, forward_torque, roll_torque, yaw_correction, addresses)
        return self._merge_vmc_and_clip(model, control, vmc_control, addresses, JumpPhase.STAND)

    def _set_lqr_target_balance_only(self, model: Any, data: Any) -> None:
        """跳跃期间的 LQR 目标: 纯 equilibrium_pitch,wheel_vel=0,无 pitch_lean。

        STAND 的 _update_lqr_target 包含 pitch_lean (从 velocity_error 推出),
        用于跟踪 target_velocity > 0 时身体前倾。但跳跃期间 wheel velocity 会被
        起跳动力学瞬时拉到几 rad/s,pitch_lean 把这个误差当成"该前倾",saturate
        到 ±0.2 rad,反而强行让 LQR 把车体推倒。跳跃只需要保持竖直 (target=0)。
        """
        if self._lqr_controller is None:
            return
        equilibrium_pitch = equilibrium_pitch_from_geometry(model, data)
        self._lqr_controller.target[0] = equilibrium_pitch
        self._lqr_controller.target[1] = 0.0
        self._lqr_controller.target[2] = 0.0
        self._lqr_controller.target[3] = 0.0
        self._lqr_controller.target[4] = 0.0  # 跳跃期间不追速度,只保持原地

    def _balance_only_control(
        self,
        model: Any,
        data: Any,
        state: SimState,
        dt: float,
        vmc_control: np.ndarray,
        addresses: Any,
        phase: JumpPhase,
    ) -> np.ndarray:
        """CROUCH/EXTEND/LAND: LQR 轮 forward + yaw 维持平衡,但 NOT 写腿 (VMC 独占)。

        LQR target: 纯 equilibrium_pitch + wheel_vel=0,不带 pitch_lean。跳跃中
        wheel velocity 被动力学拉到几 rad/s,pitch_lean 会把这视为"应该前倾"
        从而把车推倒 — 用 _set_lqr_target_balance_only 隔离这个机制。

        EXTEND 期间 LQR 轮 forward 关键: 自由发展的 pitch 会在 70-150ms 内长到
        ~0.1 rad (sqrt(g/L) 不稳定模态),让 thrust 大量分解到水平,起跳不成功;
        LQR 抑制 pitch 漂移。腿 motor 输出由 VMC 独占。

        airborne 安全: contact_count == 0 时 LQR forward 和 yaw 清零。无地面
        摩擦时任何 wheel 力矩只会让 wheel 自由加速,重新接地切向滑动反扭翻车。
        """
        if self._lqr_controller is None:
            self._initialize_lqr(model, data, state)
        if self._lqr_controller is None:
            raise ValueError("failed to initialize LQR controller")
        if not self.params.fixed_height:
            self._ensure_lqr_height_bin(model, data, state)

        self._set_lqr_target_balance_only(model, data)
        self._lqr_controller.feedforward = np.zeros(2)
        lqr_control = self._lqr_controller(model, data, state)
        if state.contact_count == 0:
            forward_torque = 0.0
            yaw_correction = 0.0
        else:
            forward_torque = float(lqr_control[0])
            yaw_correction = self._compute_yaw_correction(state, dt)
        # roll_diff_leg = 0: 不写腿 motor,把整个 leg actuator 量程让给 VMC。

        control = self._allocate_balance_control(model, forward_torque, 0.0, yaw_correction, addresses)
        return self._merge_vmc_and_clip(model, control, vmc_control, addresses, phase)

    def __call__(self, model: Any, data: Any, state: SimState) -> np.ndarray:
        dt = float(model.opt.timestep)
        addresses = model_addresses(model)
        vmc_control = self.vmc_controller(model, data, state)

        phase = (
            self.vmc_controller.phase_machine.phase
            if self.vmc_controller.phase_machine is not None
            else JumpPhase.STAND
        )

        if phase == JumpPhase.FALLEN:
            self._reset_balance_state()
            self._last_phase = phase
            return np.zeros(model.nu)

        if phase == JumpPhase.FLIGHT:
            # 不应用 LQR (无地面 wheel torque 没意义),但保留 VMC 的腿电机输出 —
            # VMC FLIGHT 分支用对称腿做 pitch_rate 反作用阻尼,防止空中翻车。
            # 见 vmc.py FLIGHT 分支。
            self._handle_jump_entry()
            self._last_phase = phase
            return vmc_control

        if phase == JumpPhase.STAND:
            self._smooth_command_targets(dt)
            self._handle_stand_entry(model, data, state)
            control = self._stand_control(model, data, state, dt, vmc_control, addresses)
        else:
            # CROUCH / EXTEND / LAND: LQR 轮平衡 + VMC 轨迹独占腿
            self._handle_jump_entry()
            control = self._balance_only_control(model, data, state, dt, vmc_control, addresses, phase)

        self._last_phase = phase
        return control
