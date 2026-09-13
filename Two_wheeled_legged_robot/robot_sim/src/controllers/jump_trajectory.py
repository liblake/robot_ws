"""跳跃 CoM 高度轨迹规划 (业界 4 相: CROUCH / EXTEND / FLIGHT / LAND)。

参考: SLIP + Hierarchical Jumping Optimization。每相位用 5 阶多项式连接
position/velocity/acceleration 边界条件,VMC 沿轨迹做动态前馈跟踪。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Tuple

import numpy as np


GRAVITY = 9.81


class QuinticTrajectory:
    """5 次多项式:h(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5。

    6 个边界条件: 起点 (h0, hd0, hdd0) 和终点 (hf, hdf, hddf)。
    """

    def __init__(
        self,
        h0: float,
        hd0: float,
        hdd0: float,
        hf: float,
        hdf: float,
        hddf: float,
        duration: float,
    ) -> None:
        if duration <= 0.0:
            raise ValueError(f"QuinticTrajectory duration must be > 0, got {duration}")
        self.duration = float(duration)

        T = float(duration)
        # 起点条件直接给出 c0/c1/c2:
        self._c0 = float(h0)
        self._c1 = float(hd0)
        self._c2 = float(hdd0) / 2.0

        # 终点条件解 c3/c4/c5: 3x3 线性方程组
        # h(T)  = c0 + c1*T + c2*T^2 + c3*T^3 + c4*T^4 + c5*T^5 = hf
        # h'(T) = c1 + 2*c2*T + 3*c3*T^2 + 4*c4*T^3 + 5*c5*T^4 = hdf
        # h''(T)= 2*c2 + 6*c3*T + 12*c4*T^2 + 20*c5*T^3 = hddf
        rhs = np.array([
            hf - self._c0 - self._c1 * T - self._c2 * T**2,
            hdf - self._c1 - 2.0 * self._c2 * T,
            hddf - 2.0 * self._c2,
        ], dtype=float)
        A = np.array([
            [T**3, T**4, T**5],
            [3.0 * T**2, 4.0 * T**3, 5.0 * T**4],
            [6.0 * T, 12.0 * T**2, 20.0 * T**3],
        ], dtype=float)
        c3, c4, c5 = np.linalg.solve(A, rhs)
        self._c3 = float(c3)
        self._c4 = float(c4)
        self._c5 = float(c5)

    def _clip_t(self, t: float) -> float:
        return float(np.clip(t, 0.0, self.duration))

    def height(self, t: float) -> float:
        t = self._clip_t(t)
        return (
            self._c0 + self._c1 * t + self._c2 * t**2
            + self._c3 * t**3 + self._c4 * t**4 + self._c5 * t**5
        )

    def velocity(self, t: float) -> float:
        t = self._clip_t(t)
        return (
            self._c1 + 2.0 * self._c2 * t + 3.0 * self._c3 * t**2
            + 4.0 * self._c4 * t**3 + 5.0 * self._c5 * t**4
        )

    def acceleration(self, t: float) -> float:
        t = self._clip_t(t)
        return (
            2.0 * self._c2 + 6.0 * self._c3 * t + 12.0 * self._c4 * t**2
            + 20.0 * self._c5 * t**3
        )

    def sample(self, t: float) -> Tuple[float, float, float]:
        return self.height(t), self.velocity(t), self.acceleration(t)


class QuarticTrajectory:
    """4 次多项式: h(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4。

    5 个边界条件: 起点 (h0, hd0, hdd0) + 终点 (hf, hdf)。
    用于 EXTEND 段 — 5 阶多项式带 hddf=0 会产生起点 "下凹" (h 先下降后上升),
    motor target 反向跟随,EXTEND 推不动。4 阶不约束 hddf,h(t) 单调递增。
    """

    def __init__(
        self,
        h0: float,
        hd0: float,
        hdd0: float,
        hf: float,
        hdf: float,
        duration: float,
    ) -> None:
        if duration <= 0.0:
            raise ValueError(f"QuarticTrajectory duration must be > 0, got {duration}")
        self.duration = float(duration)

        T = float(duration)
        self._c0 = float(h0)
        self._c1 = float(hd0)
        self._c2 = float(hdd0) / 2.0

        # 终点条件解 c3/c4: 2x2 线性方程组
        rhs = np.array([
            hf - self._c0 - self._c1 * T - self._c2 * T**2,
            hdf - self._c1 - 2.0 * self._c2 * T,
        ], dtype=float)
        A = np.array([
            [T**3, T**4],
            [3.0 * T**2, 4.0 * T**3],
        ], dtype=float)
        c3, c4 = np.linalg.solve(A, rhs)
        self._c3 = float(c3)
        self._c4 = float(c4)

    def _clip_t(self, t: float) -> float:
        return float(np.clip(t, 0.0, self.duration))

    def height(self, t: float) -> float:
        t = self._clip_t(t)
        return self._c0 + self._c1 * t + self._c2 * t**2 + self._c3 * t**3 + self._c4 * t**4

    def velocity(self, t: float) -> float:
        t = self._clip_t(t)
        return self._c1 + 2.0 * self._c2 * t + 3.0 * self._c3 * t**2 + 4.0 * self._c4 * t**3

    def acceleration(self, t: float) -> float:
        t = self._clip_t(t)
        return 2.0 * self._c2 + 6.0 * self._c3 * t + 12.0 * self._c4 * t**2

    def sample(self, t: float) -> Tuple[float, float, float]:
        return self.height(t), self.velocity(t), self.acceleration(t)


class ConstantAccelerationTrajectory:
    """恒定加速度起跳轨迹: h(t) = h0 + 0.5*a*t²,从静止加速到 v_target。

    由 stroke d = h_target - h0 和 v_target 反推 duration:
        v² = 2*a*d  →  a = v²/(2d)
        v = a*T     →  T = v/a = 2d/v
    这样电机从 t=0 就提供恒定推力 a + g (而不是 QuarticTrajectory 那种后置爆发),
    在有限的 leg stroke 内最大化能量传递。

    用于 EXTEND 段。CROUCH/LAND 用 QuinticTrajectory (需要平滑两端的速度/加速度)。
    """

    def __init__(self, h0: float, h_target: float, v_target: float) -> None:
        d = float(h_target) - float(h0)
        v = float(v_target)
        if d <= 0.0:
            raise ValueError(f"ConstantAccelerationTrajectory needs h_target > h0, got d={d}")
        if v <= 0.0:
            raise ValueError(f"ConstantAccelerationTrajectory needs v_target > 0, got v={v}")
        self.duration = 2.0 * d / v
        self._a = v / self.duration  # = v² / (2*d)
        self._h0 = float(h0)

    def _clip_t(self, t: float) -> float:
        return float(np.clip(t, 0.0, self.duration))

    def height(self, t: float) -> float:
        t = self._clip_t(t)
        return self._h0 + 0.5 * self._a * t * t

    def velocity(self, t: float) -> float:
        t = self._clip_t(t)
        return self._a * t

    def acceleration(self, t: float) -> float:
        # 恒定加速度,与 t 无关
        del t
        return self._a

    def sample(self, t: float) -> Tuple[float, float, float]:
        return self.height(t), self.velocity(t), self.acceleration(t)


@dataclass(frozen=True)
class JumpTrajectoryParams:
    """跳跃轨迹规划参数。

    EXTEND duration 由 stroke (h_high - h_low) 和 v_takeoff 反推,不再独立指定。
    """

    # === 本机（串联双轮腿）参数，2026-09-12 按 leg 工作区间 [0.31, 0.50] 重标 ===
    #
    # 注意：这组默认值原来是开源车（四连杆、腿高 0.078~0.154 m）的 LUT 数值，
    # 直接沿用到本机会完全失效：h_start=0.37 代进原参数得到 h_low=0.095、
    # h_high=0.140，再被 IK clamp_height([0.31,0.50]) 夹住 → 两端都变 0.31、
    # 行程为 0（实测触发一次跳跃机身只动 5 mm）。
    #
    # h_min / h_safe_high：本机 IK 可达区间 [0.31, 0.50] 的两端，各留 10~20 mm 余量。
    h_min: float = 0.32
    h_safe_high: float = 0.48
    # CROUCH 自适应深度: target = max(h_min, h_start - crouch_depth)。
    crouch_depth: float = 0.05
    # EXTEND 固定伸腿行程 (m): h_high = h_low + extend_stroke。固定行程让不同 cmd_height
    # 起跳的伸腿动力学一致 (离地注入机身的后仰角动量一致)。0.32 → 0.48 正好用满区间。
    #
    # 行程与可达推力的关系（实测 dh/dq 在工作区间内很平：髋 0.075、膝 0.196~0.225）：
    #   每腿竖向推力 = min(τ_max/|∂h/∂q_hip|, τ_max/|∂h/∂q_knee|)
    #   软限幅 30 N·m → 267 N 总推力（净加速度 5.9 m/s²@蹲姿 ~ 8.2@伸直）
    #   执行器上限 40 N·m → 356 N（EXTEND 不受软限幅，所以瓶颈其实在 CROUCH/LAND）
    # 恒定加速度轨迹要求 a = v²/(2·行程) = 2g·h_air/(2·行程)，反推 air_height_max 上限
    # ≈ 9.6 cm（按 30 N·m 可迁移的推力算）。
    extend_stroke: float = 0.16
    # 时间剖面 (EXTEND 由 ConstantAccelerationTrajectory 自动计算 duration)。
    crouch_duration: float = 0.25
    # 0.20（2026-09-13，原 0.30）：行驶中跳跃的实测权衡。LAND 期间俯仰下潜 +
    # 偏置站姿律会水平拖拽轮子产生滑动摩擦（整车被刹，0.4 m/s 跳一次掉到
    # 0.06 m/s 甚至短暂倒退）；拖拽时间 ∝ land_duration，0.20 使行驶跳质心
    # 速度最低点从 −0.05 回到 +0.02 m/s，原地跳质量不降反升（落地 pitch 峰
    # 10.5°→9.4°，上弹 0.04→0.03 m/s）。
    land_duration: float = 0.20
    # 默认空中高度 (m)，cmd_jump=1 时跳多高。
    # 0.08 → v_takeoff=1.25 m/s、需求加速度 4.9 m/s²、峰值膝力矩 ≈28 N·m（在软限幅内，
    # 即"用现有 30 N·m 软限幅就能跳"，结论可迁移到实机）。验证通路后再往上推。
    #
    # 2026-09-13 提到 0.30（用户要求跳更高，执行器 ctrlrange 同步 ±40→±60）：
    # v_target = sqrt(2g·0.30) = 2.42 m/s、需求加速度 18.3 m/s²、膝力矩峰
    # ≈53 N·m（±60 内余量 12%）。实测质心弹道 ≈ 目标 × 0.39 ≈ 12 cm。
    # 历史标定（±40 时代，air 0.15 → 实测弹道 58~61 mm）比例关系仍成立。
    air_height_max: float = 0.30
    # 落地后回到 stand 的目标高度；实际由 JumpTrajectory(h_start) / setup_land(h_target)
    # 覆盖，这里只是兜底。
    h_stand_after_land: float = 0.37

    def adaptive_crouch_target(self, h_start: float) -> float:
        return max(float(self.h_min), float(h_start) - float(self.crouch_depth))


class JumpTrajectory:
    """整个跳跃过程的轨迹。

    - CROUCH: QuinticTrajectory,h_start → h_low,两端静止。
    - EXTEND: ConstantAccelerationTrajectory,h_low → h_high,从静止匀加速到 v_takeoff。
      duration 由 stroke 和 v_takeoff 反推,而非独立指定 — 保证电机从 t=0 就以恒定
      推力工作,在有限 stroke 内最大化能量传递。
    - LAND: QuinticTrajectory,着地时生成,吸收落地冲量。
    - FLIGHT: 不规划 (空中无控)。
    """

    def __init__(
        self,
        params: JumpTrajectoryParams,
        h_start: float,
        cmd_jump_amplitude: float,
    ) -> None:
        self.params = params
        self.h_start = float(h_start)
        # LAND 终点 = 起跳前的高度。如果固定用 params.h_stand_after_land (0.142),
        # 当用户 cmd_height < 0.142 时 LAND 把腿伸到中位 → STAND 立刻把腿收回,
        # 产生"落地瞬间突兀伸腿再砸下"的诡异动作。
        self.h_target_after_land = float(h_start)

        amp = float(np.clip(cmd_jump_amplitude, 0.0, 1.0))
        self.h_air = amp * float(params.air_height_max)
        # 弹道起跳速度: v_takeoff = sqrt(2 g h_air)。h_air=0 时为 0 (无跳)。
        self.v_takeoff = float(np.sqrt(2.0 * GRAVITY * max(self.h_air, 0.0)))

        h_low = params.adaptive_crouch_target(h_start)
        # 固定 EXTEND 行程: h_high = h_low + extend_stroke。若超过 h_safe_high (避开奇异区
        # 的上限) 则封顶, 并把整个起跳窗口下移 (h_low 随之下降, 不低于 h_min), 保持行程
        # 一致。这样所有 cmd_height 起跳的伸腿行程相同 → 离地注入机身的后仰角动量相同,
        # 不再出现低 cmd_height 起跳行程过大、落地前倾/漂移放大的现象。
        h_high = h_low + float(params.extend_stroke)
        if h_high > float(params.h_safe_high):
            h_high = float(params.h_safe_high)
            h_low = max(float(params.h_min), h_high - float(params.extend_stroke))

        self.h_low = h_low
        self.h_high = h_high

        # CROUCH: h_start → h_low, 两端速度/加速度 = 0
        self.crouch = QuinticTrajectory(
            h0=self.h_start, hd0=0.0, hdd0=0.0,
            hf=h_low, hdf=0.0, hddf=0.0,
            duration=float(params.crouch_duration),
        )
        # EXTEND: 恒定加速度从 h_low 到 h_high,末速度 = v_takeoff。duration 自动。
        # 0 跳跃情况 (v_takeoff=0) 用占位 QuarticTrajectory (不会实际触发起跳)。
        if self.v_takeoff > 1e-6:
            self.extend: Any = ConstantAccelerationTrajectory(
                h0=h_low, h_target=h_high, v_target=self.v_takeoff,
            )
        else:
            # cmd_jump_amplitude=0 时 v_takeoff=0,用一个不会移动的占位 trajectory
            self.extend = QuarticTrajectory(
                h0=h_low, hd0=0.0, hdd0=0.0,
                hf=h_low, hdf=0.0,
                duration=0.01,
            )
        # LAND 在着地时生成 (依赖落地速度/高度)
        self.land: QuinticTrajectory | None = None

    def setup_land(self, h_contact: float, v_contact: float, h_target: float | None = None) -> None:
        """着地瞬间生成 LAND 轨迹: h_contact → h_target (默认 = 起跳前高度)。

        v_contact 应为负 (向下),终点静止。h_target 通常传入 VMC 当前的
        nominal_height (即 cmd_height 滑条值),这样落地后的目标就是用户实际
        想要的站立姿态,避免 LAND 把腿强行伸到固定的 0.142 中位、然后 STAND
        立刻收回的诡异"突兀伸腿"动作。
        """
        target = float(h_target) if h_target is not None else float(self.h_target_after_land)
        self.land = QuinticTrajectory(
            h0=float(h_contact), hd0=float(v_contact), hdd0=0.0,
            hf=target, hdf=0.0, hddf=0.0,
            duration=float(self.params.land_duration),
        )

    # CROUCH/EXTEND/LAND 期望的弹道顶点 (供相位机判断是否完成)
    @property
    def crouch_target(self) -> float:
        return self.h_low

    @property
    def extend_target(self) -> float:
        return self.h_high

    def is_zero_jump(self) -> bool:
        return self.h_air <= 1e-9
