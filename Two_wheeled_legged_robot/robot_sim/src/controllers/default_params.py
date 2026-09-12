"""控制器参数预设与序列化工具。

所有控制器参数的唯一真值源。launch_mujoco.py、测试文件和 optimize.py
均从此模块引用参数，禁止在其他位置硬编码。

调参顺序 (实机/headless 都建议从上往下逐层冻结):
    1. VMC PD:      kp_motor, kd_motor                 (腿不抖、不漂)
    2. LAND PD:     kp_land, kd_land                   (落地不弹、不软)
    3. FLIGHT damp: flight_pitch_kd                    (空中姿态稳定)
    4. LQR balance: q_diag[0:4], r_diag                (pitch/roll 不发散)
    5. Forward:     pitch_lean_gain, velocity_ki       (跟踪 target_velocity)
    6. Yaw:         yaw_damping → yaw_ki               (P 先, 必要时再 I)
       Heading:     heading_hold_kp                    (航向保持, 串级在 yaw 阻尼外)
    7. Jump:        见 launch_mujoco MANUAL_JUMP_*     (air_height_max, crouch_depth)
"""
from __future__ import annotations

import json
from typing import Any

import numpy as np

from src.controllers.combined import CombinedParams
from src.controllers.phase import JumpPhaseParams
from src.controllers.vmc import VmcParams


# --- 参数预设 ---

STAND_PARAMS = CombinedParams(
    vmc=VmcParams(
        nominal_height=0.37,  # 站姿腿高 h_base (m) —— 直立平衡（wheel_y_offset=-0.086）
        # h_hip = 0.30，机身水平、轮心离地 0.07 时 h_base = 0.37
        # 串行腿站立需要高刚度 PD + 解析式支撑前馈（地面撑重）。
        # 悬空验收脚本用的是 kp=40/kd=1.5 + leg_gravity_ff_enabled=True（另一组参数）。
        # 折中调参（2026-09-09）：kp=200/kd=30 高位 0.45m 稳撑，
        # 且转向不失稳（kp=250 会把转向反作用传给机身导致侧翻）。
        kp_motor=200.0,
        # 腿高环阻尼 30 → 60（2026-09-11）：过 2cm 单轮坡时腿部环欠阻尼、冲击后弹跳，
        # 腿/轮电机同时饱和把车弹飞（实测峰值速度 3.3 m/s）并翻车。加倍阻尼后
        # 0.10~0.60 m/s 八档全部通过，且姿态大幅改善（0.27 的翻车→9°、0.30 的 31°→5°）。
        kd_motor=60.0,
        gravity_ff_enabled=True,
        kp_land=15.0,   # 旧 land_kp_scale=0.5 × kp_motor=30 的等价绝对值
        kd_land=3.5,    # 旧 kd_motor=1 × land_kd_scale=2.5 + landing_damping=1.0
        max_height_rate=0.05,
        flight_pitch_kd=1.5,
        roll_level_kp_height=0.0,
        roll_level_kd_height=0.002,
        roll_level_offset_limit=0.035,  # 单腿过障碍时伸缩腿保持机身水平（见下方注释）
        slope_squat_margin=0.005,
    ),
    # 2026-05-16 retune after mass rebalance (10.24 kg → 2.20 kg).
    # B matrix scaled ~7× because B ∝ 1/(M·L) and CoM dropped 36 mm.
    # 2026-05-17 重构到 5D LQR:state 为 [pitch, pitch_rate, roll, roll_rate, wheel_vel],
    # 不再包含 wheel_pos。原 Q[4]=50 (wheel_pos) 已删除,原 Q[5]=500 (wheel_vel) 平移到 Q[4]。
    # 2026-05-18 删除 pitch_trim (改用 equilibrium_pitch_from_geometry 自动计算),
    # 加入位置外环参数 position_kp / position_velocity_limit。
    # 2026-05-21 相位独占重构: LQR 在跳跃全程不输出 (CROUCH/EXTEND/FLIGHT/LAND),
    # VMC 独占腿;轮力矩 = 0。删除 flight_wheel_q、jump_extend_feedforward (轨迹动态 FF 取代)。
    # 2026-05-26 解耦速度跟踪三角:Q[wheel_vel] 500 → 300。原配置下 LQR 拼命驱动轮力矩
    # 追 target_velocity, 与外环 PI (pitch_lean_gain + velocity_ki) 形成并行速度
    # 控制器, 二者互相抵消, 调任一参数都会扰动另一个 (docs/lessons_learned.md:179
    # 记录过的"q_pitch_rate=1307 让 pitch_lean=0.3 只产生 0.001 rad 倾角"就是这个症状)。
    # 把 Q[4] 降低到 300 后, LQR 对 wheel_vel 的反馈强度减弱到 60% (原 R=200 → 比值
    # 2.5→1.5), 外环 PI 的"前倾→重力分量加速"路径获得更明确的主导权, pitch_lean 与
    # LQR 不再争抢同一个目标。同时不破坏 cmd_height 阶跃跟踪 (verify_height_sweep
    # 仍 PASS, base_xy_drift 0.10→0.17m, pitch_peak 0.138→0.112)。完全归零 (q4<100)
    # 在 height_sweep 应力测试下导致 base 漂移失控 → 必须先把外环 position loop
    # 增强 (position_kp / position_velocity_limit) 才能进一步降低 Q[4]。
    # Q[4](wheel_vel)=30：阶段4 调试中（曾试 0 会导致 Riccati 奇异）。
    q_diag=np.array([1000.0, 200.0, 1000.0, 200.0, 30.0]),
    r_diag=np.array([200.0, 400.0]),
    velocity_ki=0.3,
    # 2026-09-11 平地驾驶性能专项（对齐开源车：加减速 / 刹车 / 转向 / 速度）：
    # pitch_lean_gain 是"速度误差 → 目标前倾角"的比例项，等效速度环带宽约
    # gain·g/L；0.15 时只有 ~1.5 rad/s，加减速全靠积分慢慢攒，松杆后 0.5 m/s
    # 要 2.07 s / 489 mm 才停住（同场景开源车 0.75 s / 266 mm）。
    # 提到 0.35 后（无头实测，1.0 m/s² 梯形 + 0.5 m/s 匀速）：
    #   跟踪 rms 0.126 → 0.019；0→90% 加速 1.01 → 0.79 s；
    #   刹车 2.07 s / 489 mm → 0.77 s / 230 mm；
    #   停车不反向倒退、停稳残速与停稳时间不变；单轮坡余量反而更好
    #   （|pitch|max 0.146 → 0.074）。
    # 0.5 响应更快（刹车 0.73 s / 250 mm）但松杆反冲 -56 mm/s 偏大，取 0.35 折中。
    pitch_lean_gain=0.35,
    position_kp=3.0,
    # 2026-09-11：1.5 → 3.5。位置环是停车后接管残余运动的二阶环，
    # 阻尼比 ζ ≈ kd/(2√kp)；1.5 时 ζ≈0.43（欠阻尼），高速刹车后会在
    # 锚点附近来回晃（±18 mm/s，1 s 内收不住）。3.5 → ζ≈1.0，实测：
    # 停稳时间 1.21 s → 0.90 s、反向位移 13.2 → 8.2 mm、停稳残速 -2.5 mm/s，
    # 且不影响行驶段（位置环在 blend=0 时完全不参与）。
    position_kd=3.5,
    position_velocity_limit=0.6,
    position_hold_blend_tau=0.2,
    velocity_integral_release_tau=0.25,
    velocity_integral_release_speed=0.10,
    # 试过放宽到 0.06~0.20 来加制动力：刹车距离只再降 7~47mm，但停稳残速从
    # 9mm/s 涨到 18~38mm/s（停稳变慢），得不偿失，保留 0.03。
    position_hold_velocity_gate=0.03,
    # 转向标定（2026-09-09）：yaw_damping=8 在目标 0.3 rad/s 时稳定；
    # 更大阻尼（≥30）后期失稳，保持 8。
    yaw_damping=8.0,
    # 2026-09-11：纯 P 阻尼有 ~19% 稳态速差（cmd 0.5 → 实际 0.40，cmd 0.8 →
    # 实际 0.69）——轮地摩擦是需要力矩才能维持的负载，P 环必须留误差才有输出。
    # 加积分把稳态速差清零（实测 cmd 0.5 → 0.500、0.8 → 0.800），稳态积分仅
    # -0.14（≈0.8 N·m 转向力矩，远小于 ±9 N·m 轮峰值），不挤压平衡通道。
    yaw_ki=6.0,
    # 航向保持: 未发转向指令 (target_yaw_rate≈0) 且站立接地时锁定当前航向, 抵抗扰动。
    # kp=2.0: 0.1rad(5.7°) 偏航 → 0.2rad/s 回正参考; rate_limit 限幅 0.8rad/s 防猛回正。
    # 保守起点, 实机可在 yaw_damping 冻结后再上调。
    heading_hold_kp=2.0,
    heading_hold_rate_limit=0.8,
    fixed_height=False,
    lqr_height_bin_size=0.02,
    # 2026-05-26 ff_gain 默认 0:旧值 4.0 把 leg-height-rate 信号 (经过 LUT
    # dy_wheel_dh 和 height_dtheta) 注入 LQR target[4], 跟 pitch_lean PI 路径
    # 形成第二条速度控制通路。当前几何下 dy_wheel_dh(0.142)≈0.164 (TODO P0-2),
    # 量级小, 移除后 cmd_height 阶跃的 pitch peak 改变 <5%。仅当 cmd_height 大幅
    # 阶跃 pitch peak 不达标时再启用 (启用值参考 4.0)。
    ff_gain=0.0,
    # 高位长期站立标定（2026-09-09）：0.42m 以下 120s 稳定、扰动恢复更快、
    # 残余晃动更小（原 [-38,-6.3] p2p=0.045 vs 现 [-45,-7] p2p=0.028/稳态≈0）。
    wheel_balance_gain_2d=np.array([-45.0, -7.0]),
)

# 2026-05-16: same gains as STAND_PARAMS; drive validated up to 0.5 m/s in
# headless. Old Optuna-tuned STAND_THEN_DRIVE_PARAMS (10.24 kg model) replaced.
STAND_THEN_DRIVE_PARAMS = CombinedParams(
    vmc=VmcParams(
        nominal_height=0.37,
        kp_motor=200.0,
        kd_motor=60.0,   # 见 STAND_PARAMS 注释：过坡口冲击时腿高环欠阻尼会弹跳
        gravity_ff_enabled=True,
        kp_land=15.0,
        kd_land=3.073,  # 旧 kd_motor=1 × land_kd_scale=2.5 + landing_damping=0.628
        flight_pitch_kd=1.5,
        roll_level_kp_height=0.0,
        roll_level_kd_height=0.002,
        # 地形找平前馈：单腿压上障碍时按左右轮高差伸缩双腿，保持机身水平。
        # 2026-09-11 曾误关（当时见爬坡翻车），真因是腿高环欠阻尼（kd_motor=30），
        # 找平指令把欠阻尼环激励成弹跳；kd_motor 提到 60 后找平恢复且更稳。
        # 实测过坡段（|轮高差|>5mm）统计：
        #   关闭：|roll| 均值 2.75°/峰 3.07°，腿高目标差 0.0mm（不伸缩）
        #   打开：|roll| 均值 0.97°/峰 2.30°，腿高目标差 14.7mm（= 轮高差，完全补偿）
        # 与开源车行为一致（它 6.5cm 坡上滚 <1°、单腿伸缩 47mm）。
        roll_level_offset_limit=0.035,
        slope_squat_margin=0.005,
    ),
    q_diag=np.array([1000.0, 200.0, 1000.0, 200.0, 30.0]),
    r_diag=np.array([200.0, 400.0]),
    pitch_lean_gain=0.35,
    velocity_ki=0.3,
    position_kp=3.0,
    position_kd=3.5,
    position_velocity_limit=0.6,
    position_hold_blend_tau=0.2,
    velocity_integral_release_tau=0.25,
    velocity_integral_release_speed=0.10,
    position_hold_velocity_gate=0.03,
    yaw_damping=8.0,
    yaw_ki=6.0,
    heading_hold_kp=2.0,
    heading_hold_rate_limit=0.8,
    wheel_balance_gain_2d=np.array([-45.0, -7.0]),
)

# 跳跃相位机默认参数
DEFAULT_PHASE_PARAMS = JumpPhaseParams()


# --- 序列化工具 ---

def params_to_dict(params: CombinedParams) -> dict[str, Any]:
    """将 CombinedParams 序列化为可 JSON 存储的字典。"""
    return {
        "vmc": {
            "nominal_height": params.vmc.nominal_height,
            "kp_motor": params.vmc.kp_motor,
            "kd_motor": params.vmc.kd_motor,
            "kp_land": params.vmc.kp_land,
            "kd_land": params.vmc.kd_land,
            "max_height_rate": params.vmc.max_height_rate,
            "flight_pitch_kd": params.vmc.flight_pitch_kd,
            "roll_level_kp_height": params.vmc.roll_level_kp_height,
            "roll_level_kd_height": params.vmc.roll_level_kd_height,
            "roll_level_offset_limit": params.vmc.roll_level_offset_limit,
            "slope_squat_margin": params.vmc.slope_squat_margin,
            "stand_torque_limit": params.vmc.stand_torque_limit,
            "gravity_ff_enabled": params.vmc.gravity_ff_enabled,
            "support_ff_include_leg_weight": params.vmc.support_ff_include_leg_weight,
            "leg_gravity_ff_enabled": params.vmc.leg_gravity_ff_enabled,
            "stand_rate_ff_scale": params.vmc.stand_rate_ff_scale,
        },
        "q_diag": params.q_diag.tolist(),
        "r_diag": params.r_diag.tolist(),
        "target_velocity": params.target_velocity,
        "pitch_lean_gain": params.pitch_lean_gain,
        "velocity_ki": params.velocity_ki,
        "position_kp": params.position_kp,
        "position_kd": params.position_kd,
        "position_velocity_limit": params.position_velocity_limit,
        "position_hold_blend_tau": params.position_hold_blend_tau,
        "velocity_integral_release_tau": params.velocity_integral_release_tau,
        "velocity_integral_release_speed": params.velocity_integral_release_speed,
        "position_hold_velocity_gate": params.position_hold_velocity_gate,
        "yaw_damping": params.yaw_damping,
        "yaw_ki": params.yaw_ki,
        # 2026-09-11 平地驾驶专项新增/启用的字段，一并序列化保证 round-trip。
        "yaw_correction_limit": params.yaw_correction_limit,
        "yaw_integral_limit": params.yaw_integral_limit,
        "max_linear_accel": params.max_linear_accel,
        "max_yaw_accel": params.max_yaw_accel,
        "target_yaw_rate": params.target_yaw_rate,
        "heading_hold_kp": params.heading_hold_kp,
        "heading_hold_rate_limit": params.heading_hold_rate_limit,
        "fixed_height": params.fixed_height,
        "lqr_height_bin_size": params.lqr_height_bin_size,
        "ff_gain": params.ff_gain,
        "wheel_balance_gain_2d": (
            None if params.wheel_balance_gain_2d is None else params.wheel_balance_gain_2d.tolist()
        ),
    }


def params_from_dict(d: dict[str, Any]) -> CombinedParams:
    """从字典反序列化 CombinedParams。"""
    vmc_data = d["vmc"]
    return CombinedParams(
        vmc=VmcParams(
            nominal_height=vmc_data["nominal_height"],
            kp_motor=vmc_data["kp_motor"],
            kd_motor=vmc_data["kd_motor"],
            kp_land=vmc_data.get("kp_land", 15.0),
            kd_land=vmc_data.get("kd_land", 3.5),
            max_height_rate=vmc_data.get("max_height_rate", 0.1),
            flight_pitch_kd=vmc_data.get("flight_pitch_kd", 1.5),
            roll_level_kp_height=vmc_data.get("roll_level_kp_height", 0.0),
            roll_level_kd_height=vmc_data.get("roll_level_kd_height", 0.0),
            roll_level_offset_limit=vmc_data.get("roll_level_offset_limit", 0.0),
            slope_squat_margin=vmc_data.get("slope_squat_margin", 0.0),
            stand_torque_limit=vmc_data.get("stand_torque_limit", 30.0),
            gravity_ff_enabled=vmc_data.get("gravity_ff_enabled", False),
            support_ff_include_leg_weight=vmc_data.get("support_ff_include_leg_weight", True),
            leg_gravity_ff_enabled=vmc_data.get("leg_gravity_ff_enabled", False),
            stand_rate_ff_scale=vmc_data.get("stand_rate_ff_scale", 1.0),
        ),
        q_diag=np.array(d["q_diag"]),
        r_diag=np.array(d["r_diag"]),
        target_velocity=d.get("target_velocity", 0.0),
        pitch_lean_gain=d.get("pitch_lean_gain", 0.02),
        velocity_ki=d.get("velocity_ki", 0.1),
        position_kp=d.get("position_kp", 1.5),
        position_kd=d.get("position_kd", 1.5),
        position_velocity_limit=d.get("position_velocity_limit", 0.3),
        position_hold_blend_tau=d.get("position_hold_blend_tau", 0.2),
        velocity_integral_release_tau=d.get("velocity_integral_release_tau", 0.25),
        velocity_integral_release_speed=d.get("velocity_integral_release_speed", 0.10),
        position_hold_velocity_gate=d.get("position_hold_velocity_gate", 0.03),
        yaw_damping=d.get("yaw_damping", 0.5),
        yaw_ki=d.get("yaw_ki", 0.0),
        yaw_correction_limit=d.get("yaw_correction_limit", 2.0),
        yaw_integral_limit=d.get("yaw_integral_limit", 1.0),
        max_linear_accel=d.get("max_linear_accel", 2.5),
        max_yaw_accel=d.get("max_yaw_accel", 1.0),
        target_yaw_rate=d.get("target_yaw_rate", 0.0),
        heading_hold_kp=d.get("heading_hold_kp", 0.0),
        heading_hold_rate_limit=d.get("heading_hold_rate_limit", 1.0),
        fixed_height=d.get("fixed_height", True),
        lqr_height_bin_size=d.get("lqr_height_bin_size", 0.02),
        ff_gain=d.get("ff_gain", 0.0),
        wheel_balance_gain_2d=(
            None if d.get("wheel_balance_gain_2d") is None
            else np.asarray(d["wheel_balance_gain_2d"], dtype=float)
        ),
    )


def params_to_json(params: CombinedParams) -> str:
    """将 CombinedParams 序列化为 JSON 字符串。"""
    return json.dumps(params_to_dict(params), indent=2)


def params_from_json(json_str: str) -> CombinedParams:
    """从 JSON 字符串反序列化 CombinedParams。"""
    return params_from_dict(json.loads(json_str))
