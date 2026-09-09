from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from src.controllers.lqr import solve_discrete_lqr
from src.model_semantics import MODEL_SEMANTICS, WHEEL_FORWARD_SIGNS, WHEEL_RADIUS
from src.state import model_addresses


# 实测标定（2026-09-09，名义站姿 h_hip=0.30）：
# MuJoCo 单步开环脉冲 +1 N·m 虚拟前进力矩 → Δpitch_rate ≈ -0.0243；
# 简化模型（m≈17 kg, L≈0.249 m, r=0.070 m）预测 -0.00675。
# 实际灵敏度约为简化模型的 3.6 倍（轮子/传动瞬态 + 接触摩擦延迟），
# 直接按简化 B 解 LQR 会让增益偏大 → 自激震荡。故把 B 的 pitch 行乘以该标定系数；
# 换质量/轮径/站姿后需重新标定。
BALANCE_B_PITCH_SCALE = 3.6


# =============================================================================
# balance_lqr.py
# 为轮腿机器人提供平衡控制所需的 LQR 设计函数：
#   * 通过 MuJoCo 实测动力学参数，构造简化离散线性模型；
#   * 求解 6D / 5D 平衡 LQR 增益；
#   * 计算状态选择矩阵与控制选择矩阵，供上层控制器调用。
# =============================================================================


# 串行腿 roll 差动控制的方向符号（每侧 髋+膝 同号）。
# TODO(阶段4实验)：真实 roll 方向符号要用"给左侧正力矩看机身往哪倒"的实验确认。
# 注：STAND 正常路径 roll_torque=0（roll 找平由 VMC 左右腿高差负责），
# 该映射只影响 LQR 备用的 roll 差动通道。
LEG_ROLL_DIFF_SIGNS: dict[str, float] = {
    "link_002_joint": -1.0,  # 右髋
    "link_003_joint": -1.0,  # 右膝
    "link_005_joint": 1.0,   # 左髋
    "link_006_joint": 1.0,   # 左膝
}


def _wheel_body_names() -> tuple[str, str]:
    """按 MODEL_SEMANTICS 返回 (左轮 body, 右轮 body) 名字。"""
    role_to_body = {role: body for body, role in MODEL_SEMANTICS.body_roles.items()}
    try:
        return role_to_body["left_wheel"], role_to_body["right_wheel"]
    except KeyError as exc:  # pragma: no cover
        raise ValueError("MODEL_SEMANTICS.body_roles missing left/right wheel roles") from exc


def _leg_common_to_pitch_coupling(model: Any, data: Any) -> float:
    """实测 common-mode leg ctrl 对 base pitch_rate 的耦合系数 (rad/s² per N·m·leg)。

    在当前 (qpos, qvel) snapshot 下 mj_forward 两次: ctrl=0 baseline 与 ctrl=(L:+1, R:+1)。
    取 base pitch dof (root_qvel + 3) 的 qacc 差, 转换为 pitch_rate (= -wx) 加速度。

    返回值约定: τ_L = τ_R = c 时, base pitch_rate_acc ≈ coupling * c。
    实测在 standing pose ≈ -0.15 rad/s² per N·m·leg (两腿同号 +1 → body 向后仰)。

    不修改 data 状态 (snapshot/restore)。
    """
    addresses = model_addresses(model)  # 获取模型中各关节/执行器的索引映射
    qpos_save = np.array(data.qpos, copy=True)  # 备份当前位置，准备临时修改数据
    qvel_save = np.array(data.qvel, copy=True)  # 备份当前速度
    ctrl_save = np.array(data.ctrl, copy=True)  # 备份当前控制输入

    try:
        data.qvel[:] = 0.0  # 基线测试时将速度清零，排除初始速度影响
        data.ctrl[:] = 0.0  # 基线测试时将控制输入清零
        mujoco.mj_forward(model, data)  # 进行一次正向动力学计算，得到零输入基线
        pitch_dof = addresses.root_qvel + 3  # 根自由度速度中的 pitch 自由度索引
        qacc_pitch_base = float(data.qacc[pitch_dof])  # 记录基线条件下的 pitch 加速度

        for joint_name in MODEL_SEMANTICS.leg_motor_joints:
            data.ctrl[addresses.actuators[joint_name]] = 1.0  # 给每条腿施加 +1 单位 common-mode 力矩脉冲
        mujoco.mj_forward(model, data)  # 在脉冲输入下再次进行正向动力学计算
        qacc_pitch_pulse = float(data.qacc[pitch_dof])  # 记录脉冲条件下的 pitch 加速度
    finally:
        data.qpos[:] = qpos_save  # 无论是否出错，都恢复原始位置
        data.qvel[:] = qvel_save  # 恢复原始速度
        data.ctrl[:] = ctrl_save  # 恢复原始控制输入
        mujoco.mj_forward(model, data)  # 恢复后重新正向计算，保证外部状态一致

    # state.pitch_rate 符号约定: pitch_rate = -wx (见 sim/state.py).
    pitch_rate_acc = -(qacc_pitch_pulse - qacc_pitch_base)
    if not np.isfinite(pitch_rate_acc):
        raise ValueError("leg-common to pitch coupling probe produced non-finite result")
    return pitch_rate_acc


def wheel_ff_gain_for_leg_common(model: Any, data: Any) -> float:
    """计算 wheel forward virtual torque 的 feedforward 增益, 用于抵消 VMC 共模 leg
    torque 引起的 pitch 扰动。

    原理:
      continuous pitch_rate_acc 来自 leg common: coupling * τ_leg_common
      continuous pitch_rate_acc 来自 wheel:      -1/(R*M*L) * τ_wheel_forward
      抵消方程: τ_wheel_forward = coupling * τ_leg_common * R * M * L

    返回 gain 使得 τ_wheel_ff = gain * τ_leg_common_average。
    """
    coupling = _leg_common_to_pitch_coupling(model, data)  # 实测共模腿力矩对 pitch_rate 的耦合系数
    pendulum_length = _com_height_above_wheels(model, data)  # 计算轮轴到质心的等效摆长
    effective_mass = max(float(np.sum(model.body_mass)), 1e-6)  # 总质量，避免零质量导致除零
    gain = coupling * WHEEL_RADIUS * effective_mass * pendulum_length  # 按抵消方程计算前馈增益
    if not np.isfinite(gain):
        raise ValueError("wheel feedforward gain non-finite")
    return gain


def compute_balance_lqr_gain(
    model: Any,
    data: Any,
    q_diag: np.ndarray,
    r_diag: np.ndarray,
) -> np.ndarray:
    """求解 6 维平衡 LQR 增益。

    状态量为 [pitch, pitch_rate, roll, roll_rate, wheel_pos, wheel_vel]，
    虚拟控制量为 [wheel_forward, roll_diff]。
    """
    q_diag = _validate_positive_diag(q_diag, (6,), "q_diag")  # 校验 6 维状态权重对角线
    r_diag = _validate_positive_diag(r_diag, (2,), "r_diag")  # 校验 2 维控制权重对角线

    a, b = _reduced_balance_system(model, data)  # 构造 6 维简化离散系统
    q = np.diag(q_diag)  # 将状态权重对角线转成完整 Q 矩阵
    r = np.diag(r_diag)  # 将控制权重对角线转成完整 R 矩阵
    try:
        gain = solve_discrete_lqr(a, b, q, r)  # 调用离散 LQR 求解器得到反馈增益
    except np.linalg.LinAlgError as exc:
        raise ValueError("failed to solve balance LQR") from exc  # 求解失败时包装成业务异常
    if gain.shape != (2, 6) or not np.all(np.isfinite(gain)):
        raise ValueError("balance LQR gain must be finite shape (2, 6)")  # 校验增益尺寸和数值
    return gain  # 返回 2x6 反馈增益矩阵


def compute_balance_lqr_gain_5d(
    model: Any,
    data: Any,
    q_diag: np.ndarray,
    r_diag: np.ndarray,
) -> np.ndarray:
    """5 维平衡 LQR 增益。State: [pitch, pitch_rate, roll, roll_rate, wheel_vel]。

    位置 wheel_pos 不在 LQR state 中——避免位置反馈在 stand 模式下与平衡
    所需的轮子自由运动产生正反馈。位置漂移由外环处理（target_velocity
    或可选的慢速 position anchor）。
    """
    q_diag = _validate_positive_diag(q_diag, (5,), "q_diag")  # 校验 5 维状态权重对角线
    r_diag = _validate_positive_diag(r_diag, (2,), "r_diag")  # 校验 2 维控制权重对角线

    a, b = _reduced_balance_system_5d(model, data)  # 构造不含 wheel_pos 的 5 维系统
    q = np.diag(q_diag)  # 组装状态权重矩阵 Q
    r = np.diag(r_diag)  # 组装控制权重矩阵 R
    try:
        gain = solve_discrete_lqr(a, b, q, r)  # 求解 5 维 LQR 反馈增益
    except np.linalg.LinAlgError as exc:
        raise ValueError("failed to solve balance LQR") from exc  # 求解失败时向上抛出可读错误
    if gain.shape != (2, 5) or not np.all(np.isfinite(gain)):
        raise ValueError("5D balance LQR gain must be finite shape (2, 5)")  # 校验结果尺寸和数值
    return gain  # 返回 2x5 反馈增益矩阵


def _reduced_balance_system(model: Any, data: Any) -> tuple[np.ndarray, np.ndarray]:
    """构造 6 维离散化平衡动力学系统。

    状态顺序：[pitch, pitch_rate, roll, roll_rate, wheel_pos, wheel_vel]。
    采用一阶欧拉离散化，并把轮子前向力矩作为第一个虚拟控制量。
    """
    mujoco.mj_forward(model, data)  # 先执行正向计算，刷新依赖位置/质量矩阵的数据
    pendulum_length = _com_height_above_wheels(model, data)  # 获取轮轴到质心高度
    effective_mass = max(float(np.sum(model.body_mass)), 1e-6)  # 使用总质量作为等效质量并保护除零
    track_width = _track_width(model, data)  # 获取左右轮沿本体 X 轴方向的间距
    roll_inertia = _base_roll_inertia(model, data)  # 获取本体绕轮轴中点的 roll 惯量
    g = 9.81  # 重力加速度，单位 m/s²

    # 控制环以 500Hz 运行，因此离散周期 dt=0.002；MuJoCo 内部步长为 2000Hz。
    control_dt = 0.002  # 控制循环周期，单位秒
    dt = control_dt  # 离散 LQR 使用的采样周期

    b_vel = (1.0 / WHEEL_RADIUS) / effective_mass * dt  # 单位轮力矩在 dt 内产生的轮速增量系数

    a = np.zeros((6, 6))  # 初始化 6x6 离散状态转移矩阵
    a[0, 0] = 1.0  # pitch 状态恒等项
    a[0, 1] = dt  # pitch 位置由 pitch_rate 按 dt 积分
    a[1, 0] = (g / pendulum_length) * dt  # 倒立摆恢复力矩对 pitch_rate 的影响
    a[1, 1] = 1.0  # pitch_rate 状态恒等项
    a[2, 2] = 1.0  # roll 状态恒等项
    a[2, 3] = dt  # roll 位置由 roll_rate 按 dt 积分
    a[3, 2] = (g / pendulum_length) * dt  # 倒立摆恢复力矩对 roll_rate 的影响
    a[3, 3] = 1.0  # roll_rate 状态恒等项
    a[4, 4] = 1.0  # wheel_pos 状态恒等项
    a[4, 5] = dt  # wheel_pos 由 wheel_vel 按 dt 积分
    a[5, 5] = 1.0  # wheel_vel 状态恒等项

    b = np.zeros((6, 2))  # 初始化 6x2 离散控制输入矩阵
    b[1, 0] = -(b_vel / pendulum_length)  # wheel_forward 力矩对 pitch_rate 的离散影响
    b[5, 0] = b_vel  # wheel_forward 力矩对 wheel_vel 的离散影响
    b[3, 1] = (track_width / 2.0) / (roll_inertia * pendulum_length) * dt  # roll_diff 力矩对 roll_rate 的影响

    if a.shape != (6, 6) or b.shape != (6, 2) or not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise ValueError("reduced balance system must be finite with shapes (6, 6) and (6, 2)")
    return a, b


def _reduced_balance_system_5d(model: Any, data: Any) -> tuple[np.ndarray, np.ndarray]:
    """5D 状态线性化系统。State: [pitch, pitch_rate, roll, roll_rate, wheel_vel]。

    与 6D 的区别: 删掉 wheel_pos 行/列。wheel_pos 是 wheel_vel 的积分，
    不在 LQR 反馈中。
    """
    mujoco.mj_forward(model, data)  # 刷新当前模型状态，为后续动力学参数计算做准备
    pendulum_length = _com_height_above_wheels(model, data)  # 获取轮轴到质心的等效摆长
    effective_mass = max(float(np.sum(model.body_mass)), 1e-6)  # 总质量作为等效质量，防止为零
    track_width = _track_width(model, data)  # 获取左右轮之间的横向距离
    roll_inertia = _base_roll_inertia(model, data)  # 获取绕轮轴中点的 roll 转动惯量
    g = 9.81  # 重力加速度

    control_dt = 0.002  # 控制周期，对应 500Hz
    dt = control_dt  # 离散系统采样周期
    b_vel = (1.0 / WHEEL_RADIUS) / effective_mass * dt  # 单位轮力矩造成的轮速增量

    a = np.zeros((5, 5))  # 初始化 5x5 状态转移矩阵
    a[0, 0] = 1.0  # pitch 位置恒等项
    a[0, 1] = dt  # pitch_rate 到 pitch 的离散积分
    a[1, 0] = (g / pendulum_length) * dt  # 倒立摆恢复项对 pitch_rate 的影响
    a[1, 1] = 1.0  # pitch_rate 恒等项
    a[2, 2] = 1.0  # roll 位置恒等项
    a[2, 3] = dt  # roll_rate 到 roll 的离散积分
    a[3, 2] = (g / pendulum_length) * dt  # 倒立摆恢复项对 roll_rate 的影响
    a[3, 3] = 1.0  # roll_rate 恒等项
    a[4, 4] = 1.0  # wheel_vel 恒等项（在 6D 模型中对应 a[5,5]）

    b = np.zeros((5, 2))  # 初始化 5x2 控制输入矩阵
    b[1, 0] = -(b_vel / pendulum_length)  # 前向轮力矩对 pitch_rate 的影响
    b[4, 0] = b_vel  # 前向轮力矩对 wheel_vel 的影响（在 6D 中对应 b[5,0]）
    b[3, 1] = (track_width / 2.0) / (roll_inertia * pendulum_length) * dt  # roll 差动力矩对 roll_rate 的影响
    # 经验标定：pitch/wheel 通道按实测灵敏度放大（见 BALANCE_B_PITCH_SCALE）
    b[1, 0] *= BALANCE_B_PITCH_SCALE
    b[4, 0] *= BALANCE_B_PITCH_SCALE

    if a.shape != (5, 5) or b.shape != (5, 2) or not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise ValueError("5D reduced balance system must be finite with shapes (5, 5) and (5, 2)")
    return a, b


def _com_height_above_wheels(model: Any, data: Any) -> float:
    """计算质心到左右轮轴中点的高度，作为倒立摆等效摆长。"""
    mujoco.mj_forward(model, data)  # 刷新刚体位置，保证 xipos 是最新结果
    body_masses = np.asarray(model.body_mass, dtype=float)  # 读取所有刚体质量
    total_mass = float(np.sum(body_masses))  # 计算模型总质量
    if total_mass <= 0.0:
        raise ValueError("model mass must be positive")  # 总质量非法时直接报错
    com_z = float(np.sum(body_masses * data.xipos[:, 2]) / total_mass)  # 用质量加权平均求质心 Z 坐标
    wheel_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        for body_name in _wheel_body_names()
    ]  # 查找左右轮刚体 ID
    if any(body_id == -1 for body_id in wheel_ids):
        raise ValueError("missing wheel bodies for LQR pendulum geometry")  # 缺少轮体时无法计算摆长
    wheel_z = float(np.mean([data.xipos[body_id, 2] for body_id in wheel_ids]))  # 取左右轮 Z 坐标平均值
    height = com_z - wheel_z  # 质心高度减去轮轴高度
    if not np.isfinite(height) or abs(height) < 1e-6:
        raise ValueError("invalid CoM height above wheels for LQR")  # 高度非法或过小则报错
    return abs(float(height))  # 返回正值高度


def equilibrium_pitch_from_geometry(model: Any, data: Any) -> float:
    """当前关节构型下,CoM 落到 wheel_mid 正上方所需的 pitch 偏置(项目约定)。

    几何推导: 设 CoM 相对 wheel_mid 在本体系下偏移 [_, dy, dz]。绕本体 X 轴
    旋转 θ 后,世界系 Y 分量 = cos(θ)*dy - sin(θ)*dz。稳态 → tan(θ) = dy/dz。
    项目 pitch 约定 (pitch>0=前倾) 与数学旋转角差一个负号,故返回 -atan2(dy, dz)。

    每次调用都重算,跟随腿高度变化。
    """
    mujoco.mj_forward(model, data)  # 刷新位姿数据，用于计算质心和旋转矩阵
    masses = np.asarray(model.body_mass, dtype=float)  # 获取各刚体质量
    total_mass = float(masses.sum())  # 求总质量
    if total_mass <= 0.0:
        raise ValueError("model mass must be positive")  # 总质量非法时抛出异常
    com_world = (masses[:, None] * np.asarray(data.xipos)).sum(axis=0) / total_mass  # 计算世界系质心坐标
    wheel_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        for body_name in _wheel_body_names()
    ]  # 查找左右轮刚体 ID
    if any(b == -1 for b in wheel_ids):
        raise ValueError("missing wheel bodies for equilibrium pitch")  # 缺少轮体时无法求轮轴中点
    wheel_mid = np.mean([data.xipos[b] for b in wheel_ids], axis=0)  # 计算左右轮中点坐标
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")  # 获取 base_link 刚体 ID
    if base_id == -1:
        raise ValueError("missing base_link body")  # 缺少基座时无法求本体坐标系
    rotation = np.asarray(data.xmat[base_id]).reshape(3, 3)  # 读取基座姿态的 3x3 旋转矩阵
    offset_body = rotation.T @ (com_world - wheel_mid)  # 把世界系相对偏移变换到本体坐标系
    dy = float(offset_body[1])  # 本体坐标系下质心相对轮轴中点的 Y 偏移
    dz = float(offset_body[2])  # 本体坐标系下质心相对轮轴中点的 Z 偏移
    if not np.isfinite(dy) or not np.isfinite(dz) or abs(dz) < 1e-6:
        raise ValueError("invalid CoM/wheel geometry for equilibrium pitch")  # 几何关系无效时报错
    return float(-np.arctan2(dy, dz))  # 按项目 pitch 符号约定返回平衡倾角


def _track_width(model: Any, data: Any) -> float:
    """计算左右轮在本体 X 轴方向上的投影距离，即轮距。"""
    mujoco.mj_forward(model, data)  # 刷新轮体和基座位姿
    wheel_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        for body_name in _wheel_body_names()
    ]  # 查找左右轮刚体 ID
    if any(body_id == -1 for body_id in wheel_ids):
        raise ValueError("missing wheel bodies for LQR track width")  # 缺少轮体时无法计算轮距
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")  # 获取基座刚体 ID
    if base_id == -1:
        raise ValueError("missing base_link body for LQR track width")  # 缺少基座时无法确定本体 X 轴
    base_x_axis = data.xmat[base_id].reshape(3, 3)[:, 0]  # 取基座姿态矩阵的第一列，即本体 X 轴
    wheel_delta = data.xipos[wheel_ids[0]] - data.xipos[wheel_ids[1]]  # 左右轮位置差向量
    width = abs(float(np.dot(wheel_delta, base_x_axis)))  # 把轮差投影到本体 X 轴并取绝对值
    if not np.isfinite(width) or width < 1e-6:
        raise ValueError("invalid wheel track width for LQR")  # 轮距非法或过小则报错
    return width  # 返回轮距


def _base_roll_inertia(model: Any, data: Any) -> float:
    """计算整机绕轮轴中点的 roll 方向转动惯量。

    这里使用平行轴定理：每个刚体的惯量加上其质量乘以到转轴的垂直距离平方，
    再累加得到整机关于轮轴中点的 roll 惯量。
    """
    mujoco.mj_forward(model, data)  # 刷新刚体位姿
    wheel_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        for body_name in _wheel_body_names()
    ]  # 查找左右轮刚体 ID
    if any(body_id == -1 for body_id in wheel_ids):
        raise ValueError("missing wheel bodies for LQR roll inertia")  # 缺少轮体时无法确定转轴
    roll_axis = np.array([0.0, 1.0, 0.0])  # 用世界系 Y 轴作为 roll 转轴方向
    pivot = np.mean([data.xipos[body_id] for body_id in wheel_ids], axis=0)  # 转轴经过左右轮中点
    inertia = 0.0  # 初始化总转动惯量
    for body_id, mass in enumerate(np.asarray(model.body_mass, dtype=float)):
        if mass <= 0.0:
            continue  # 跳过质量为零或负值的刚体
        body_inertia = np.asarray(model.body_inertia[body_id], dtype=float)  # 读取刚体自身惯量矩阵
        inertia += float(np.dot(body_inertia, roll_axis * roll_axis))  # 提取刚体绕 roll 轴的自身惯量
        offset = data.xipos[body_id] - pivot  # 刚体质心相对转轴点的位置向量
        perpendicular_sq = float(np.dot(offset, offset) - np.dot(offset, roll_axis) ** 2)  # 计算到转轴的垂直距离平方
        inertia += float(mass) * max(perpendicular_sq, 0.0)  # 平行轴定理累加质量项
    if not np.isfinite(inertia) or inertia < 1e-9:
        raise ValueError("invalid roll inertia for LQR")  # 惯量非法或过小则报错
    return inertia  # 返回 roll 总惯量


def _balance_state_selection(model: Any) -> np.ndarray:
    """构建 6 维平衡状态选择矩阵，从完整 MuJoCo 状态中挑出 LQR 状态。"""
    addresses = model_addresses(model)  # 获取关节/自由度索引映射
    p = np.zeros((6, 2 * model.nv))  # 初始化 6 x (位置速度总自由度) 的零矩阵
    # 与 extract_sim_state()/balance_tangent_state() 保持一致: pitch 是本体 X 轴倾角（与轮轴平行）。
    pitch_index = addresses.root_qvel + 3  # 根速度自由度中的 pitch 角速度索引
    pitch_rate_index = model.nv + addresses.root_qvel + 3  # qvel 段中 pitch 角速度在完整状态中的索引
    roll_index = addresses.root_qvel + 4  # 根速度自由度中的 roll 角速度索引
    roll_rate_index = model.nv + addresses.root_qvel + 4  # qvel 段中 roll 角速度在完整状态中的索引
    wheel_dof_indices = [addresses.joint_qvel[name] for name in MODEL_SEMANTICS.wheel_joints]  # 获取各轮关节速度自由度索引

    p[0, pitch_index] = -1.0  # 提取 pitch 角速度并取负，匹配项目 pitch 符号约定
    p[1, pitch_rate_index] = -1.0  # 提取 pitch 加速度状态并取负
    p[2, roll_index] = 1.0  # 直接提取 roll 角速度
    p[3, roll_rate_index] = 1.0  # 直接提取 roll 加速度状态
    for joint_name, dof_index in zip(MODEL_SEMANTICS.wheel_joints, wheel_dof_indices):
        sign = WHEEL_FORWARD_SIGNS[joint_name]  # 获取该轮的前进方向符号
        p[4, dof_index] = (sign / len(wheel_dof_indices)) * WHEEL_RADIUS  # 轮速转换并平均成 wheel_pos 状态
        p[5, model.nv + dof_index] = (sign / len(wheel_dof_indices)) * WHEEL_RADIUS  # 轮加速度映射到 wheel_vel 状态
    return p  # 返回状态选择矩阵


def _lqr_control_selection(model: Any) -> np.ndarray:
    """构建控制选择矩阵 S, 将 2 维虚拟控制映射到执行器空间。

    列 0 是 forward_wheel，列 1 是实测正 roll 加速度方向的 leg diff torque。
    """
    addresses = model_addresses(model)  # 获取执行器索引映射
    s = np.zeros((model.nu, 2))  # 初始化执行器数 x 2 的零矩阵
    for joint_name in MODEL_SEMANTICS.wheel_joints:
        s[addresses.actuators[joint_name], 0] = WHEEL_FORWARD_SIGNS[joint_name]  # 把前向轮虚拟控制映射到对应轮执行器
    for joint_name, sign in LEG_ROLL_DIFF_SIGNS.items():
        s[addresses.actuators[joint_name], 1] = sign  # 把 roll 差动虚拟控制按符号分配到左右腿
    return s  # 返回控制选择矩阵


def _wheel_control_selection(model: Any) -> np.ndarray:
    """返回轮子相关控制选择矩阵，当前与完整 LQR 控制选择一致。"""
    return _lqr_control_selection(model)  # 委托给完整 LQR 控制选择函数


def _validate_positive_diag(value: np.ndarray, shape: tuple[int, ...], name: str) -> np.ndarray:
    """校验权重对角线向量：形状正确、有限且非负。"""
    array = np.asarray(value, dtype=float)  # 统一转为浮点数组
    if array.shape != shape or not np.all(np.isfinite(array)) or not np.all(array >= 0.0):
        raise ValueError(f"{name} must be finite non-negative shape {shape}")  # 校验失败时给出参数名和期望形状
    return array  # 返回校验后的数组
