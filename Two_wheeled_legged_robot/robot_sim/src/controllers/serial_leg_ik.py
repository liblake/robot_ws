"""串行双轮腿（Two_wheeled_legged_robot）的 2R 平面腿 IK。

几何来源：robot.urdf（2026-09-09 数值实测）+ 实机确认。
用户确认的左右关系：002 侧 = 右腿（002髋-003膝-004轮），005 侧 = 左腿。

机构事实：
- 髋/膝/轮三个旋转轴都平行于机身 X 轴，腿在机身 Y-Z 平面内运动；
- 右腿关节轴 = +X，左腿关节轴 = -X（URDF 里 axis=-1 0 0）；
- 髋关节在机身系 z = -0.070 m；
- q=0 时两腿平面零位向量（机身系 Y,Z）相同：
    thigh0 = 髋->膝 = (-0.29348091, -0.06220095)，长度 L1 = 0.300
    shin0  = 膝->轮 = ( 0.28210870, -0.19553796)，长度 L2 = 0.34325（URDF）

站姿律（第一阶段）：轮心保持在髋正下方（同一竖直线上）。
输入约定与上游一致：leg height h_base = base_z - wheel_z（m）。
因髋在机身下 0.07 m：h_hip = h_base - 0.07。

工作区间（2026-09-09 按 wheel_y_offset=-0.075 与关节限位/保持力矩复核）：
- h_base ∈ [0.31, 0.50]（下限 0.31 受髋关节 -0.30 rad 限位限制；上限 0.50 为
  已验证余量内的保守值，静态保持力矩约 15 N·m < 40 N·m 软限）
- 名义站高：h_hip = 0.30 => h_base = 0.37
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# --- 几何常量（唯一真值源；实机测量后改这里） ---
L1 = 0.300        # 髋->膝 平面内长度 (m)
# 注意：L2 必须与"当前 MuJoCo URDF"一致，仿真验收才能 <5mm。
# 实机实测膝轴到轮轴 = 0.340 m（2026-09-09，用户提供）。
# 当 URDF 更新成实机尺寸（或写实机固件）时，把 L2 改为 0.340。
L2 = 0.34325      # 膝->轮 平面内长度 (m) —— URDF 实测
L2_REAL = 0.340   # 实机实测值（存档，暂不用于仿真）
THIGH0 = np.array([-0.29348091, -0.06220095])   # q=0 时 髋->膝 (Y,Z)
SHIN0 = np.array([0.28210870, -0.19553796])     # q=0 时 膝->轮 (Y,Z)，长度 0.34325
HIP_Z_BASE = -0.070   # 髋关节在机身系 z 偏移 (m)

# 关节限位（URDF，用来校核工作区间；运行时 IK 不做硬性检查，由限位挡）
RIGHT_HIP_RANGE = (-0.30, 1.57)
RIGHT_KNEE_RANGE = (-1.57, 0.20)
LEFT_HIP_RANGE = (-1.57, 0.30)
LEFT_KNEE_RANGE = (-0.20, 1.57)


def _rot_angle_about_x(v_from: np.ndarray, v_to: np.ndarray) -> float:
    """绕 +X 把平面向量 v_from 转到 v_to 的旋转角（atan2，单位 rad）。"""
    u1, w1 = v_from
    u2, w2 = v_to
    return float(np.arctan2(u1 * w2 - w1 * u2, u1 * u2 + w1 * w2))


@dataclass(frozen=True)
class SerialLegIk:
    """轮心在髋正下方的解析 2R IK。

    h_base 全部以 h_base = base_z - wheel_z 计（与上游 VMC/日志一致）；
    内部换算成髋-轮竖直落差 h_hip = h_base - 0.07。

    wheel_y_offset：轮心相对髋关节的 Y 偏移（m，+Y=前）。默认 0=轮心在髋正下方。
    2026-09-09：URDF 整机质心在轮轴中点后方约 58 mm，轮心在髋正下方时
    平衡前倾约 13°。把轮子后移 75 mm（wheel_y_offset=-0.075）后平衡角约
    -1.6°（基本直立，同时给低姿态留限位余量）。
    """

    h_min: float = 0.31   # h_base 下限（髋限位）
    h_max: float = 0.50   # h_base 上限（保守，已测保持力矩富余）
    nominal_height: float = 0.37  # h_base 名义站高
    wheel_y_offset: float = -0.075  # 轮心相对髋的 Y 偏移 (m)，正=向前，负=向后

    def clamp_height(self, h_base: float) -> float:
        return float(np.clip(h_base, self.h_min, self.h_max))

    def angles_from_base_height(self, h_base: float, side: str) -> tuple[float, float]:
        """返回 (髋角, 膝角)，单位为 URDF 关节 qpos 的单位（rad）。"""
        if side not in ("left", "right"):
            raise ValueError(f"unknown leg side: {side}")
        h_base = self.clamp_height(h_base)
        h_hip = h_base - 0.07  # HIP_Z_BASE = -0.07 => hip 在 base 下方
        off = float(self.wheel_y_offset)
        r = float(np.hypot(off, h_hip))
        if not (abs(L2 - L1) <= r <= L1 + L2):
            raise ValueError(
                f"hip-wheel distance r={r:.3f} outside planar reach "
                f"[{abs(L2 - L1):.3f}, {L1 + L2:.3f}]"
            )
        a = (L1 * L1 - L2 * L2 + r * r) / (2.0 * r)
        b_sq = L1 * L1 - a * a
        if b_sq < 0.0:
            raise ValueError(f"r={r:.3f} has no two-link solution")
        # 负分支：膝弯向与 q=0 位形同侧（零位姿态是自然站姿）
        b = -float(np.sqrt(b_sq))
        unit = np.array([off, -h_hip]) / r     # 髋->轮 单位方向
        perp = np.array([-unit[1], unit[0]])   # 与 unit 垂直（与旧公式一致的方向）
        knee = a * unit + b * perp             # 髋->膝
        thigh_des = knee
        shin_des = np.array([off, -h_hip]) - knee  # 膝->轮
        psi_hip = _rot_angle_about_x(THIGH0, thigh_des)
        psi_shin_abs = _rot_angle_about_x(SHIN0, shin_des)
        knee_rel = psi_shin_abs - psi_hip
        if side == "right":
            return psi_hip, knee_rel      # 右腿轴 +X
        return -psi_hip, -knee_rel        # 左腿轴 -X
