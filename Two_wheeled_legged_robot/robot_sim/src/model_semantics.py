from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ModelSemantics:
    body_roles: dict[str, str]
    joint_roles: dict[str, str]
    wheel_joints: tuple[str, ...]
    leg_motor_joints: tuple[str, ...]
    passive_joints: tuple[str, ...]
    connect_sites: tuple[str, ...]


# =============================================================================
# Two_wheeled_legged_robot（串联双轮腿）模型语义
# 左右对应关系（用户 2026-09-09 确认，勿再改动）：
#   002 侧 = 右腿：link_002 髋、link_003 膝、link_004 轮
#   005 侧 = 左腿：link_005 髋、link_006 膝、link_007 轮
# 每条腿 2 个主动电机 + 1 个轮电机 = 共 6 个物理执行器；
# 串联机构没有四连杆被动关节/闭链约束。
# =============================================================================

# 轮半径：从 URDF 网格实测约 0.070 m（待实机复核后修正）。
WHEEL_RADIUS = 0.070

# 前进方向符号（约定前进 = 机身 +Y）：
#   右轮轴 +X：正力矩 ω>0 绕 +X → 轮子滚向 -Y（需要 -1 反号才向前）
#   左轮轴 -X：正力矩 ω>0 绕 -X → 轮子滚向 +Y（+1 直接向前）
WHEEL_FORWARD_SIGNS: dict[str, float] = {
    "link_007_joint": 1.0,   # 左轮
    "link_004_joint": -1.0,  # 右轮
}


MODEL_SEMANTICS = ModelSemantics(
    body_roles={
        "base_link": "base",
        "link_005": "left_hip",
        "link_006": "left_knee",
        "link_007": "left_wheel",
        "link_002": "right_hip",
        "link_003": "right_knee",
        "link_004": "right_wheel",
    },
    joint_roles={
        "link_005_joint": "left_hip_motor",
        "link_006_joint": "left_knee_motor",
        "link_007_joint": "left_wheel_drive",
        "link_002_joint": "right_hip_motor",
        "link_003_joint": "right_knee_motor",
        "link_004_joint": "right_wheel_drive",
    },
    # 顺序约定（state.py / balance_state.py 依赖）：wheel_joints[0]=左、[1]=右
    wheel_joints=("link_007_joint", "link_004_joint"),
    leg_motor_joints=(
        "link_002_joint",
        "link_003_joint",
        "link_005_joint",
        "link_006_joint",
    ),
    passive_joints=(),
    connect_sites=(),
)
