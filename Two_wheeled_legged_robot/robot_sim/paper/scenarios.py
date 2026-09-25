"""论文场景注册表。

一个"场景"= 地形 + 命令时序（速度/转向/腿高）+ 时长 + 参数覆盖 + 分析窗口。
这里只做声明，不实现控制与仿真——控制器和仿真都复用 src/ 下已验证的代码。

想加场景：在 SCENARIOS 字典里加一条即可，不需要动控制器。
想临时改参数：命令行 `--set vmc.stand_rate_ff_scale=0`，不用改本文件。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from src.mjcf_builder import (
    JumpStepTerrain,
    SingleWheelTrapezoidTerrain,
    WavyRoadTerrain,
)

URDF_RELATIVE_PATH = "src/robot/robot.urdf"
NOMINAL_HEIGHT = 0.37


@dataclass(frozen=True)
class Schedule:
    """分段线性时间序列。points = ((t0, v0), (t1, v1), ...)，t 递增，两端外推保持。"""

    points: tuple[tuple[float, float], ...]

    def __call__(self, t: float) -> float:
        pts = self.points
        if not pts:
            return 0.0
        if t <= pts[0][0]:
            return float(pts[0][1])
        for (t0, v0), (t1, v1) in zip(pts, pts[1:]):
            if t <= t1:
                if t1 <= t0:
                    return float(v1)
                a = (t - t0) / (t1 - t0)
                return float(v0 + a * (v1 - v0))
        return float(pts[-1][1])


def hold(value: float) -> Schedule:
    return Schedule(((0.0, float(value)),))


ZERO = hold(0.0)
HEIGHT_NOMINAL = hold(NOMINAL_HEIGHT)


@dataclass(frozen=True)
class Scenario:
    name: str
    group: str
    desc: str
    duration: float
    terrain: str | None = "flat"
    terrain_kwargs: dict = field(default_factory=dict)
    velocity: Schedule = ZERO
    yaw_rate: Schedule = ZERO
    height: Schedule = HEIGHT_NOMINAL
    # 跳跃触发：按时间（秒）或按机身 y（m）——台阶用后者。
    jump_times: tuple[float, ...] = ()
    jump_at_y: tuple[float, ...] = ()
    jump_amplitude: float = 1.0
    # 参数覆盖，形如 {"vmc.stand_rate_ff_scale": 0.0}；命令行 --set 会叠加在这上面。
    overrides: dict = field(default_factory=dict)
    # 跳跃轨迹参数覆盖，点分路径写入 JumpTrajectoryParams（例如 {"crouch_duration": 0.25}）。
    # 与 overrides 分开是因为轨迹参数不在 CombinedParams 里，只在起跳瞬间被读取。
    jump_overrides: dict = field(default_factory=dict)
    # 分析窗口（名称 → (起, 止) 秒），指标只在这段时间内统计，便于做对照。
    windows: dict = field(default_factory=dict)


def _drive_speed_sweep(v_target: float) -> Schedule:
    """0→1.5 s 线性加速到 v_target，保持到 8.5 s，然后松杆滑行到 10.5 s。"""
    return Schedule(((0.0, 0.0), (1.5, v_target), (8.5, v_target), (10.5, 0.0)))


# --------------------------------------------------------------------------- #
# 场景表
# --------------------------------------------------------------------------- #
SCENARIOS: dict[str, Scenario] = {}


def _add(scn: Scenario) -> None:
    if scn.name in SCENARIOS:
        raise ValueError(f"duplicate scenario name: {scn.name}")
    SCENARIOS[scn.name] = scn


# === A. 平地基础运动 =========================================================

_add(Scenario(
    name="stand_12s",
    group="ground",
    desc="平地静态站立 12 s（俯仰/侧倾/漂移）",
    duration=12.0,
    windows={"steady": (1.0, 12.0)},
))

_add(Scenario(
    name="drive_pm0p5",
    group="ground",
    desc="前进 0.5 m/s → 停 → 后退 0.5 m/s → 停（往返）",
    duration=15.0,
    velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.25, 0.5), (6.25, 0.5),
                       (7.5, 0.0), (8.5, 0.0), (9.75, -0.5), (13.75, -0.5), (15.0, 0.0))),
    windows={"forward": (2.5, 6.0), "backward": (10.0, 13.5)},
))

for _v, _tag in ((0.8, "0p8"), (1.5, "1p5"), (2.0, "2p0"), (2.5, "2p5")):
    _add(Scenario(
        name=f"speed_{_tag}",
        group="ground",
        desc=f"平地加速到 {_v} m/s 匀速后松杆（速度跟踪与刹车）",
        duration=11.0,
        velocity=_drive_speed_sweep(_v),
        windows={"cruise": (2.0, 8.0), "brake": (8.5, 11.0)},
    ))

# 3.0 m/s：仿真能跑，但超过实机转速上限（350 rpm → 2.57 m/s），用于说明"仿真≠实机可用"
_add(Scenario(
    name="speed_3p0",
    group="ground",
    desc="平地加速到 3.0 m/s 匀速后松杆（超出实机转速上限，仅说明仿真能力）",
    duration=11.0,
    velocity=_drive_speed_sweep(3.0),
    windows={"cruise": (2.0, 8.0), "brake": (8.5, 11.0)},
))

_add(Scenario(
    name="turn_0p5",
    group="ground",
    desc="平地原地转向：指令 0.5 rad/s 正转，再反转",
    duration=12.0,
    yaw_rate=Schedule(((0.0, 0.0), (1.0, 0.0), (1.5, 0.5), (5.5, 0.5),
                       (6.0, 0.0), (6.5, -0.5), (10.5, -0.5), (11.0, 0.0))),
    windows={"turn_pos": (2.0, 5.0), "turn_neg": (7.0, 10.0)},
))

_add(Scenario(
    name="height_cycle",
    group="ground",
    desc="原地调腿高：0.37 → 0.45 → 0.33 → 0.37 m",
    duration=20.0,
    height=Schedule(((0.0, 0.37), (2.0, 0.37), (5.0, 0.45), (8.0, 0.45),
                     (11.0, 0.33), (14.0, 0.33), (17.0, 0.37), (20.0, 0.37))),
    windows={"high": (6.0, 8.0), "low": (12.0, 14.0)},
))

_add(Scenario(
    name="composite_0p4_0p2",
    group="ground",
    desc="复合运动：0.4 m/s 前进 + 0.2 rad/s 转向（弧线）",
    duration=14.0,
    velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.5, 0.4), (13.0, 0.4), (14.0, 0.0))),
    yaw_rate=Schedule(((0.0, 0.0), (1.0, 0.0), (1.5, 0.2), (12.0, 0.2), (13.0, 0.0))),
    windows={"cruise": (3.0, 11.0)},
))

_add(Scenario(
    name="straight_10m",
    group="ground",
    desc="平地直线 10 m（横向漂移与航向偏差）",
    duration=30.0,
    velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.5, 0.4), (28.0, 0.4), (30.0, 0.0))),
    windows={"cruise": (3.0, 27.0)},
))

_add(Scenario(
    name="stop_release",
    group="ground",
    desc="0.5 m/s 匀速后松杆刹车（刹车时间与距离）",
    duration=12.0,
    velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.5, 0.5), (6.0, 0.5), (6.01, 0.0))),
    windows={"cruise": (4.0, 5.9), "brake": (6.0, 12.0)},
))

# 速度阶梯：0.5→1.0→1.5→2.0→2.5 m/s，每档 1.5 s 斜坡 + 4 s 保持。
# 一次跑完就能在一张图里展示多档速度跟踪，供论文"平地运动"一节使用。
_add(Scenario(
    name="speed_staircase",
    group="ground",
    desc="速度阶梯 0.5→2.5 m/s（每档 1.5 s 斜坡 + 4 s 保持），用于速度跟踪图",
    duration=31.0,
    velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.5, 0.5), (6.5, 0.5), (8.0, 1.0),
                       (12.0, 1.0), (13.5, 1.5), (17.5, 1.5), (19.0, 2.0), (23.0, 2.0),
                       (24.5, 2.5), (28.5, 2.5), (30.0, 0.0))),
    windows={"hold_0p5": (5.1, 6.5), "hold_1p0": (10.6, 12.0), "hold_1p5": (16.1, 17.5),
             "hold_2p0": (21.6, 23.0), "hold_2p5": (27.1, 28.5)},
))


# === B. 地形 =================================================================

for _h_mm, _h, _v, _tag in ((20, 0.020, 0.30, "ramp_20_03"),
                            (40, 0.040, 0.30, "ramp_40_03"),
                            (65, 0.065, 0.30, "ramp_65_03"),
                            (65, 0.065, 0.45, "ramp_65_045")):
    _add(Scenario(
        name=_tag,
        group="terrain",
        desc=f"左轮 {_h_mm} mm 单轮梯形坡 @ {_v} m/s（单腿变高度越障）",
        duration=9.0,
        terrain="ramp",
        terrain_kwargs={"terrain_side": "left", "terrain_height": _h},
        velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.0, _v), (8.0, _v), (9.0, 0.0))),
        windows={"obstacle": (3.0, 5.0), "whole": (1.0, 8.0)},
    ))

_add(Scenario(
    name="pad_static_10mm",
    group="terrain",
    desc="静态单轮垫高 10 mm：左轮骑垫、右轮在平地，检验找平",
    duration=6.0,
    terrain="ramp",
    terrain_kwargs={"terrain_side": "left", "terrain_height": 0.010},
    windows={"steady": (1.0, 6.0)},
))

_add(Scenario(
    name="ramp_65_03_right",
    group="terrain",
    desc="右轮 65 mm 单轮梯形坡 @ 0.30 m/s（检验单腿变高度的左右对称性）",
    duration=9.0,
    terrain="ramp",
    terrain_kwargs={"terrain_side": "right", "terrain_height": 0.065},
    velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.0, 0.3), (8.0, 0.3), (9.0, 0.0))),
    windows={"obstacle": (3.0, 5.0), "whole": (1.0, 8.0)},
))

# 路面几何见 WavyRoadTerrain：y ∈ [1.00, 5.00]，4.0 m / 0.40 m 波长 = 10 个完整波。
# 速度曲线四条速度档统一：t<1 s 站定，1→2 s 线性提速到目标速度（位移 v/2），
# 之后匀速走完 4.0 m，出路面 0.6 s 后开始减速停车。入路时刻随速度变化
# （t_entry = 2 + (y_start - v/2)/v），所以时长不同：0.30 m/s 要走 13.3 s，1.0 m/s 只要 4.0 s。
WAVY_ROAD = WavyRoadTerrain()
_WAVY_ENTRY_Y = WAVY_ROAD.y_start
_WAVY_EXIT_Y = WAVY_ROAD.y_start + WAVY_ROAD.length


def _wavy_times(v: float) -> tuple[float, float]:
    """(入路时刻, 出路面时刻)，按标称速度解析；实测入路点由出图脚本从 base_y 反查。"""
    entry = 2.0 + (_WAVY_ENTRY_Y - 0.5 * v) / v
    return entry, entry + WAVY_ROAD.length / v


for _v, _tag in ((0.30, "0p3"), (0.50, "0p5"), (0.80, "0p8"), (1.00, "1p0")):
    _entry, _exit = _wavy_times(_v)
    _end = round(_exit + 1.6, 1)          # 出路面后再留 1.6 s 观察恢复
    _add(Scenario(
        name=f"wavy_{_tag}",
        group="terrain",
        desc=f"波浪路（4.0 m，波峰 36~60 mm）@ {_v} m/s（连续地形跟随）",
        duration=_end,
        terrain="wavy",
        terrain_kwargs={"wavy_road": WAVY_ROAD},
        velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.0, _v),
                           (_end - 1.0, _v), (_end, 0.0))),
        windows={"wavy": (round(_entry - 0.2, 2), round(_exit + 0.2, 2)),
                 "whole": (1.0, _end)},
    ))


# === C. 前馈对照（论文 Figure 4a 的三条线） ==================================

for _scale, _tag in ((0.0, "scale0p0"), (0.8, "scale0p8"), (1.0, "scale1p0")):
    _add(Scenario(
        name=f"ff_ramp65_{_tag}",
        group="ff",
        desc=f"65 mm 单轮坡 @ 0.3 m/s，关节角速度前馈缩放 {_scale}（对照）",
        duration=9.0,
        terrain="ramp",
        terrain_kwargs={"terrain_side": "left", "terrain_height": 0.065},
        velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.0, 0.3), (8.0, 0.3), (9.0, 0.0))),
        overrides={"vmc.stand_rate_ff_scale": _scale},
        windows={"obstacle": (3.0, 5.0)},
    ))


# === C2. 单腿变高度（5 m 梯形坡）前馈对照 × 三个坡高 =========================
# 对应交互式场景
#   .venv/bin/python -m src.launch_mujoco --scenario stand --terrain ramp \
#       --terrain-side left --terrain-height 0.065
# 的脚本版：地形几何与手柄那套完全一致（0.50 m 上坡 + 4.00 m 台面 + 0.50 m
# 下坡，前缘 y=0.22 m），只把速度曲线固定下来。
#
# 三个坡高 20 / 40 / 65 mm 的**坡长与台面长度完全相同**（只改高度），速度曲线
# 也逐点相同，因此三档的时间轴可直接对齐叠加；同一坡高内两次实验唯一的差别
# 就是 vmc.stand_rate_ff_scale（关节角速度前馈缩放）。
_SINGLE_LEG_HEIGHT_MM = (20, 40, 65)

for _h_mm in _SINGLE_LEG_HEIGHT_MM:
    _ramp = SingleWheelTrapezoidTerrain(
        side="left",
        height=_h_mm / 1000.0,
        ramp_length=0.50,
        platform_length=4.00,
        y_start=0.22,
    )
    for _scale, _tag, _label in ((0.0, "off", "关"), (1.0, "on", "开")):
        _add(Scenario(
            name=f"single_leg_h{_h_mm}_ff_{_tag}",
            group="height",
            desc=f"单腿变高度：{_h_mm} mm 单轮梯形坡 @ 0.30 m/s，关节角速度前馈{_label}"
                 f"（vmc.stand_rate_ff_scale={_scale}）",
            duration=22.0,
            terrain="ramp",
            terrain_kwargs={"ramp": _ramp},
            velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.0, 0.30),
                               (20.0, 0.30), (22.0, 0.0))),
            overrides={"vmc.stand_rate_ff_scale": _scale},
            # 过坡段：y=0.22 m 在 t≈2.23 s，y=5.22 m 在 t≈18.9 s（0.30 m/s 匀速段）
            windows={"obstacle": (2.5, 18.5)},
        ))


# === D. 跳跃：跳上 0.20 m 台阶（论文 4.4 节唯一的跳跃场景） ====================
#
# 一条场景，三个观测量：车速（速度保持）、轮子高度（越障净空）、俯仰角速度（姿态代价）。
# 台阶几何与交互式命令
#     .venv/bin/python -m src.launch_mujoco --terrain step --step-height 0.20
# 完全一致（JumpStepTerrain：1.00 m 宽、20 m 长、前缘 y=1.00 m）。
#
# 触发距离 0.42 m 是标定出来的：0.20 m 台阶要求轮心越过前缘时高于 0.27 m
# （台阶高 0.20 + 轮半径 0.07），而本机原地跳的轮心峰值只有 0.287 m，容差很小，
# 所以起跳点必须落在台阶前缘前 0.40~0.44 m（车速 0.8 m/s）这一窄窗口内。
_add(Scenario(
    name="jump_step20",
    group="jump",
    desc="0.20 m 台阶：0.8 m/s 行驶中起跳（距前缘 0.42 m 触发），跳上后继续行驶",
    duration=9.0,
    terrain="jump_step",
    terrain_kwargs={"jump_step": JumpStepTerrain(height=0.20)},
    velocity=Schedule(((0.0, 0.0), (0.5, 0.0), (2.0, 0.8), (9.0, 0.8))),
    jump_at_y=(1.00 - 0.42,),
    jump_amplitude=1.0,
    windows={"cruise": (1.0, 1.9), "flight": (3.1, 3.7), "after_step": (5.0, 9.0)},
))


def groups() -> list[str]:
    return sorted({scn.group for scn in SCENARIOS.values()})


def select(names: list[str] | None = None, group_names: list[str] | None = None) -> list[Scenario]:
    """按名字或分组挑选场景；都不给则返回全部。"""
    picked: list[Scenario] = []
    if names:
        for name in names:
            if name not in SCENARIOS:
                raise KeyError(f"unknown scenario: {name!r}; use --list to see available cases")
            picked.append(SCENARIOS[name])
    if group_names:
        for g in group_names:
            matched = [s for s in SCENARIOS.values() if s.group == g]
            if not matched:
                raise KeyError(f"unknown group: {g!r}; available: {', '.join(groups())}")
            for s in matched:
                if s not in picked:
                    picked.append(s)
    if not picked:
        picked = list(SCENARIOS.values())
    return picked
