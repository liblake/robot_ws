"""停车后的平衡响应回归测试。

复现 test_drive.py 的 前进 -> 停车 -> 后退 -> 停车 曲线，验证指令回零时：

1. 前后向轮力矩不出现单周期阶跃（这是停车时被感知到的"顿挫"）；
2. 腿力矩不出现改动前的 2~3.6 N.m 级阶跃；
3. 机器人不再朝反方向倒退（"减速到 0 时往回退一段才停"）；
4. 停车后能在限时内收敛到静止。

阈值来自 2026-09-11 与改动前版本 (b80cfa1) 的实测对照：

    改动前: 轮跳变 0.69/1.01 N.m, 腿跳变 3.59/2.14 N.m,
            反向倒退 83/61 mm, 停车后 5 s 仍未收敛;
    修复后: 轮跳变 0.05/0.04 N.m, 腿跳变 0.36/0.32 N.m,
            反向倒退 <1 mm, 0.9 s 内收敛。
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import NamedTuple

import mujoco
import numpy as np

from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.state import extract_sim_state


# 停车事件: (名称, 外部指令回零时刻, 停车保持窗口结束时刻)
STOP_EVENTS = (
    ("forward stop (+0.5 -> 0)", 7.25, 8.25),
    ("reverse stop (-0.5 -> 0)", 13.25, 18.0),
)

# 阈值取自改动前/后的实测对照，留出约 5 倍余量。
WHEEL_TORQUE_STEP_LIMIT = 0.25    # N.m, 前后向轮力矩单周期跳变
LEG_TORQUE_STEP_LIMIT = 1.0       # N.m, VMC 腿力矩（刹车阶段本身有 ~0.3 N.m 抖动）
REVERSE_DISPLACEMENT_LIMIT = 0.04  # m, 指令回零后朝反方向移动的距离
SETTLE_TIME_LIMIT = 1.5           # s, 从指令回零到速度保持收敛所需时间
SETTLE_VELOCITY = 0.01            # m/s
STOP_EDGE_WINDOW = 0.15           # s, 统计力矩跳变的窗口


class Sample(NamedTuple):
    time: float
    position: float
    velocity: float
    control: np.ndarray
    previous_control: np.ndarray | None


def velocity_profile(t: float) -> float:
    """与 test_drive.py 相同的前进、停车、后退、停车曲线。"""
    t -= 1.0
    if t < 0.0:
        return 0.0
    if t < 1.25:
        return 0.4 * t
    if t < 5.0:
        return 0.5
    if t < 6.25:
        return max(0.0, 0.5 - 0.4 * (t - 5.0))
    if t < 7.25:
        return 0.0
    if t < 8.5:
        return -0.4 * (t - 7.25)
    if t < 11.0:
        return -0.5
    if t < 12.25:
        return max(-0.5, -0.5 + 0.4 * (t - 11.0))
    return 0.0


def _settle_time(samples: list[Sample], t_start: float, t_end: float) -> float | None:
    """返回指令回零后速度进入并保持死区所需的秒数，未收敛时返回 None。"""
    window = [s for s in samples if t_start <= s.time <= t_end]
    for index, sample in enumerate(window):
        if all(abs(item.velocity) <= SETTLE_VELOCITY for item in window[index:]):
            return sample.time - t_start
    return None


def _stop_metrics(samples: list[Sample], t_stop: float, t_end: float) -> dict[str, float | None]:
    edge = [
        s for s in samples
        if s.previous_control is not None and t_stop <= s.time <= t_stop + STOP_EDGE_WINDOW
    ]
    window = [s for s in samples if t_stop <= s.time <= t_end]
    entry = min(window, key=lambda s: abs(s.time - t_stop))

    # 指令回零瞬间的运动方向；反方向位移即为用户观察到的"往回退"。
    travel_direction = 1.0 if entry.velocity >= 0.0 else -1.0
    reverse_displacement = max(
        -travel_direction * (s.position - entry.position) for s in window
    )
    return {
        "wheel_step": max(
            max(
                abs(float(s.control[0] - s.previous_control[0])),
                abs(float(s.control[1] - s.previous_control[1])),
            )
            for s in edge
        ),
        "leg_step": max(
            max(abs(float(s.control[i] - s.previous_control[i])) for i in range(2, 6))
            for s in edge
        ),
        "reverse_displacement": reverse_displacement,
        "settle_time": _settle_time(samples, t_stop, t_end),
        "final_velocity": window[-1].velocity,
    }


def main() -> int:
    xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id == -1:
        raise RuntimeError("missing stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)

    controller = CombinedController(copy.deepcopy(STAND_PARAMS))
    samples: list[Sample] = []
    previous_control: np.ndarray | None = None
    for _ in range(int(18.0 / model.opt.timestep)):
        state = extract_sim_state(model, data)
        controller.params.target_velocity = velocity_profile(float(data.time))
        control = controller(model, data, state)
        samples.append(
            Sample(
                time=float(data.time),
                position=float(state.base_position[1]),
                velocity=controller._project_forward_body_velocity(model, data, state),
                control=control.copy(),
                previous_control=None if previous_control is None else previous_control.copy(),
            )
        )
        previous_control = control.copy()
        data.ctrl[: model.nu] = control
        mujoco.mj_step(model, data)

    for name, t_stop, t_end in STOP_EVENTS:
        metrics = _stop_metrics(samples, t_stop, t_end)
        print(
            f"{name}: wheel_step={metrics['wheel_step']:.3f} N.m, "
            f"leg_step={metrics['leg_step']:.3f} N.m, "
            f"reverse_displacement={metrics['reverse_displacement'] * 1000:+.2f} mm, "
            f"settle_time={metrics['settle_time']:.3f} s, "
            f"final_v={metrics['final_velocity']:+.4f} m/s"
        )
        if metrics["wheel_step"] > WHEEL_TORQUE_STEP_LIMIT:
            raise AssertionError(
                f"{name}: wheel torque step is too large: "
                f"{metrics['wheel_step']:.3f} N.m"
            )
        if metrics["leg_step"] > LEG_TORQUE_STEP_LIMIT:
            raise AssertionError(
                f"{name}: leg torque step is too large: {metrics['leg_step']:.3f} N.m"
            )
        if metrics["reverse_displacement"] > REVERSE_DISPLACEMENT_LIMIT:
            raise AssertionError(
                f"{name}: robot rolled backwards "
                f"{metrics['reverse_displacement'] * 1000:.1f} mm after the stop command"
            )
        if metrics["settle_time"] is None or metrics["settle_time"] > SETTLE_TIME_LIMIT:
            raise AssertionError(
                f"{name}: did not settle within {SETTLE_TIME_LIMIT} s "
                f"(settle_time={metrics['settle_time']})"
            )
        if abs(metrics["final_velocity"]) > SETTLE_VELOCITY:
            raise AssertionError(
                f"{name}: final velocity is not zero: {metrics['final_velocity']:.4f} m/s"
            )

    if abs(controller._velocity_integral) > 1e-4:
        raise AssertionError("velocity integral was not released during zero command")
    print("PASS: both stop events are smooth, non-reversing and settle to rest")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
