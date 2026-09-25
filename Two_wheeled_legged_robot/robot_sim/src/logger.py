from __future__ import annotations

import csv
import logging
import shutil
from pathlib import Path
import numpy as np

from src.state import SimState

def setup_system_logger(log_path: Path, console: bool = True) -> logging.Logger:
    """初始化标准文本日志，用于记录事件和配置。"""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    
    logger = logging.getLogger("sim")
    logger.setLevel(logging.INFO)
    
    # 清理可能存在的旧 handlers
    logger.handlers.clear()
    
    formatter = logging.Formatter('%(asctime)s - %(process)d - %(levelname)s - %(message)s')
    
    file_handler = logging.FileHandler(log_path, mode='a')
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    
    if console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
        
    # 防止向 root logger 传递
    logger.propagate = False
    return logger


def _rpy_from_quaternion(q: np.ndarray) -> tuple[float, float]:
    """从四元数提取 roll 和 yaw 角（单位: 弧度）。

    使用 ZYX 内旋（航空）顺序。pitch 已在 SimState 中独立计算，此处不重复。

    警告：本函数返回的 roll 是**绕本体 X 轴**的旋转。本机前进轴是 +Y、轮轴是 X，
    所以绕 X 的旋转其实就是俯仰角，不是侧倾角。真正与 LQR 约定一致的侧倾角是
    "绕本体 Y 轴"的旋转，存在 SimState.roll（见 src/state.py::_roll_from_quaternion）。
    遥测里的 roll 列请直接用 state.roll，不要用本函数的返回值。

    Args:
        q: 四元数 [w, x, y, z]。

    Returns:
        (绕本体 X 轴的旋转角, yaw) 元组。
    """
    w, x, y, z = q
    # roll (x 轴旋转)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = float(np.arctan2(sinr_cosp, cosr_cosp))
    # yaw (z 轴旋转)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = float(np.arctan2(siny_cosp, cosy_cosp))
    return roll, yaw


class TelemetryLogger:
    """高频物理状态遥测日志，保存为 CSV 文件。"""
    def __init__(self, log_path: Path):
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.file = open(self.log_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        
        # 写入表头
        self.writer.writerow([
            "time",
            "pos_x", "pos_y", "pos_z",
            "quat_w", "quat_x", "quat_y", "quat_z",
            "roll", "pitch", "yaw",
            "vel_x", "vel_y", "vel_z",
            "ang_vel_x", "ang_vel_y", "ang_vel_z",
            "pitch_rate",
            "wheel_pos_l", "wheel_pos_r",
            "wheel_vel_l", "wheel_vel_r",
            "wheel_z_l", "wheel_z_r",
            "contact_count",
            "target_info",
            "control_output",
        ])
        
    def log_step(
        self,
        time: float,
        state: SimState,
        target_info: str,
        control: np.ndarray,
        wheel_z: dict[str, float] | None = None,
    ) -> None:
        """记录每帧的关键数据，包含完整 6-DOF pose。

        wheel_z 是轮心在世界系下的 z（m），由调用方用
        ``src.geometry.wheel_center_z`` 从 MuJoCo 模型读出后传入；不传时写空。
        不要用 base_z 减腿高指令反推：腿高控制器有毫米级稳态误差，而越障净空
        本身就是毫米级，反推会把控制误差直接算进结论里。
        """
        q = state.base_quaternion
        # 注意：roll 取 state.roll（绕本体 Y 轴的真实侧倾角），不能用
        # _rpy_from_quaternion 的返回值——那是绕本体 X 轴的旋转，对本机来说等于俯仰，
        # 会让遥测里 roll 与 pitch 两列变成同一个量（历史 bug，2026-09-16 修）。
        _roll_about_x, yaw = _rpy_from_quaternion(q)
        roll = float(state.roll)
        av = state.base_angular_velocity
        row = [
            f"{time:.4f}",
            f"{state.base_position[0]:.4f}", f"{state.base_position[1]:.4f}", f"{state.base_position[2]:.4f}",
            f"{q[0]:.6f}", f"{q[1]:.6f}", f"{q[2]:.6f}", f"{q[3]:.6f}",
            f"{roll:.4f}", f"{state.pitch:.4f}", f"{yaw:.4f}",
            f"{state.base_linear_velocity[0]:.4f}", f"{state.base_linear_velocity[1]:.4f}", f"{state.base_linear_velocity[2]:.4f}",
            f"{av[0]:.4f}", f"{av[1]:.4f}", f"{av[2]:.4f}",
            f"{state.pitch_rate:.4f}",
            f"{state.wheel_positions.get('left', 0.0):.4f}", f"{state.wheel_positions.get('right', 0.0):.4f}",
            f"{state.wheel_velocities.get('left', 0.0):.4f}", f"{state.wheel_velocities.get('right', 0.0):.4f}",
            "" if wheel_z is None else f"{float(wheel_z['left']):.5f}",
            "" if wheel_z is None else f"{float(wheel_z['right']):.5f}",
            str(state.contact_count),
            target_info,
            np.array2string(control, precision=3, separator=',', suppress_small=True)
        ]
        self.writer.writerow(row)
        
    def close(self) -> None:
        if not self.file.closed:
            self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

def cleanup_old_logs(base_dir: Path, max_keep: int = 4) -> None:
    """清理旧日志目录或文件，只保留最新的 max_keep 个。"""
    if not base_dir.exists():
        return
        
    entries = []
    for p in base_dir.iterdir():
        if p.name.startswith("run_") or p.name.startswith("opt_"):
            entries.append(p)
            
    # 按名称倒序排序（名称包含时间戳，保证最新的在前）
    entries.sort(key=lambda p: p.name, reverse=True)
    
    # 删除超出的部分
    for p in entries[max_keep:]:
        try:
            if p.is_dir():
                shutil.rmtree(p)
            else:
                p.unlink()
        except Exception as e:
            print(f"Warning: Failed to cleanup old log {p}: {e}")
