from __future__ import annotations  

from collections.abc import Callable  
from dataclasses import dataclass  

import mujoco  
import numpy as np  
from scipy.linalg import solve_discrete_are  

from src.state import SimState  # 从项目的 state 模块导入仿真状态数据结构


TangentStateProvider = Callable[[mujoco.MjModel, mujoco.MjData, SimState], np.ndarray]  # 定义切线状态提供函数的类型别名


@dataclass  # 将 LqrController 标记为数据类，自动生成初始化与表示等方法
class LqrController:  # 定义 LQR 控制器数据类
    gain: np.ndarray  # 状态反馈增益矩阵 K
    target: np.ndarray  # 目标切线状态向量
    feedforward: np.ndarray  # 前馈控制量向量
    tangent_state_provider: TangentStateProvider | None = None  # 可选的切线状态提取函数，默认未提供

    @classmethod  # 声明 zero_gain 是类方法，可通过类本身调用
    def zero_gain(cls, model: mujoco.MjModel) -> LqrController:  # 根据模型自由度构造零增益控制器
        return cls(  # 使用当前类构造并返回控制器实例
            gain=np.zeros((model.nu, 2 * model.nv)),  # 将增益矩阵初始化为零，形状为执行器数乘以两倍速度自由度
            target=np.zeros(2 * model.nv),  # 将目标状态初始化为零向量，长度与状态空间一致
            feedforward=np.zeros(model.nu),  # 将前馈控制量初始化为零向量，长度等于执行器数
        )  

    def __call__(self, model: mujoco.MjModel, data: mujoco.MjData, state: SimState) -> np.ndarray:  # 使控制器实例可像函数一样调用
        if not np.all(np.isfinite(self.gain)) or not np.all(np.isfinite(self.target)) or not np.all(np.isfinite(self.feedforward)):  # 检查增益、目标与前馈量是否全部为有限数值
            raise ValueError("LQR gain, target, and feedforward must be finite")  # 若有 NaN 或无穷大，抛出数值错误
        if self.tangent_state_provider is None:  # 未提供切线状态提取函数时，进入零增益分支
            if np.any(self.gain):  # 零增益分支中若增益矩阵含有非零元素
                raise ValueError("tangent_state_provider is required for nonzero LQR gain")  
            current = np.zeros(self.gain.shape[1]) 
        else:  
            current = self.tangent_state_provider(model, data, state)  # 调用该函数计算当前切线状态
        if current.shape != self.target.shape:  # 检查当前状态与目标状态的形状是否一致
            raise ValueError(f"LQR state shape must be {self.target.shape}, got {current.shape}")  
        if not np.all(np.isfinite(current)):  
            raise ValueError("LQR state must be finite")  
        raw_control = self.feedforward - self.gain @ (current - self.target) 
        if raw_control.shape == (model.nu,): 
            clipped = np.clip(raw_control, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])  
        else:  
            clipped = raw_control  
        if not np.all(np.isfinite(clipped)):  
            raise ValueError("LQR control must be finite")  
        return clipped  


def solve_discrete_lqr(  # 定义求解离散 LQR 反馈增益的函数
    a: np.ndarray,  # 离散系统状态矩阵 A
    b: np.ndarray,  # 离散系统输入矩阵 B
    q: np.ndarray,  # 状态权重矩阵 Q
    r: np.ndarray,  # 控制权重矩阵 R
) -> np.ndarray:  # 返回状态反馈增益矩阵 K
    p = solve_discrete_are(a, b, q, r)  # 求解离散代数 Riccati 方程，得到解矩阵 P
    return np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)  # 按 LQR 增益公式计算并返回 K
