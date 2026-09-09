# 控制算法迁移成果归档（2026-09-09）

> 目标：把"开源 LQR+VMC 轮腿控制 → 你的串联双轮腿机器人"的当前稳定成果固化成档。
> 范围：MuJoCo 仿真（`Two_wheeled_legged_robot/robot_sim`），平地功能；真机未开始。

## 1. 运行环境与铁律

- 统一用项目虚拟环境运行，否则结果不可复现：
  ```bash
  cd ~/robot_ws/Two_wheeled_legged_robot/robot_sim
  .venv/bin/python <脚本>
  ```
- 不要用系统 `python3`（conda robot 环境 MuJoCo 版本不同；且关闭可视化窗口时
  该环境会段错误）。
- Wayland 下关窗口的 GLFW 段错误属环境问题；归档脚本在 viewer 模式结束后
  已用 `os._exit(0)` 规避。

## 2. 已验证稳定功能（平地）

| # | 功能 | 脚本 | 关键结果 |
|---|---|---|---|
| 1 | 模型加载/受控模型 | `launch_viewer.py` | 8 body / 6 执行器 / stand keyframe |
| 2 | 腿高跟踪 | `test_leg_height_tracking.py` | 稳态误差 ≤5mm，无越限 |
| 3 | 自平衡站立 | `test_stand_wheel_balance.py` | 12s：pitch≤0.046、roll=0、y 漂移≤8mm、双轮接地 |
| 4 | 前进/后退 | `test_drive.py` | ±0.5 m/s 梯形速度，稳定往返 |
| 5 | 原地转向 | `test_turn.py` | 0.3 rad/s 档稳定，正反净角 +0.23 rad |
| 6 | 原地调高 | `test_height.py` | 0.33↔0.45 m 误差 ≤1mm |
| 7 | 复合运动 | `test_composite.py` | 0.4 m/s + 0.2 rad/s 弧线稳定 |
| 8 | 10m 直线 | `test_straight_10m.py` | 26s；横漂 0、航向 0、pitch≤0.066 |
| 9 | 手柄控制 | `launch_mujoco --controller combined --scenario stand --flat-ground` | 前进/后退/转向/调高；符号与平滑已修 |

所有脚本均可加 `--viewer` 可视化（`test_straight_10m.py` 除外）。

## 3. 关键参数（当前默认，集中在这几个文件）

| 参数 | 值 | 位置 |
|---|---|---|
| 轮毂电机力矩 | ±9 N·m（实机峰值） | `mjcf_builder._replace_actuators` |
| 腿电机力矩 | ±40 N·m（软限 40） | `mjcf_builder` / `VmcParams.stand_torque_limit` |
| 腿 VMC PD | kp=200 / kd=30 | `default_params.py` |
| 轮心相对髋偏移 | -0.075 m（后移，平衡角≈1.6°） | `serial_leg_ik.wheel_y_offset` |
| 大腿 / 小腿长度 | 0.300 / 0.34325 m（URDF；实机小腿 0.34 已记录 `L2_REAL`） | `serial_leg_ik.py` |
| 腿高工作区间 | h_base ∈ [0.31, 0.50] m | `serial_leg_ik` / `mjcf_builder` cmd 滑块 |
| 轮子平衡 LQR | K=[-45.0, -7.0]（2 状态；0.42m 内快恢复且长时间稳定） | `CombinedParams.wheel_balance_gain_2d` |
| yaw 阻尼 | 8.0 | `default_params.py` |
| 指令斜坡限速 | 线 0.4 m/s² / 转向 0.15 rad/s² | `CombinedParams` |
| 高位限速保护 | h>0.45m 时 线速≤0.3m/s、转向≤0.2rad/s | `CombinedParams.high_height_*` |
| 手柄/滑块指令范围 | 线 ±0.5 m/s、转向 ±0.3 rad/s、高 [0.31,0.42] | `mjcf_builder` cmd 滑块 |
| 碰撞代理 | 轮子=圆柱、腿=胶囊、机身=盒 | `mjcf_builder._add_collision_proxies` |

## 4. 过程中修掉的“坑”（防止回归）

1. URDF `effort=10` 会变成关节级 `actuatorfrcrange=±10`，腿根本出不了力 → 已在 builder 中删除，由自己的 ctrlrange 管理。
2. 腿高必须用机身原点 `xpos`（不是质心 `xipos`），否则系统性偏高 11mm。
3. 简化 LQR 的轮子模型与实际单步响应差约 3.6 倍 → 2 状态经验 LQR + 实测映射。
4. 轮子方向符号与开源相反（左+1/右-1）；转向在执行器坐标中“左右同号”。
5. 球体碰撞代理会侧滚造成慢速晃动 → 改圆柱。
6. 机身前倾 13° 是质心几何造成的，不是惯量 bug → 轮心后移 75mm 调直。
7. 手柄 Y 轴方向与默认假设相反 → 映射反号；松杆阶跃会摆动 → 指令斜坡限速。
8. 爬坡失稳根因：轮速≠车身速度（爬坡/打滑时轮速虚高），外环误判“超速”
   倒拉轮子 → 速度反馈已改为机身实际前向速度。

## 5. 已知限制 / 未完成

- 速度上限：线 ±0.5 m/s、转向 ±0.3 rad/s（当前安全包线，可后续扩但需重测）。
- 高位稳定性：0.45~0.50 m 未做完整动态验证；2026-09-09 手柄实测在 0.50m +
  0.48m/s 长跑时失稳倒地，已加"高位限速联动保护"（>0.45m 自动限速 0.3m/s）；
  0.50m + 0.3m/s 松杆减速仍失稳 → 可操作上限收回 0.45m。
- 高位长期站立标定（2026-09-09）：0.42m 以下原 K[-38.2,-6.3] 120s 稳定
  （p2p≈0）；0.45m 原增益约 36s 慢发散；pitch_rate 阻尼加到 -9 虽不倒但产生
  ±0.2rad 极限环；多组增益无法在 0.45 兼顾两者 → 可操作上限设 0.42m，
  0.45+ 留待任务空间力控专项。平衡速度优化：增益调为 K[-45,-7]（扰动后
  残余晃动 p2p 0.045→0.028，0.42m 120s 稳定），站立/前进/转向/调高回归通过。
- 斜坡/roll 找平：**开发中，未完成**。已确认“地形前馈”（左右轮高差）在 3cm
  单轮坡 + 慢速配置下能把过坡 roll 峰峰从 20.2° 压到 4.6°，但结果对加减速曲线
  敏感，roll 反馈仍会失稳；需按专项继续（固定验收场景 → 遥测 → 逐层改进）。
- 斜坡专项进展（2026-09-09 后续）：速度外环改用“机身实际前向速度”后，
  右车道 2cm 坡从“摔倒倒退”改善为“能骑上坡并前进约 3m”；左车道仍失败；
  5D LQR 高度分箱通路在平地即不稳定，属移植遗留问题，暂以 2 状态经验 LQR 为
  默认。斜坡仍**未完成**，需要坡度感知目标倾角或任务空间力控级改造。
- 再尝试（2026-09-09）：增加“爬坡自适应倾角参考”（上坡时缓慢跟随地形姿态、
  出坡衰减回 0）。结果左右车道/开关注册组合互不一致（左车道无 roll 前馈时能
  前进 4.3m，右车道有前馈反而不稳），说明该启发式也不鲁棒；已保留代码但标注
  实验性，默认平地功能不受影响。
- M0 任务空间力控（2026-09-09 开始）：脚本 `test_leg_task_force.py` 已建立，
  但 Jᵀ 方向/重力前馈/执行器符号叠加未对齐，高度阶跃未通过；详见
  `SLOPE_LEG_FORCE_DESIGN.md` 第 8 节的 checkpoint。平地功能不受影响。
- 跳跃（阶段 6）：未开始，需要髋/膝电机规格。
- 真机迁移：未开始（需电机驱动接口、IMU、主控、真实质心/尺寸）。
- 地形/波浪路/wavy road 尚未按本机轮距参数化。

## 6. 文件速查

```text
Two_wheeled_legged_robot/
├── MIGRATION_GUIDE.md        # 分阶段实施指南（0~6）
├── RESULTS_ARCHIVE.md        # 本文档
├── src/robot/robot.urdf      # 你的原始模型
└── robot_sim/                # 迁移工程（可改）
    ├── src/controllers/
    │   ├── serial_leg_ik.py  # 腿几何/IK（唯一真值源）
    │   ├── vmc.py            # 串行腿 VMC
    │   ├── combined.py       # 组合控制器（含经验轮 LQR、斜坡限速）
    │   └── default_params.py # 参数唯一入口
    ├── src/mjcf_builder.py   # URDF→MJCF + 地形/碰撞/执行器/keyframe
    └── test_*.py             # 各功能验收/演示脚本
```

## 7. 当前默认启动方式（正式入口）

```bash
cd ~/robot_ws/Two_wheeled_legged_robot/robot_sim
.venv/bin/python -m src.launch_mujoco --controller combined --scenario stand --flat-ground
```
