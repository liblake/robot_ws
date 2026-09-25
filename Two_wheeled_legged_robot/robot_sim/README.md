# wheel_legged_robot_sim

轮腿机器人的 MuJoCo 仿真。机器人左右各一条四连杆并联腿，腿端各装一个驱动轮；控制上用 LQR + VMC 组合，能自平衡站立、行走，也能起跳。

## 算法简介

控制器按跳跃相位把平衡和腿部动作分开管，免得两个环在腿电机上抢量程（代码里叫相位独占）。

平衡用 LQR。状态取切空间 5 维 `[pitch, pitch_rate, roll, roll_rate, wheel_vel]`，解出两路虚拟控制：前进轮力矩和左右腿差分。外面再套几层 PI/PD：`pitch_lean` 把速度误差转成目标俯仰，`yaw` 管转向，`heading_hold` 锁航向，零速时位置外环把车拉回原地。

腿高用 VMC。每条腿是四连杆闭链，先查 LUT 把目标腿高换成电机角，再在关节空间做 PD 跟踪。LAND 阶段单独给一组增益，P 更软、D 更硬，专门吸冲击。

跳跃是一台相位机：`STAND → CROUCH → EXTEND → FLIGHT → LAND`，摔倒了进 `FALLEN`。STAND 时 LQR 全开、VMC 管高度；CROUCH、EXTEND、LAND 期间 LQR 只靠轮前进和 yaw 稳住本体，腿整个交给 VMC 轨迹；FLIGHT 把腿和轮都置零，靠角动量在空中保持姿态，这样落地不会反扭。

参数和调参顺序都集中在 `src/controllers/default_params.py`，注释里写了该从哪项开始、按什么顺序往下冻结。

## 环境搭建

依赖用 [uv](https://docs.astral.sh/uv/) 管理，需要 Python 3.13。

```bash
uv sync
```

> macOS 上 MuJoCo 的交互式查看器必须用 `mjpython` 启动（`uv sync` 会一并装好）。Linux/Windows 用普通 `python` 即可。

## 快速开始

一般只需要直接启动带控制器的仿真：

```bash
# macOS
uv run mjpython -m src.launch_mujoco
# Linux / Windows
uv run python -m src.launch_mujoco
```

默认启动后就会加载机器人、地形和控制器。手柄接上以后可以直接控制行进和跳跃；没有手柄时也能正常打开仿真。

如果只想看 URDF 模型本身，不加载控制器，可以运行：

```bash
uv run mjpython launch_viewer.py
```

## 手柄控制（可选）

手柄默认启用。**左摇杆上下控制前进/后退（满杆 ±3.0 m/s），右摇杆左右控制转向（满杆 ±0.6 rad/s）**，RT/LT 升降站高（0.31~0.42 m），A 键触发跳跃（需要 STAND 相位停留 ≥1.2 s 且机身稳定，否则本次按键被丢弃并打印原因）。未接手柄或后端不可用时自动跳过，不影响仿真；显式关闭用 `--no-enable-gamepad`。

- macOS：接实体 Xbox 手柄需要额外依赖：

  ```bash
  uv sync --extra gamepad
  ```

- Linux：无需额外依赖。内核把兼容手柄（例如 xpad 驱动的 Xbox360 风格 2.4G 手柄）暴露为标准 Linux 摇杆 `/dev/input/js*`，程序启动时自动打开第一个可用设备。

> `launch_mujoco` 目前没有 `--gamepad-device` 参数（只有 `python -m src.gamepad` 支持按路径探测）。需要指定非第一个设备时，请先断开其它手柄，或给 `launch_mujoco.open_gamepad()` 传入 `device_path`。

需要确认轴/按键映射时，可运行 `python -m src.gamepad [设备路径]` 实时打印手柄事件与映射结果。

## 第四章场景的单独运行命令

论文第 4 章的四组可视化场景各有一条命令，都在 `robot_sim/` 下执行（`.venv/bin/python` 等价于 `uv run python`）：

```bash
# 1. 平地运动（左摇杆前后 → 前进/后退，松杆自动刹停）
.venv/bin/python -m src.launch_mujoco --scenario stand --flat-ground

# 2. 转向（右摇杆左右 → 原地/行进转向，满杆 ±0.6 rad/s）
.venv/bin/python -m src.launch_mujoco --scenario stand --flat-ground

# 3. 单腿变高度：只有单轮梯形坡，左轮（--terrain-side right 换右轮）上坡
.venv/bin/python -m src.launch_mujoco --scenario stand --terrain ramp --terrain-side left --terrain-height 0.065

# 4. 地形适应：只有波浪路，默认 1.2 m 宽 × 4.0 m 长（10 个波，波峰 36~60 mm）
.venv/bin/python -m src.launch_mujoco --scenario stand --terrain wavy

# 5. 跳跃（A 键触发；加 --scenario jump 则启动即自动跳一次）
.venv/bin/python -m src.launch_mujoco --scenario stand --flat-ground

# 6. 跳上台阶（默认 5 cm，--step-height 可改 0.10 / 0.15）
.venv/bin/python -m src.launch_mujoco --scenario stand --terrain step --step-height 0.05
```

地形参数：`--terrain {ramp,wavy,ramp_wavy,step,none}`，`--flat-ground` 等价于 `none`。
`ramp_wavy` 是旧 `--terrain ramp` 的"坡 + 波浪路"合体行为。

单轮梯形坡可用 `--ramp-length`（沿 y 的总长度，默认 5 m）、`--ramp-slope-length`（单个上下坡斜段长度，默认 0.50 m，65 mm 高对应约 7.4° 坡度）、`--ramp-y-start`（前缘位置，默认 0.22 m）调整；台面长度由前两者推出，即 `--ramp-length - 2 × --ramp-slope-length`（默认 4.0 m）。想完全复现论文里那个 0.65 m 的小坡（18° 坡度），用 `--ramp-length 0.65 --ramp-slope-length 0.20`。在 `ramp_wavy` 下，如果波浪路的 `--wavy-y-start` 落在梯形坡范围内，程序会自动把它顺延到坡后 0.5 m 并打印一行提示。

波浪路（washboard，横波：沿 x 同相、沿 y 波浪）尺寸 2026-09-21 按本机几何重标定，默认
**1.2 m 宽 × 4.0 m 长 / 0.40 m 波长 / 波峰 36~60 mm / 波谷贴地**（见 `WavyRoadTerrain` 的文档字符串）：
沿 x 两侧的淡出带会让路面宽度打八折才是"全幅"区，而本机两条轮迹跨 x ∈ [-0.05, 0.39]，
所以宽度必须 ≥ 0.6 m 才不至于让某个轮子骑在淡出带上——旧默认 0.44 m 宽 / x 中心 0.20 m
时左轮几乎全程走平地，左右轮不同相，量出来的"侧倾"是几何 bug。`--wavy-y-start` 默认 1.00 m
（起点先走一段平地），`--wavy-amplitude` 默认 0.03 m（半峰峰值），`--wavy-wavelength` 默认 0.40 m。
波长按轮半径 0.07 m 选：λ=0.35 m 时 1.0 m/s 通过波峰所需向心加速度 v²/R_c≈9.6 m/s²≈g，
轮子开始离地；0.40 m 时约 7 m/s²，仍贴地。高度场网格按 `--wavy-length` / `--wavy-width`
以 5 mm × 36.7 mm 的格子自动推算（默认 4.0 m × 1.2 m 对应 801 × 33），也可以用
`--wavy-nrow` / `--wavy-ncol` 手动指定。

关节角速度前馈可以用 `--rate-ff-scale`（对应 `vmc.stand_rate_ff_scale`）在启动时开关：`1.0` = 默认（前馈开），`0.0` = 前馈关。做论文 4.3 节那个"单腿变高度前馈对照"时，用同一套地形和速度各跑一次：

```bash
.venv/bin/python -m src.launch_mujoco --scenario stand --terrain ramp --terrain-height 0.065 --rate-ff-scale 0
.venv/bin/python -m src.launch_mujoco --scenario stand --terrain ramp --terrain-height 0.065 --rate-ff-scale 1
```

## 单腿变高度对比实验（场景运行器版）

关节角速度前馈的"关 / 开"对照不用手柄跑——手动推杆两次速度对不齐，改成固定速度曲线的场景。
坡高分 20 / 40 / 65 mm 三档，每档各跑前馈关、前馈开，共六个场景（`single_leg_h{20,40,65}_ff_{off,on}`）：

```bash
# 六个场景一次跑完（约 1 分钟）
.venv/bin/python -m paper.run_cases --group height --out "paper/data/单腿变高度"

# 只要 65 mm 那一档
.venv/bin/python -m paper.run_cases --case single_leg_h65_ff_off --case single_leg_h65_ff_on \
    --out "paper/data/单腿变高度"

# 只画 65 mm 对照（横轴时间，上栏速度、下栏侧倾角），输出到 paper/figures/
.venv/bin/python paper/figures/fig_single_leg_height.py

# 画三档坡高的侧倾时程对比（前馈关一张、前馈开一张，另加并排对照），输出到 paper/figures/
.venv/bin/python paper/figures/fig_single_leg_height_roll_3h.py
```

三档的地形与手柄那套完全一致（0.50 m 上坡 + 4.00 m 台面 + 0.50 m 下坡，前缘 y=0.22 m，左轮），
只改坡高、坡长与速度曲线逐点相同，速度固定为 0→0.30 m/s 匀速走完 5 m 坡，
同一坡高内两次唯一的差别就是 `vmc.stand_rate_ff_scale`。数据落在 `paper/data/单腿变高度/`：

| 文件 | 内容 |
|---|---|
| `single_leg_h{20,40,65}_ff_{off,on}.csv` | 500 Hz 逐步遥测（含 `t`、`v_fwd`、`roll_deg`、`pitch_deg`、`base_y`、力矩、接触数等 27 列） |
| `single_leg_h{20,40,65}_ff_{off,on}.json` | 汇总指标：过坡段侧倾峰峰值、最大侧倾、最大俯仰、平均速度、离地占比、力矩峰 |
| `roll_3h_metrics.csv` | 三档 × 前馈关/开的论文数字表（由 `fig_single_leg_height_roll_3h.py` 生成） |
| `index.json` | 该目录内六次实验的索引 |

当前实测（过坡段 t=2.50~18.95 s）：

| 坡高 | 前馈 | 平均速度 | 侧倾峰峰值 | 最大侧倾 | 最大俯仰 |
|---|---|---|---|---|---|
| 20 mm | 关 | 0.3042 m/s | 0.059° | 0.034° | 2.83° |
| 20 mm | 开 | 0.3041 m/s | 0.010° | 0.007° | 2.82° |
| 40 mm | 关 | 0.3043 m/s | 0.252° | 0.137° | 3.09° |
| 40 mm | 开 | 0.3038 m/s | 0.033° | 0.027° | 2.90° |
| 65 mm | 关 | 0.3039 m/s | 1.960° | 1.137° | 3.86° |
| 65 mm | 开 | 0.3036 m/s | 0.056° | 0.047° | 3.79° |

六次速度一致到小数点后三位、全程无离地，侧倾峰峰随坡高单调增（0.059° → 0.252° → 1.960°），
前馈开后被压到 0.01~0.06° 量级且仍随坡高有序，俯仰基本不变——正好把差异锁定在关节角速度前馈上。

想要更大的对比度（论文那张 0.65 m 陡坡的图是 8.38° vs 0.068°），可以改 `paper/scenarios.py` 里 `_SINGLE_LEG_HEIGHT_RAMP` 的坡形或 `velocity` 的速度曲线——坡度越陡、速度越高，腿需要的关节角速度越大，前馈的作用越明显。

## 手柄手动跑的数据存在哪、怎么分析

每次启动 `launch_mujoco` 都会新建一个运行目录：

```
logs/manual/run_20260919_191815/
  telemetry.csv   # 500 Hz 逐步遥测（time/位姿/姿态/速度/轮速/接触数/target_info/控制量）
  system.log      # 本次的启动参数：地形几何、前馈档位、手柄名等
```

**只保留最近 4 次运行**（`cleanup_old_logs`），要做对比实验记得把要用的目录先复制出去。另外手动日志里 `roll` / `pitch` 两列的单位是**弧度**，不是论文 CSV 里的度数。

分析脚本：

```bash
# 单次：列出这一趟过坡时侧倾有多大
.venv/bin/python analyze_manual_log.py logs/manual/run_20260919_191815

# 前馈关 vs 开 两次对比，只看 5 m 梯形坡那一段（0.22~5.22 m）
.venv/bin/python analyze_manual_log.py logs/manual/run_第一个 --label "ff off" \
    logs/manual/run_第二个 --label "ff on" \
    --x y --window-y 0.22:5.22 --pass 1 --out /tmp/ff_compare.png
```

`--window-y` 会先把手动日志切成若干趟"窗口内持续前进"的穿越并逐趟打印 `侧倾峰峰值 / 最大侧倾 / 最大俯仰 / 平均速度 / 离地步数`，用 `--pass N` 挑一趟来画。注意手动数据不可重复，只适合自查和演示；论文里那张前馈对照图建议用可复现的场景运行器产出：

```bash
.venv/bin/python -m paper.run_cases --case ff_ramp65_scale0p0 --case ff_ramp65_scale1p0
.venv/bin/python paper/figures/fig5_terrain.py
```

## 项目结构

```
src/
  controllers/     # LQR、VMC、相位机、跳跃轨迹、参数与 LUT
  robot/           # robot.urdf + STL 网格
  launch_mujoco.py # 带控制器的仿真入口
  mjcf_builder.py  # URDF → MJCF 转换与预处理
  rollout.py       # 无头 rollout
  optimize.py      # Optuna 调参
launch_viewer.py   # 纯 URDF 查看器
profile_sim.py     # 性能分析
```

## 许可证

[Apache-2.0](LICENSE)
