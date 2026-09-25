# paper/ —— 论文场景运行器

给论文用的"按名字跑仿真"工具。不改控制器、不改仿真，只把已有能力组织成**可复现的命名场景**，
一次跑完自动落 CSV + JSON。

## 快速开始

```bash
cd ~/robot_ws/Two_wheeled_legged_robot/robot_sim

.venv/bin/python -m paper.run_cases --list                       # 看所有场景
.venv/bin/python -m paper.run_cases --case stand_12s             # 跑一个
.venv/bin/python -m paper.run_cases --group terrain              # 跑一组
.venv/bin/python -m paper.run_cases --group ff --group jump      # 跑多组
.venv/bin/python -m paper.run_cases --all                        # 跑全部（33 个，约 5 分钟）
.venv/bin/python -m paper.run_cases --case jump_static_1p0 --viewer   # 边看边跑
```

输出在 `paper/data/`：

- `<场景名>.csv`：逐步遥测，列为 t / 相位 / 指令 / 前向速度 / 姿态 / 腿高 / 四个腿电机力矩 / 轮力矩 / 接触数
- `<场景名>.json`：汇总指标（摔倒、俯仰侧倾峰、各分析窗口指标、弹道、蹬伸力矩峰、收腿净空、偏航漂移……）
- `index.json`：所有跑过的场景索引，一眼看全

## 临时改参数（不改代码）

```bash
# 关节角速度前馈关掉，看它对过坡的影响
.venv/bin/python -m paper.run_cases --case ramp_65_03 --set vmc.stand_rate_ff_scale=0 --tag ff_off

# 一次跑三档做对照
for s in 0 0.8 1.0; do
  .venv/bin/python -m paper.run_cases --case ramp_65_03 \
    --set vmc.stand_rate_ff_scale=$s --tag scale$s
done
```

参数用点分路径，直接对应代码里的字段，例如：

| 参数 | 作用 |
|---|---|
| `vmc.stand_rate_ff_scale` | 关节角速度前馈缩放（0=旧行为） |
| `vmc.gravity_ff_enabled` | 解析式支撑前馈开关 |
| `vmc.support_ff_include_leg_weight` | 腿连杆自重项开关 |
| `vmc.flight_tuck_enable` | 腾空收腿开关 |
| `vmc.flight_attitude_diff_enable` | 镜像髋差分驱动开关（False = 复现修正前行为） |
| `vmc.kp_land` / `vmc.kd_land` | 落地吸收增益 |
| `vmc.roll_level_offset_limit` | 单腿变高度的找平偏置上限 |
| `pitch_lean_gain` / `yaw_damping` | 速度环 / 转向通道增益 |

## 加新场景

改 `paper/scenarios.py`，往 `SCENARIOS` 里加一条即可，例如：

```python
_add(Scenario(
    name="ramp_30_05",
    group="terrain",
    desc="左轮 30 mm 坡 @ 0.5 m/s",
    duration=9.0,
    terrain="ramp",
    terrain_kwargs={"terrain_side": "left", "terrain_height": 0.030},
    velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.0, 0.5), (8.0, 0.5), (9.0, 0.0))),
    windows={"obstacle": (3.0, 5.0)},
))
```

- `terrain`：`flat` / `ramp`（单轮梯形坡）/ `wavy`（波浪路）/ `ramp_wavy` / `jump_step`（台阶）
- `velocity` / `yaw_rate` / `height`：分段线性时序，点写成 `((时刻, 值), ...)`
- `jump_times`：按时间触发跳跃；`jump_at_y`：按机身 y 触发（台阶用）
- `overrides`：这条场景自带的参数覆盖
- `windows`：分析窗口，指标只在这段时间内统计（做对照时很有用）

## 现在的场景清单（33 个）

| 分组 | 场景 |
|---|---|
| ground（13） | 站立、往返行驶、0.8/1.5/2.0/2.5/3.0 m/s 速度扫描、速度阶梯、转向、调高、复合运动、10 m 直线、松杆刹车 |
| terrain（10） | 20/40/65 mm 单轮坡（0.3 与 0.45 m/s，含右轮对称性）、静态垫高、波浪路 0.3/0.5/0.8/1.0 m/s |
| ff（3） | 65 mm 坡 × 关节角速度前馈 0 / 0.8 / 1.0（论文里的三条对照曲线） |
| height（6） | 单腿变高度：20/40/65 mm 梯形坡 × 关节角速度前馈 关/开（论文的侧倾对照图） |
| jump（1） | 0.20 m 台阶：0.8 m/s 行驶中起跳（`jump_step20`）。⚠️ 原来的跳跃实验矩阵已不在仓库里，见文末第 6 条 |

## 几点说明

1. **波形路的场景以前跑不出来**：`test_wavy_road.py` 当年没传地形（跑的是平地），
   根因是地形开关只支持"梯形坡 + 波浪路"绑定。现在 `terrain="wavy"` 可以单独跑波浪路。
2. **数字口径**：本运行器的采样在 `mj_step` 之后，与 `test_jump.py` / `test_jump_moving.py` 一致。
   已验证：原地跳弹道 133.2 mm、蹬伸膝力矩峰 58.2 N·m，与验收脚本的 133.0 / 58.1 一致。
3. **收腿净空是 213 mm**（归档里写 223 mm）：差别来自收腿深度默认值 0.35 与归档当时的 0.33，
   以本运行器的结果为准。
4. 想复现某个 `test_*.py` 的完整判据（PASS/FAIL），仍用那个脚本；本运行器是"批量取数 + 出图数据"用的。
5. **波浪路的几何 2026-09-21 重标定**（`WavyRoadTerrain` 默认值 + `wavy_{0p3,0p5,0p8,1p0}` 场景）：
   4.0 m 长 / 1.2 m 宽 / 0.40 m 波长 / 波峰 36~60 mm / 波谷贴地，速度曲线统一为
   "1 s 站定 → 2 s 提到目标速度 → 匀速走完 4.0 m → 出路面 0.6 s 后减速"，所以各档时长不同
   （0.30 m/s 19.8 s、0.50 m/s 13.1 s、0.80 m/s 9.3 s、1.00 m/s 8.1 s）。出图：
   `.venv/bin/python paper/figures/fig_wavy_speed.py` 出 7 张图（`fig_wavy_roll_speed_{0p3,0p5,1p0}`、
   `fig_wavy_wheel_height_speed_{0p3,0p5,1p0}`、`fig_wavy_profile`）：侧倾和轮心高度按速度**分开出图**，
   同一物理量的三张图共用纵轴范围；横轴是"自入路起的相对时间"，因为三档入路时刻不同。
6. **⚠️ 跳跃实验矩阵缺失（待补）**：历史上设计过一组按论文 4.4 节三个设计量组织的跳跃对照
   （D1 弹道标定 `jump_amp_*`、D2 净空 `jump_tuck_*`、D3 姿态 `jump_attitude_*`、
   D4 速度保持 `jump_run_*`、D5 着地相位 `*_crouch0p25`、D6 越障 `step_{05,10,15}_04`、
   以及 `jump_triple`），配套出图脚本为 `paper/figures/fig_jump_experiment.py`。
   **这些场景与脚本目前都不在仓库里**，`--group jump` 现在只剩 `jump_step20` 一条。
   要复现论文 4.4 节的跳跃图，需要先把 `scenarios.py` 里的这批场景和出图脚本补回来。
   另外 `paper/figures/fig1_render.py`（图 1(a) 的渲染脚本）同样缺失，
   它引用的 `fig1_render.png` 也不在仓库里。
7. **文件体积**：跑全 33 个场景后 `paper/data/` 约 44 MB（500 Hz 采样）。
   长场景（如 30 s 的 `straight_10m`）可以加 `--sample-every 2` 或 `5` 把体积减半以下，
   画曲线完全够用；跳跃落地那一段建议保持默认的每步采样。
   `.cache/` 与 `data/**/*.csv`（含 `data/单腿变高度/` 子目录）已在 `paper/.gitignore` 里忽略，
   所以真正进仓库的只有几 MB 的 `*.json` 汇总指标与 `index.json`——
   它们是论文数字的出处，建议保留。
