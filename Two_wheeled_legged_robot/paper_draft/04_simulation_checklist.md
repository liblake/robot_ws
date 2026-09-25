# 第 4 章需要做哪些仿真（逐条核对）

> 做法：把 `04_results_draft.md` 里每一句带数字的结论，反查它依赖哪个仿真。
> **状态（2026-09-19 更新）：必须补的 3 项与建议补的 3 项均已跑完**，只剩 3 个可选项未做。

---

## 0. 总览

| 章节 | 要支撑的结论 | 需要的仿真 | 状态 |
|---|---|---|---|
| 4.1 | 指标定义、前向速度口径 | 无（定义） | — |
| 4.2 | 站立 2.66°、漂移 <1 mm | `stand_12s` | ✅ |
| 4.2 | 四档速度的稳态误差与上升时间 | `speed_0p8/1p5/2p0/2p5` | ✅ |
| 4.2 | "仿真在 3.0 m/s 仍稳定" | `speed_3p0` | ✅ 已补 |
| 4.2 | 转向 0.500→0.493 rad/s | `turn_0p5` | ✅ |
| 4.2 | 变高度、复合、直线 10 m | `height_cycle` / `composite_0p4_0p2` / `straight_10m` | ✅ |
| 4.2 | "超调来自积分项累积" | 关积分对照（可选） | ⚠️ 可选 |
| 4.2 | 松杆刹车（0.70 s / 219 mm） | `stop_release` | ✅ 已补 |
| 4.3 | 前馈三档 8.38/0.44/0.07° | `ff_ramp65_scale0p0/0p8/1p0` | ✅ |
| 4.3 | 单轮坡 20/40/65 mm × 两档速度 | `ramp_20_03/40_03/65_03/65_045` | ✅ |
| 4.3 | 静态单轮垫高 | `pad_static_10mm` | ✅ |
| 4.3 | 波浪路 0.3~1.0 m/s | `wavy_0p3/0p5/0p8/1p0` | ✅ |
| 4.3 | "单腿变高度"左右对称 | `ramp_65_03_right` | ✅ 已补（0.072° vs 0.068°） |
| 4.4 | D1 弹道—力矩标定（幅度 0.40~1.00） | `jump_amp_0p40/0p55/0p70/0p85/1p00` | ✅ |
| 4.4 | D2 收腿开关对照（净空 213 vs 114 mm） | `jump_tuck_on` / `jump_tuck_off` | ✅ |
| 4.4 | D3 镜像髋对照（2×2） | `jump_attitude_diff/samesign(_notuck)` | ✅ |
| 4.4 | "连续跳跃无累积漂移" | `jump_triple` | ✅ 已补（三跳 0.00°） |
| 4.4 | D4 行驶跳保持率 91.4~85.0%（0.5~2.5 m/s） | `jump_run_0p5/0p8/1p5/2p0/2p5` | ✅ |
| 4.4 | D5 着地相位单变量对照（0.5 / 2.5 m/s） | `jump_run_0p5/2p5_crouch0p25` | ✅（**21.5% → 85.0%**） |
| 4.4 | 台阶 5/10/15 cm | `step_05_04/10_04/15_04` | ✅ |
| 4.4 | "残余 15% 损失来自落地冲击" | 落地质心偏移遥测（可选） | ⚠️ 可选 |
| 4.5 | 质量/质心/摩擦/增益/传感/推力 | `test_robustness.py --quick` | ✅ |
| 4.5 | 延迟预算 2~20 ms | `paper/delay_scan.py` | ✅ |

---

## 1. 已经跑完的 34 项（可直接引用）

### 1.1 场景运行器的 32 个场景

命令：`.venv/bin/python -m paper.run_cases --all`（约 110 s，实时倍率约 3.2×）

| 分组 | 数量 | 场景 | 支撑的结论 |
|---|---|---|---|
| ground | 10 | `stand_12s`、`drive_pm0p5`、`speed_0p8/1p5/2p0/2p5`、`turn_0p5`、`height_cycle`、`composite_0p4_0p2`、`straight_10m` | 表 3 上半部分全部 |
| terrain | 9 | `ramp_20_03/40_03/65_03/65_045`、`pad_static_10mm`、`wavy_0p3/0p5/0p8/1p0` | 表 3 下半 + 图 4(b) |
| ff | 3 | `ff_ramp65_scale0p0/0p8/1p0` | 图 4(a) + 4.3 节的核心结论 |
| jump | 21 | D1 `jump_amp_0p40..1p00`、D2 `jump_tuck_on/off`、D3 `jump_attitude_*`、D4 `jump_run_0p5..2p5`、D5 `jump_run_*_crouch0p25`、D6 `step_*_04`、`jump_triple` | 表 4 全部 + 图 6~图 19 |

产出：`paper/data/<场景>.csv`（500 Hz 逐步遥测）+ `.json`（汇总指标）+ 一张总表 `paper/data/summary.md`。

### 1.2 两个独立脚本

| 脚本 | 内容 | 耗时 | 产出 |
|---|---|---|---|
| `test_robustness.py --quick` | 17 种扰动 × 4 场景（质量、质心、摩擦、增益、传感、推力、延迟 10/20/40 ms） | 约 4 min | 控制台表格（已存 `/tmp/robust2.log`，建议转存） |
| `python -m paper.delay_scan` | 延迟 2/4/6/8/10/12/16/20 ms × 4 场景 | 约 2 min | `paper/data/delay_scan.json` |

---

## 2. 还缺的仿真

### 2.1 必须补（3 项，直接决定正文里的三句话能不能写）

**① 三连跳 —— 支撑"连续跳跃之间无累积"**

现在 `jump_amp_*` 只在 t=2.0 s 触发一次跳跃。正文写的"每跳偏航漂移 0.00°、多次连续跳跃之间无累积"
目前**没有直接数据**（单跳测不出"累积"）。补法：新增场景 `jump_triple`，
触发时刻 2.0 / 3.5 / 5.0 s，时长 7 s，比较三跳各自的偏航漂移与落地俯仰。
工作量：scenarios.py 加一条 + 跑一次，约 30 s。

**② 3.0 m/s 速度工况 —— 支撑"仿真在 3.0 m/s 仍能稳定跟踪"**

现有速度场景最高 2.5 m/s。补法：新增 `speed_3p0`（同模板，目标 3.0 m/s），约 15 s。
（归档里 `test_speed_sweep.py` 有这个工况，但那是旧脚本口径，建议用新场景统一口径重跑。）

**③ 下蹲时长基线 —— 支撑"49% → 85%"**

正文写"把下蹲时长由 0.25 s 缩短到 0.08 s 后，2.5 m/s 行程保持率由 49% 提升到 84%"。
当前 runner 只能改控制器参数，**改不到跳跃轨迹参数**（那些在 `launch_mujoco.MANUAL_JUMP_TRAJECTORY_PARAMS`），
所以 49% 这个基线数只能由专用脚本产出：

```bash
.venv/bin/python test_jump_moving.py --baseline     # 同时跑改动前(0.25/0.20)与当前(0.08/0.15)
```

耗时约 1~2 min。**若不想跑，就把这句改成定性表述**（"缩短着地相位时长可显著提高速度保持率"）并删掉 49%。

### 2.2 建议补（3 项，用于堵审稿人的追问）

**④ 右轮单侧坡 `ramp_65_03_right`**

现在所有地形工况都是"左轮上坡"。审稿人可能问"换另一侧还成立吗"。
补法：复制 `ramp_65_03` 把 `terrain_side` 改成 `"right"`，约 20 s。
顺带还能验证 `_add_single_wheel_trapezoid` 的右侧车道映射没问题。

**⑤ 松杆刹车 `stop_release`**

归档里有"0.5 m/s 松杆刹车 0.83 s / 279 mm"这个数据，但不在当前 runner 里。
若表 3 想加"刹车距离/时间"一行（对审稿人是个不错的加分项），补一个场景：匀速 0.5 m/s → 松杆 → 停，
时长 12 s，约 20 s 机时。

**⑥ 补全鲁棒性表：摩擦 ×2 与初始倾角扰动**

`--quick` 只跑了摩擦 ×0.5；完整版还有摩擦 ×2 与初始倾角。若表 5 想写"摩擦 ×0.5/×2"，
需要跑一次完整版（`test_robustness.py`，约 12 min）或单独补这两档。

### 2.3 可选（不影响当前草稿）

**⑦ 关闭速度环积分做对照** —— 用来证明"超调来自积分项累积"这个解释性表述。
做法：把 `velocity_ki` 设为 0 重跑 `speed_2p5`，看超调是否消失。约 20 s。

**⑧ 落地瞬间质心偏移遥测** —— 用来支撑"残余 15% 速度损失来自落地冲击"。
runner 现在不记录质心与轮轴的纵向偏移；若要写实，需要在 `sample_row` 里加一列或写个专用探针。

**⑨ 与开源四连杆基线对比** —— **当前草稿没有写"优于基线"，所以不需要**。
只有当你想加一句"过障性能优于某公开基线"时才要跑（需在 `wheel_legged_robot_sim-main` 上跑同口径场景，成本较高）。

---

## 3. 建议的执行顺序

```bash
cd ~/robot_ws/Two_wheeled_legged_robot/robot_sim

# 1) 补三个新场景（需先在 paper/scenarios.py 里加，见下）
.venv/bin/python -m paper.run_cases --case speed_3p0 --case jump_static_triple \
    --case ramp_65_03_right --case stop_release

# 2) 下蹲时长基线
.venv/bin/python test_jump_moving.py --baseline

# 3) 重新汇总数字
.venv/bin/python -m paper.summarize --write
```

新增场景的写法（`paper/scenarios.py` 里加 4 条即可，全部复用现有模板）：

```python
# 3.0 m/s 速度工况
_add(Scenario(name="speed_3p0", group="ground", desc="平地 3.0 m/s（仿真相对于实机上限）",
              duration=11.0, velocity=_drive_speed_sweep(3.0),
              windows={"cruise": (2.0, 8.0), "brake": (8.5, 11.0)}))

# 三连跳
_add(Scenario(name="jump_static_triple", group="jump", desc="原地三连跳（累积漂移）",
              duration=7.0, jump_times=(2.0, 3.5, 5.0), jump_amplitude=1.0))

# 右轮单侧坡
_add(Scenario(name="ramp_65_03_right", group="terrain", desc="右轮 65 mm 单轮坡 @0.30 m/s",
              duration=9.0, terrain="ramp",
              terrain_kwargs={"terrain_side": "right", "terrain_height": 0.065},
              velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.0, 0.3), (8.0, 0.3), (9.0, 0.0))),
              windows={"obstacle": (3.0, 5.0)}))

# 松杆刹车
_add(Scenario(name="stop_release", group="ground", desc="0.5 m/s 松杆刹车",
              duration=12.0,
              velocity=Schedule(((0.0, 0.0), (1.0, 0.0), (2.5, 0.5), (6.0, 0.5), (6.01, 0.0))),
              windows={"cruise": (4.0, 5.9), "brake": (6.0, 12.0)}))
```

> 三连跳需要在 `paper/summarize.py` 里补一小段：把 `jump1_*`、`jump2_*` 也打印出来
> （现在只打印 `jump0_*`，因为场景里只有一次跳跃）。

---

## 4. 时间预算

| 项目 | 机时 |
|---|---|
| 必须补的 3 项 | 约 2.5 min（其中基线 1~2 min） |
| 建议补的 3 项 | 约 15 min（主要是完整版鲁棒性） |
| 可选 ⑦⑧ | 约 1 min |
| **合计（只做必须 + 建议 ④⑤）** | **约 3 min** |

---

## 5. 一句话结论

第四章的骨架已经跑齐了：**主要结论（地形前馈对照、镜像髋对照、收腿对照、行驶跳速度保持、鲁棒性与延迟）
全部有直接数据**。真正需要补的只有三句话的支撑——"3.0 m/s 仍稳定"、"连续跳跃无累积"、"49% → 85% 的基线"。
其中前两项各加一条场景就能跑出来（约 45 s），第三项跑一次 `test_jump_moving.py --baseline`（约 1~2 min）。
