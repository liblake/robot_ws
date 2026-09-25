# 代码检查报告（robot_sim）

> 检查日期：2026-09-16
> 范围：`Two_wheeled_legged_robot/robot_sim`（17 kg 串联双轮腿，MuJoCo 仿真 + LQR/VMC 控制器 + 18 个验收脚本）
> 目的：为论文（方案 A：纯仿真）确认数据来源可靠，并把"取场景"这件事做成一条命令。

---

## 0. 结论摘要

1. 控制器与仿真代码质量整体良好：**模块职责清晰、参数集中在 `default_params.py`、18 个验收脚本都有明确判据**，
   数字与归档一致（本次抽检：站立 |pitch| 0.046 rad、原地跳弹道 133.0 mm、蹬伸力矩 58.1 N·m，全部对上）。
2. 发现 **3 个会直接影响论文数据的问题**，已修（见第 2 节）：
   - 波浪路无法单独加载（这就是 `test_wavy_road.py` 一直在跑平地的根因）；
   - 空中姿态"修正前/后"写在代码里、无法复现对照；
   - 遥测里 `roll` 列实际上是俯仰角（与归档 §14.1 记录的"roll 与 pitch 互为相反数"完全吻合）。
3. 发现 **1 个数据安全风险**：交互模式的遥测日志只保留最近 4 次，论文用到的手柄实跑记录会被自动删除。
4. 最大的工程性问题是**重复劳动**：18 个测试脚本各自实现"建模型 → 循环 → 算指标"，共 3345 行，
   判据阈值散落在各脚本注释里。论文需要几十个工况时，这样取数又慢又容易口径不一致。
   → 已新增 `paper/` 场景运行器（第 3 节），一条命令跑一组场景并落 CSV+JSON。

---

## 1. 结构总览

| 模块 | 行数 | 职责 | 评价 |
|---|---|---|---|
| `src/launch_mujoco.py` | 1110 | 入口：参数解析、模型构建、交互循环、手柄、遥测、跳跃门槛 | 职责过多（见 4.1） |
| `src/mjcf_builder.py` | 872 | URDF→MJCF、地形、碰撞代理、执行器、假滑块关节 | 核心资产，地形已支持组合 |
| `src/controllers/combined.py` | 1036 | 组合控制器：平衡、速度、航向、相位调度 | 核心，注释详细 |
| `src/controllers/vmc.py` | 766 | 腿层：IK 目标、关节 PD、解析前馈、收腿、空中姿态 | 核心，注释详细 |
| `src/controllers/balance_lqr.py` | 414 | 平衡增益求解 + 实测标定（B 的 3.6 倍） | 标定过程有记录 |
| 其他 src | ~1500 | 状态提取、几何、日志、线性化、优化 | 正常 |
| `test_*.py` × 18 | 3345 | 各功能验收 | **重复度高**（见 4.2） |

入口有三类：

1. 交互：`python -m src.launch_mujoco --controller combined --scenario stand --flat-ground`（手柄、遥测、可视化）
2. 验收：`python test_xxx.py`（无头跑 + PASS/FAIL）
3. 论文取数：`python -m paper.run_cases --case <名字>`（本次新增）

---

## 2. 已修复的问题

### 2.1 波浪路无法单独加载（影响：论文第一章"波浪路地形跟随"没数据）

**现象**：`test_wavy_road.py` 跑的是平地；归档 §9.3 自己记了这条，当年用"传 `terrain=` 再手动删掉梯形坡 geom"绕过。

**根因**：`mjcf_builder._ensure_test_terrain` 只有三个分支，且把波浪路挂在梯形坡分支的末尾：

```python
if terrain != "single_wheel_trapezoid":
    raise ValueError(...)
_add_single_wheel_trapezoid(...)
_add_wavy_road(...)          # ← 只能跟梯形坡一起出现
```

`WavyRoadTerrain` 和 `_add_wavy_road` 本身是完整的，只是**没有入口**。

**修法**：改成地形注册表（`_TERRAIN_RAMP` / `_TERRAIN_WAVY`），新增三个名字：

| 名字 | 含义 |
|---|---|
| `ramp` | 只有单轮梯形坡 |
| `wavy` | 只有波浪路 |
| `ramp_wavy` | 两者都有（与旧的 `single_wheel_trapezoid` 等价） |
| `single_wheel_trapezoid` | **行为完全不变**（保持向后兼容） |

同时给 `prepare_controlled_mujoco_xml` / `build_controlled_model` 加了 `wavy_road` 参数，
可以自定义波浪路参数（幅值、波长、种子）。

**验证**：六种地形逐一加载，geom 列表正确；`wavy` 场景 0.3/0.5/0.8/1.0 m/s 全部通过。

### 2.2 空中姿态"修正前/后"无法复现（影响：论文核心机理那张对照图）

**现象**：镜像髋轴的修正是直接改进 `vmc.py` 的 FLIGHT 分支（把两侧同号改成反号），
所以"修正前"的行为在代码里已经不存在了——归档里的 12.4~14.9° vs 4.7° 无法用一条命令复现。

**修法**：新增参数 `vmc.flight_attitude_diff_enable`（默认 `True` = 现在的正确行为）。
置 `False` 即复现"两侧同号、世界系力矩互相抵消"的旧写法。已加入参数 round-trip
（`params_to_dict` / `params_from_dict`），保证存读一致。

**意外收获**：这条开关让机理的证据链变得非常硬。同一工况下：

| 收腿 | 差分驱动（现在） | 同号驱动（修正前） |
|---|---|---|
| 开 | 飞行俯仰 11.4° | 36.0° |
| 关 | 飞行俯仰 6.4° | **180°（翻覆）** |

即"同号驱动 + 不收腿"会直接摔，这正是"阻尼器一直没生效"的最强证据。

### 2.3 遥测 `roll` 列其实是俯仰角（影响：一旦用遥测 LOG 画 roll 曲线就是错的）

**现象**：归档 §14.1 记录"遥测 roll 列与 pitch 列全程互为相反数"，当时归因为"`state.py` 欧拉角提取有轴间串扰"。
**实际结论是反的**：`state.py` 是对的，问题在 `logger.py`。

- 本机前进轴是 **+Y**、轮轴是 **X**，所以"俯仰"= 绕 X 轴旋转，"侧倾"= 绕 Y 轴旋转。
- `state.pitch` = 绕 X（带负号）✅；`state.roll` = 绕 Y ✅。
- 但 `logger._rpy_from_quaternion` 用的是航空 ZYX 约定，返回的 `roll` 是**绕 X 轴**的旋转，
  对本机而言就等于俯仰角 → 遥测里 roll 与 pitch 是同一个物理量（差一个符号），所以"互为相反数"。

**验证**（构造 20° 俯仰 + 5° 侧倾的四元数）：

| | 修复前 | 修复后 |
|---|---|---|
| `roll` 列 | 0.349（= 20°，错） | 0.087（= 5°，对） |
| `pitch` 列 | 0.349 | 0.349 |

**修法**：`logger.log_step` 改用 `state.roll`，并在 `_rpy_from_quaternion` 上加了显式警告，
避免以后有人再拿它当侧倾角用。`test_jump_moving.py --log` 只读 pitch/yaw/速度列，不受影响。

> **对论文的意义**：`paper/run_cases.py` 导出的 `roll_deg` 用的是 `state.roll`，是对的；
> 而 `logs/manual/*/telemetry.csv` 里**修复前**的历史记录 roll 列不能用（修复后的新记录可以用）。

---

## 3. 新增：`paper/` 场景运行器（解决"取场景不方便"）

### 3.1 它解决什么

以前跑一个工况：翻 `test_*.py` → 找到对应脚本 → 改里面的常量 → 跑 → 从 print 里抄数字。
现在：

```bash
.venv/bin/python -m paper.run_cases --group ff          # 跑一组对照，3 秒
.venv/bin/python -m paper.run_cases --all               # 31 个场景，约 3 分钟
.venv/bin/python -m paper.run_cases --case ramp_65_03 --set vmc.stand_rate_ff_scale=0 --tag ff_off
```

输出 `paper/data/<场景>.csv`（逐步遥测）+ `<场景>.json`（汇总指标）+ `index.json`（总索引）。

### 3.2 设计要点

1. **场景是声明式的**：`paper/scenarios.py` 里一条 `Scenario(...)` = 地形 + 命令时序 + 时长 + 参数覆盖 + 分析窗口。
   加场景不用碰控制器。
2. **不重复实现控制逻辑**：模型构建复用 `launch_mujoco.build_controlled_model`，
   控制器复用 `CombinedController` / `JumpPhaseMachine` / `JumpTrajectory`。
3. **参数用点分路径临时覆盖**（`--set vmc.flight_tuck_enable=false`），做对照实验不用改代码。
4. **指标按窗口给**（`windows={"obstacle": (3.0, 5.0)}`），因为论文里的数字大多是"某个区段"的峰值，
   全段统计和分段统计差别很大（这也是归档里数字口径容易混的原因）。
5. **数字口径与验收脚本对齐**：采样放在 `mj_step` 之后（与 `test_jump.py` 一致）。
   已验证：弹道 133.2 mm / 蹬伸力矩 58.2 N·m，对得上脚本的 133.0 / 58.1。

### 3.3 现在的场景（31 个）

| 分组 | 数量 | 内容 |
|---|---|---|
| ground | 10 | 站立、往返行驶、0.8/1.5/2.0/2.5 m/s、转向、调高、复合、10 m 直线 |
| terrain | 9 | 20/40/65 mm 单轮坡（两档速度）、静态垫高、波浪路 0.3~1.0 m/s |
| ff | 3 | 65 mm 坡 × 角速度前馈 0 / 0.8 / 1.0（论文三条对照曲线） |
| jump | 9 | 原地跳 0.7/1.0、收腿与镜像髋的 2×2 对照、行驶跳 0.8/2.5、5/10/15 cm 台阶 |

---

## 4. 其他发现（未改，供你决定）

### 4.1 `launch_mujoco.py` 1110 行，职责过多（中）

一个文件里同时有：CLI 参数、模型构建、手柄映射、遥测记录、跳跃触发门槛、交互循环、性能剖析。
论文写作期间它不会挡路，但如果之后要迁实机或做更多自动化，建议拆成
`cli.py` / `interactive_loop.py` / `gamepad_map.py` / `telemetry.py`。**现在不用动。**

### 4.2 18 个验收脚本重复实现仿真循环（中，已被 `paper/` 部分化解）

`test_*.py` 共 3345 行，每个都自己写"建模型 → 重置 → 循环 → 算指标 → 打印 PASS/FAIL"，
判据阈值（如 `FALL_PITCH`、`MIN_BALLISTIC`）散落在各脚本注释里。
风险：同一个量在不同脚本里口径可能不一致（例如弹道"质心"还是"机身高"）。
`src/rollout.py` 里已有 rollout 骨架，但只有 `optimize.py` 在用。

**建议**：论文期间保持现状（脚本是已验证的，不要动），取数统一走 `paper/run_cases.py`。
等论文投出去再考虑把脚本收敛到统一 harness。

### 4.3 控制器直接读 MuJoCo 内部量 42 处（中，只影响"迁实机"的说法）

`data.xmat / xpos / xipos / qpos / qvel` 被控制器直接读取（机身姿态、连杆位姿、关节状态），
以及 `data.ncon` 用于支撑判定与起跳确认。对纯仿真论文**没有影响**，
但撰写"可迁移到实机"的段落时要注意措辞：其中关节状态可由编码器替代，
机身位姿由 IMU 替代，**接触数是实机上没有对应量的**（需要另行估计）。

### 4.4 交互遥测只保留最近 4 次（数据安全）

`launch_mujoco.main` 每次启动都调用 `cleanup_old_logs(manual_log_base, max_keep=4)`，
会删除第 5 次之前的 `run_*` 目录。论文如果要引用手柄实跑记录，
**跑完立刻把对应 `run_*` 目录复制到别处**（例如 `paper/data/manual/`）。

### 4.5 小问题（低）

- `test_*.py` 与 `src/` 混在仓库根目录，18 个脚本没有分组（可以后移到 `tests/`）。
- `MUJOCO_LOG.TXT`、`__pycache__`、多处 `sys.path.insert` 属于历史遗留。
- 新增参数必须同步加进 `params_to_dict` / `params_from_dict`，否则存读不一致（本次已同步）。

---

## 5. 这次改动清单与回归验证

| 文件 | 改动 | 是否改变默认行为 |
|---|---|---|
| `src/mjcf_builder.py` | 地形注册表：新增 `ramp` / `wavy` / `ramp_wavy`，新增 `wavy_road` 参数 | 否（旧名字行为不变） |
| `src/launch_mujoco.py` | `build_controlled_model` 透传 `wavy_road` | 否 |
| `src/controllers/vmc.py` | 新增 `flight_attitude_diff_enable`（默认 True） | 否 |
| `src/controllers/default_params.py` | 新参数进 round-trip | 否 |
| `src/logger.py` | 遥测 roll 改用 `state.roll`，函数加警告注释 | **是**（修 bug，roll 列从此是真实侧倾角） |
| `paper/`（新增） | 场景注册表 + 运行器 + README | 新增，不影响既有链路 |

回归抽检（改完后重跑原脚本）：

| 脚本 | 结果 |
|---|---|
| `test_stand_wheel_balance.py` | \|pitch\|max 0.046、roll 0、接触 4 —— 与改动前一致 |
| `test_drive.py` | 正常往返、最终位移 +0.62 m —— 正常 |
| `test_slope_v2.py` | 过坡 roll p2p 0.1°、单轮离地 1/159 —— 与归档一致 |
| `test_jump.py` | 弹道 133.0 mm、蹬伸力矩 58.1 N·m、落地俯仰 4.1° —— PASS |
| `paper/run_cases.py`（新） | 站立 2.66°、坡 65 mm roll p2p 0.09°、原地跳弹道 133.2 mm —— 与脚本口径一致 |

---

## 6. 建议的下一步（按优先级）

1. **把论文要用的场景跑一遍，保存到 `paper/data/`**，并把 `paper/data/` 纳入版本管理或单独备份
   （CSV 很小，一个场景几十 KB）。
2. **需要"修正前"数据的场景**（镜像髋、收腿、角速度前馈）已经能一条命令复现，建议一次跑完三组对照并留档。
3. **不要依赖 `logs/manual/`**：它只保留 4 次，且 2026-09-16 之前的 roll 列不可用。
4. 论文里所有数字都从 `paper/data/*.json` 取，正文里标注场景名（如 `ramp_65_03`），
   这样任何人都能一条命令复现。
5. 等论文投出去之后，再考虑把 18 个验收脚本收敛到统一 harness（现在动它有回归风险）。
