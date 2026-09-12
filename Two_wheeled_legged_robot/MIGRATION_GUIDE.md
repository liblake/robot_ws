# 开源轮腿控制算法移植实施指南（串联双轮腿机器人）

> 适用对象：`/home/lixiang/robot_ws/Two_wheeled_legged_robot`（你的串联双轮腿机器人）
> 参考对象：`/home/lixiang/robot_ws/wheel_legged_robot_sim-main`（开源 LQR + VMC 双轮腿仿真工程）
> 本指南范围：**只看这两个目录**，不涉及工作区里其他机器人工程。
> 当前结论：**可行**。算法架构（5D LQR 平衡 + VMC 腿高 + 跳跃相位机）与腿机构无关的部分约占代码量的 2/3，可以直接沿用；需要重做的是"串行双电机腿"的腿层（逆运动学 + 雅可比 + 执行器分配）。

---

## 0. 这份指南怎么用（先读这里）

你目前是"小白"，所以每个阶段都按固定格式写：

1. **你做什么**：复制粘贴命令、看现象。
2. **我（Codex）做什么**：负责改代码，并且每一处改动都给你讲为什么。
3. **验收标准**：达成后才进入下一阶段，不要跳。
4. **卡住了怎么办**：把终端里最后 20~30 行报错、或者你观察到的现象直接发给我。

铁律：

- **只改复制出来的工程，绝不动原版** `wheel_legged_robot_sim-main`（它是你的对照物）。
- 每阶段先备份（复制一份文件夹或 `git commit`）。
- 每次只改一个东西，改完先跑通，再改下一个。
- 遇到不懂的名词，查本文档第 6 节术语表，再问 Codex。

---

## 1. 结论与原理（一页版）

开源机器人：每条腿是**四连杆并联闭链**，腿高由 1 个主动电机通过机械连杆决定（所以高度→电机角是 1 对 1，用一张查表 LUT）。

你的机器人：每条腿是**串联髋+膝两个电机**，腿高由两个角度共同决定（2 对 1，不唯一），这是唯一的"机构级差异"。

控制器的三层结构和腿机构无关：

```text
最外层：速度/位置/航向/高度 命令（人给）
   ↓
中层：LQR —— 管"像独轮车一样前后不倒"（pitch）和左右不翻（roll）
   ↓
最内层：VMC —— 管每条腿"伸多高、蹲多低"
```

开源代码里 LQR 用的几何量（质心高度、轮距、转动惯量、平衡前倾角）是**运行时从 MuJoCo 模型现算的**，所以换成你的模型后会自动重算，不用手推公式。这就是迁移可行、而且工作量可控的根本原因。

跳跃相位机（STAND→CROUCH→EXTEND→FLIGHT→LAND）更只是"状态机 + 轨迹"，与机构基本无关。

---

## 2. 功能目标清单（对齐开源 README）

| # | 功能 | 难度 | 优先级 |
|---|---|---|---|
| 1 | 模型能在 MuJoCo 中加载、站立不穿地 | 低 | 必须 |
| 2 | 自平衡站立（LQR + VMC） | 中 | 必须 |
| 3 | 前进/后退速度控制 | 中 | 必须 |
| 4 | 转向（yaw）与航向保持 | 中 | 必须 |
| 5 | 原地调腿高（蹲/站） | 中 | 必须 |
| 6 | 斜坡/颠簸地形找平（roll leveling） | 中高 | 建议 |
| 7 | 跳跃（相位机） | 高 | 可选（后期） |

> 说明：你现在没有实机底层控制代码，所以"功能复制"的交付范围是**在你的 MuJoCo 模型里跑通以上全部功能**。等以后有电机驱动接口和 IMU 再谈真机。

---

## 3. 工程安排

把开源工程复制到你的机器人文件夹里作为迁移工程，原版保持不动：

```bash
cd /home/lixiang/robot_ws
cp -r wheel_legged_robot_sim-main Two_wheeled_legged_robot/robot_sim
```

以后所有工作都在 `Two_wheeled_legged_robot/robot_sim/` 里进行：

```text
Two_wheeled_legged_robot/
├── src/robot/robot.urdf        # 你的原始模型（不要动）
├── MIGRATION_GUIDE.md          # 本指南
└── robot_sim/                  # 迁移工程 = 开源工程副本（可随便改）
    ├── src/robot/robot.urdf    # 阶段1 会替换成你的 URDF
    ├── src/robot/meshes/
    ├── src/mjcf_builder.py
    ├── src/model_semantics.py
    ├── src/state.py
    ├── src/controllers/...     # 主要改造点
    └── profile_sim.py          # 无头 5 秒站立测试
```

常用命令速查（都在 `robot_sim` 目录里执行）：

```bash
# 安装/同步依赖（首次或换了 Python 环境时）
uv sync

# 有图形界面：打开带控制器的仿真，用手柄/滑块操作
uv run python -m src.launch_mujoco

# 无图形界面（本机当前就无显示）：无头 5 秒 STAND 性能测试
.venv/bin/python profile_sim.py
```

> 复制目录时会连 `.venv` 一起复制，同机器上一般可直接用；如果运行报 Python 环境错误，删掉 `robot_sim/.venv` 后重新 `uv sync` 即可。

---

## 4. 分阶段实施

### 阶段 0：跑通原版，建立"对照组"（约 1 小时）

**目标**：亲眼看到开源机器人站起来、会走会跳；记录它的正常现象，作为以后对照。

你做什么：

```bash
cd /home/lixiang/robot_ws/wheel_legged_robot_sim-main
uv sync
uv run python -m src.launch_mujoco
```

观察：

- 机器人从站立姿势开始，2~3 秒内稳定立住；
- 左侧滑块：`cmd_linear_x` 前进、`cmd_angular_z` 转向、`cmd_height` 腿高、`cmd_jump` 跳跃；
- 把 `cmd_linear_x` 拉到 0.3，观察它前倾→加速→回正；
- 点 `cmd_jump`，观察 蹲下→蹬伸→腾空→落地缓冲。

无图形界面时的对照组命令（保存为 `check_stand.py` 后运行）：

```bash
cd /home/lixiang/robot_ws/wheel_legged_robot_sim-main
.venv/bin/python check_stand.py
```

```python
# check_stand.py —— 无头跑 2 秒 STAND，输出是否站稳
import sys
from pathlib import Path
import mujoco
sys.path.insert(0, ".")
from src.mjcf_builder import prepare_controlled_mujoco_xml
from src.controllers.combined import CombinedController
from src.controllers.default_params import STAND_PARAMS
from src.state import extract_sim_state

xml = prepare_controlled_mujoco_xml(Path("src/robot/robot.urdf"))
model = mujoco.MjModel.from_xml_path(str(xml))
data = mujoco.MjData(model)
stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
mujoco.mj_resetDataKeyframe(model, data, stand_id)
mujoco.mj_forward(model, data)
ctrl = CombinedController(STAND_PARAMS)
z0 = float(data.qpos[2])
worst_pitch = 0.0
for _ in range(1000):  # 2 秒 @ 500Hz
    st = extract_sim_state(model, data)
    data.ctrl[: model.nu] = ctrl(model, data, st)
    mujoco.mj_step(model, data)
    worst_pitch = max(worst_pitch, abs(st.pitch))
st = extract_sim_state(model, data)
print("2s stand: base_z_drop =", round(z0 - st.base_position[2], 4),
      "|pitch|max =", round(worst_pitch, 4), "contacts =", st.contact_count)
```

本机实测对照值（阶段 0 验收参考）：

```text
2s stand: base_z_drop = 0.0019 m, |pitch|max = 0.0976 rad, contacts = 2
```

**验收标准**：原版能站住；你知道怎么开 viewer；`check_stand.py` 能输出类似上面的数值。

---

### 阶段 1：把工程复制好，替换成你的 URDF（约 1~2 小时）

**目标**：让 MuJoCo 能加载**你的**机器人模型并显示出来（此时还没有控制）。

你做什么：

```bash
cd /home/lixiang/robot_ws/Two_wheeled_legged_robot
cp -r /home/lixiang/robot_ws/wheel_legged_robot_sim-main robot_sim

# 替换模型：你的 URDF 和网格覆盖进复制工程
cp /home/lixiang/robot_ws/Two_wheeled_legged_robot/src/robot/robot.urdf \
   robot_sim/src/robot/robot.urdf
rm -rf robot_sim/src/robot/meshes
cp -r /home/lixiang/robot_ws/Two_wheeled_legged_robot/src/robot/meshes \
   robot_sim/src/robot/meshes
```

验证模型能纯加载（这是我自己核对过的命令，应该能通过）：

```bash
cd /home/lixiang/robot_ws/Two_wheeled_legged_robot/robot_sim
.venv/bin/python - <<'EOF'
import mujoco
m = mujoco.MjModel.from_xml_path("src/robot/robot.urdf")
print("bodies:", m.nbody, "joints:", m.njnt, "geoms:", m.ngeom)
print("total mass:", round(float(sum(m.body_mass)), 3))
EOF
```

预期输出（几何来自模型实测）：

```text
bodies: 8  joints: 7  geoms: 578
total mass: 16.991
```

再运行纯模型查看器（需要图形界面）：

```bash
uv run mjpython launch_viewer.py
```

此时你看到的应该是：一个机身 + 左右两条"髋-膝-轮"串行腿。若报错（比如找不到关节、网格路径错），把报错发给我。

**验收标准**：加载命令输出上面 3 个数字；viewer 里能看到你的完整模型。

---

### 阶段 2：模型工程化——把你的机器人变成"可控制"模型（约 1~2 天）

**目标**：让 `mjcf_builder` 生成带 6 个执行器（4 腿电机 + 2 轮电机）、freejoint、站立 keyframe、简单碰撞体的 MJCF。这是第一个真正的代码改造阶段。

背景：开源工程加载 URDF 后会做一系列后处理。你的模型有两个"机构差异"会让原代码直接报错：

1. 原代码会给腿加 **equality 闭链约束**（你的腿没有闭链，必须关掉）；
2. 原代码只找 **2 个腿电机**（你的有 4 个）。

先约定角色命名（左右以你的实机确认为准：002 侧=右腿、005 侧=左腿；以后所有代码都用这套名字）：

| 你的 URDF 名字 | 角色 | 我建议的规范名 |
|---|---|---|
| `base_link` | 机身（free/floating base） | `base_link` |
| `link_002_joint` | 右髋电机 | `right_hip` |
| `link_003_joint` | 右膝电机 | `right_knee` |
| `link_004_joint` | 右轮 | `right_wheel` |
| `link_005_joint` | 左髋电机 | `left_hip` |
| `link_006_joint` | 左膝电机 | `left_knee` |
| `link_007_joint` | 左轮 | `left_wheel` |

> 左右命名已按你提供的实机对应关系锁定（002/003/004=右，005/006/007=左）。
> 电机/轮子的"正方向符号"仍需靠阶段 4 的转动实验验证，不要凭感觉定符号。

需要改的代码（由我做，但你要知道改哪里、为什么）：

```text
src/model_semantics.py    # 重写：关节角色表、轮半径、前进方向符号、connect_sites 清空
src/mjcf_builder.py       # 关掉 equality 约束；执行器 2→6 个；keyframe 改成你的站立位形
                          # 碰撞代理按你的连杆重画（可选：先保留网格碰撞保证正确）
src/state.py              # 关节/轮子名称表跟随新语义
src/controllers/default_params.py  # 参数占位：先给保守初值，数值后面统一调
```

测量脚本（确定你的模型几何，做一次并记录结果）：

```bash
cd /home/lixiang/robot_ws/Two_wheeled_legged_robot/robot_sim
.venv/bin/python - <<'EOF'
import numpy as np, mujoco
m = mujoco.MjModel.from_xml_path("src/robot/robot.urdf")
d = mujoco.MjData(m); mujoco.mj_forward(m, d)
names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i): i for i in range(m.nbody)}
print("total mass =", round(float(sum(m.body_mass)), 3))
for bname in ("link_004", "link_007"):
    b = names[bname]
    origin = d.xpos[b]                       # 轮轴通过轮 body 原点
    axis = d.xmat[b].reshape(3, 3)[:, 0]     # 轮轴方向（本体系 X 轴）
    r = 0.0
    for g in range(m.body_geomadr[b], m.body_geomadr[b] + m.body_geomnum[b]):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
        p = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        # 滚动半径 = 网格点到"轮轴"的最大径向距离，不能用网格质心！
        r = max(r, float(np.linalg.norm(np.cross(axis, p - origin), axis=1).max()))
    print(bname, "rolling radius (from axle) ~", round(r, 4))
EOF
```

> 实测结果（2026-09-09）：total mass = 16.991 kg；两轮滚动半径 = 0.070 m。
> 注意：如果以"网格质心"为圆心量半径会得到 ~0.0766，那会把轮毂/电机偏移也算进去，
> 不是滚动半径，控制器里 `WHEEL_RADIUS` 必须用 0.070。

**验收标准**：

- `prepare_controlled_mujoco_xml("src/robot/robot.urdf")` 不报 equality/执行器错误；
- 物理执行器共 6 个（`act_*`：4 腿 + 2 轮）；`model.nu == 10` 是因为另含 4 个 cmd 滑块伪执行器；
- 存在名为 `stand` 的 keyframe（先摆成一个合理站立姿势，能站在地面不穿模）；
- 无头跑 0.1 秒不出 NaN。

---

### 阶段 3：串行腿腿层——IK、站姿律与 VMC（约 2~4 天，核心工作量）

**目标**：让每条腿能用髋、膝两个电机精确跟踪"目标腿高"。

为什么必须先做这步：开源 VMC 假设"一个腿电机角 ↔ 一个腿高"（查 LUT）。你的腿有两个电机，多了一个自由度，所以要先定**站姿律**。

推荐站姿律（第一阶段最简单、最稳）：

> 让轮心始终保持在髋关节正下方，腿在矢状面内摆动；给定腿高 h（髋到轮心的竖直距离），用解析 2 连杆 IK 解出唯一的髋角、膝角。

需要新增/重写的代码（由我做）：

```text
src/controllers/serial_leg_ik.py    # 新增：串行 2R 解析 IK（几何常量唯一真值源）
src/controllers/vmc.py              # 重写腿层：每条腿 髋+膝 双关节 PD
                                    #   + 腿自重前馈(qfrc_bias) + 可选整车支撑前馈
src/controllers/combined.py         # 跟随新语义：h 范围走 ik，软限幅参数化
src/controllers/default_params.py   # nominal_height=0.37、软限幅 30 N·m
src/launch_mujoco.py                # 关节诊断/目标角读取适配 4 个腿电机
src/mjcf_builder.py                 # cmd_height 滑块范围 0.31~0.50
test_leg_height_tracking.py         # 新增：固定机身悬空高度方波验收
```

实测几何（来自 URDF，2026-09-09；实机如不同改 `serial_leg_ik.py` 顶部）：

```text
大腿 L1 = 0.300 m（髋→膝 平面内），小腿 L2 = 0.34325 m（膝→轮）
名义站高：h_hip = 0.30 m  ⇔  h_base(=机身原点-轮心) = 0.37 m
工作区间：h_base ∈ [0.31, 0.50] m（下限受髋限位约束，上限为保守验证值）
站姿律：轮心保持在髋正下方
```

测试方法（不接平衡，先只动腿）：

```bash
cd ~/robot_ws/Two_wheeled_legged_robot/robot_sim
.venv/bin/python test_leg_height_tracking.py
```

脚本把机身固定悬空（轮子离地），用 weld 改为"固定体+占位 root"，
目标高度走方波 0.37 → 0.32 → 0.42 → 0.37。

**验收标准**：

- 高度跟踪误差 < 5 mm（稳态）；
- 升降不超关节限位、不撞奇异点（腿打直的角度要预留安全余量）；
- 髋/膝力矩不互相"顶牛"、不常驻饱和；
- 左右腿同高=升降；左右腿差高=机身左右找平（roll leveling 的雏形）。

阶段 3 实测结果（2026-09-09，kp=40 / kd=1.5 / 腿自重前馈开）：

```text
finite = True  limit_violation = False  饱和步数 = 0/1600
最大腿力矩 = 9.18 N·m（软限幅 30，执行器 40）
目标 h=0.37：稳态误差 +1.8 mm（max 1.9）
目标 h=0.32：稳态误差 +4.1 mm（max 4.8）
目标 h=0.42：稳态误差 -0.2 mm（max 1.7）
验收：PASS
```

> 调试中踩过的坑：腿高必须用机身 **body 原点（xpos）** 而不是质心（xipos），
> 否则 base 的 ipos z=+0.011 会造成系统性 +11~15 mm 误差。

---

### 阶段 4：站起来——LQR 平衡 + STAND（约 3~7 天，第一个大里程碑）

**目标**：机器人像原版一样自平衡站立 10 秒以上，扰动后能回稳。

需要改的代码：

```text
src/controllers/balance_lqr.py   # WHEEL_RADIUS→0.07；轮子 body 名走语义表；
                                 # roll 差动通道从 2 个腿电机扩展成 4 个（髋为主，膝跟随站姿律）
src/controllers/balance_state.py # 名称跟随语义
src/controllers/combined.py      # 执行器分配：6 个执行器；腿力矩 = LQR roll 差动 + VMC 高度，按相位独占
src/controllers/default_params.py# 初值 + 调参记录（这是唯一改参数的地方）
```

好消息：开源代码的 LQR 增益是**根据你的模型自动重算**的（质量 17kg、轮距、质心高度都在运行时测量），所以不需要手算。

测试/调参顺序（沿用开源作者的经验，很重要）：

```text
1) VMC 腿高 PD      （腿先稳，不抖、不漂）
2) LQR Q/R 权重      （身体立住：pitch/roll 不发散）
3) 速度外环 pitch_lean / velocity_ki
4) yaw 阻尼 → heading_hold
```

对照脚本沿用阶段 0 的 `check_stand.py`，把模型路径和参数换成 `robot_sim` 的即可。交互式观察：

```bash
cd /home/lixiang/robot_ws/Two_wheeled_legged_robot/robot_sim
uv run python -m src.launch_mujoco
```

常见"症状 → 原因"对照（初学最容易踩）：

| 现象 | 最可能原因 | 怎么调 |
|---|---|---|
| 一启动就猛往前/后倒 | 轮子前进方向符号反了 | 翻转 `WHEEL_FORWARD_SIGNS` 再试 |
| 站住但腿抖 | VMC `kp_motor` 太大 / `kd_motor` 太小 | 先降 kp，再加 kd |
| 身体慢慢前倾加速 | LQR `q_pitch`/`q_pitch_rate` 弱 或 轮扭矩限幅太小 | 加 Q[0]/Q[1]，或加大轮 ctrlrange |
| 左右晃 | roll 通道符号/映射错，或左右腿高差方向反 | 先关 roll，只调 pitch；再用左右腿高差做 roll leveling |
| 数值爆 NaN | 腿进奇异点 / 高度超出 LUT 范围 | 限位加安全边距，检查 `h_min/h_max` |

**验收标准**：

- 无头 10 秒 STAND 不摔倒（`base_z_drop` < 2 cm）；
- 启动瞬态后 `|pitch|` < 0.15 rad；
- 给一个前倾初值（比如 0.15 rad）能自己回正；
- 双轮始终接地（contacts = 2）。

阶段 4 试验台验收（2026-09-09，`test_stand_wheel_balance.py`，项目虚拟环境运行）：

```text
12s VMC+wheel-LQR: |pitch|max = 0.233 |roll|max = 0.025
end_pitch = 0.227（平衡角≈0.229） end_z = 0.472 z_min = 0.465 contacts = 2
```

说明：该试验台使用"VMC 腿高控制（接触逆动力学前馈，kp=120/kd=15）+ 2 状态
轮子 LQR（增益 K=[−38.2, −6.3]，按实机轮峰值 ±9 N·m 限幅）"，稳态 pitch 恒定、
roll≈0、双轮全程接地。

> 注（2026-09-12）：正式控制器里的支撑前馈已从 `mj_inverse` 换成解析式
> `τ_j = (∂h/∂q_j)·m·g/N + Σ m_i·g·(∂z_i/∂q_j)`（可移植到实机），
> 见 [RESULTS_ARCHIVE.md](RESULTS_ARCHIVE.md) 第 9 节。上面这段是阶段 4 当时的记录。

已正式接进 `CombinedController`/`launch_mujoco`（STAND_PARAMS 启用
`wheel_balance_gain_2d` 覆盖通道，stand keyframe 改为倾斜平衡位形 pitch≈0.229、
机身 0.459 m）。CombinedController 12 秒实测：

```text
Combined 12s: |pitch|max = 0.05 |roll|max = 0.0
end_pitch = 0.028（平衡角≈1.6°）end_z = 0.441 z_min = 0.437 y_max = 0.008 contacts = 4
```

修正（2026-09-09 视窗观察反馈）：只锁平衡角会让机器人一直向前加速；
已把速度 PI + 位置保持外环接回覆盖通道（LQR 跟踪"平衡角 + pitch_lean"目标）。
复测：`y_max = 0.004 m`（原地站立，不再前冲），轮速收敛不再增大。

修正 2（2026-09-09 视窗反馈"3~4 秒后机身抖动"）：抖动源于碰撞代理用球体
（球可侧滚造成慢速 roll/pitch 晃动）。轮子代理改为沿轮轴的圆柱（更接近真实
轮毂直驱轮）后：3 秒后 pitch 峰峰 0.0008 rad、roll≈0、无离地，晃动消除。

修正 3（2026-09-09 视窗反馈"机身一直前倾约 10°"）：这不是惯量不匹配，
而是 URDF 整机质心在轮轴中点后方约 58 mm，轮心在髋正下方时平衡角天然
≈13°。已在 `serial_leg_ik.py` 引入 `wheel_y_offset=-0.075`（轮心相对髋后移
75mm），把平衡角降到 ≈1.6°（基本直立），并保留低姿态关节限位余量。

阶段 4 验收：**PASS**（偏差按相对平衡角计：稳态 |pitch−eq|<0.03 rad，
双轮全程接地、机身高度不掉、原地站立 y 漂移 <2 cm）。

---

### 阶段 5：运动功能对齐（前进、转向、调高、地形）（约 1~2 周）

按顺序逐步放开（每个都先在无头 rollout 验证再上 viewer）：

1. **前进/后退**：`cmd_linear_x` → `pitch_lean` 速度外环（原版能到 ±0.5 m/s，先对齐这个量级）；
2. **转向与航向保持**：`cmd_angular_z`、yaw 阻尼、heading_hold；
3. **原地调高**：`cmd_height` 沿站姿律升降，注意别顶到 IK 极限；
4. **左右轮高差找平**：VMC roll leveling（应对斜坡、单轮垫高）；
5. **场景化验证**：用原工程的 `stand_then_drive` 思路做 10 m 直线。

对应开源文件基本不用动结构，主要是参数与场景：

```text
src/controllers/default_params.py  # 新增 STAND_THEN_DRIVE 等参数预设
src/launch_mujoco.py               # 场景/滑块/手柄（原版已经写好了，基本白拿）
src/optimize.py                    # 可选：用 Optuna 无头自动调参
```

**验收标准**：原版 README 里的功能（站立、行进、转向、调高）在你的模型上一一复现；无头 rollout 指标与阶段 0 对照组同量级。

阶段 5 实测结果（2026-09-09，默认参数 kp=200/kd=30、yaw_damping=8、
轮心偏移 -0.075、IK 高度范围 [0.31, 0.50]）：

```text
前进/后退：0.5 m/s 梯形速度曲线，往返稳定，终点 +0.64 m
原地转向：0.3 rad/s 档实际≈0.23，正反转净角 +0.23 rad
原地调高：0.33↔0.45 m，误差≤1mm，y 漂移≈0
复合运动：0.4m/s + 0.2rad/s 弧线，pitch≈0.028，净转角回 0
10m 直线：26.0s 到达；横向漂移 0、航向偏差 0、|pitch|max=0.066、
          轮速峰值 7.4 rad/s（电机峰值 36.7 rad/s 余量充足）
```

配套演示脚本：`test_stand_wheel_balance.py`、`test_drive.py`、
`test_turn.py`、`test_height.py`、`test_composite.py`、`test_straight_10m.py`
（均可用 `--viewer` 可视化；正式入口 `launch_mujoco` 的 cmd_* 滑块已直接
驱动 target_velocity / target_yaw_rate / nominal_height）。

手柄实机反馈修正（2026-09-09）：
- 前进/后退方向：用户手柄左摇杆 Y 轴与默认假设相反，映射符号已反号；
- 松杆减速摆动：指令阶跃会造成前后猛摆，CombinedController 增加
  `max_linear_accel=0.4 m/s²` / `max_yaw_accel=0.15 rad/s²` 斜坡限速，
  减速段 pitch 峰峰从 0.306 降到 0.145 rad 并可收敛。

阶段 5 验收：**PASS**（站立/前进后退/转向/调高/复合/10m 直线全部稳定；
roll 找平与斜坡地形留作后续扩展）。

> 完整参数表、已修 bug 清单、脚本速查与已知限制见
> [RESULTS_ARCHIVE.md](RESULTS_ARCHIVE.md)（成果归档，2026-09-09）。

---

### 阶段 6（可选）：跳跃与相位机（约 1~2 周）

跳跃依赖相位机（`phase.py`）+ 轨迹（`jump_trajectory.py`），这两个文件可以整份保留。

要重做的是"蹬伸轨迹怎么分给髋、膝两个电机"：

- CROUCH：沿站姿律从名义高蹲到低点；
- EXTEND：双关节按任务空间加速度前馈猛蹬（力矩上限可临时放宽）；
- FLIGHT：对称阻尼防空中翻转；
- LAND：软 P 硬 D 吸收冲击。

注意：跳跃对腿电机峰值力矩要求很高，你的模型关节 effort=10 N·m 看起来是 CAD 导出默认值，不代表真实电机能力。仿真里先放宽限位验证"算法能跳"，真机再校核电机。

---

## 5. 调参总顺序（抄录自开源 `default_params.py`，贴在你的工作台旁边）

```text
1) VMC PD:      kp_motor, kd_motor               （腿不抖、不漂）
2) LAND PD:     kp_land, kd_land                 （落地不弹）
3) FLIGHT 阻尼: flight_pitch_kd
4) LQR balance: q_diag[0:4], r_diag              （pitch/roll 不发散）
5) 前进速度:    pitch_lean_gain, velocity_ki
6) 转向:        yaw_damping → yaw_ki；航向 heading_hold_kp
7) 跳跃:        air_height_max, crouch_depth, extend_stroke
```

规则：**所有参数只存在 `default_params.py` 一处**；每调一次记一句"改了谁、现象是什么"，否则两周后你会忘了为什么。

---

## 6. 术语小字典（小白版）

| 术语 | 一句话解释 |
|---|---|
| URDF | 机器人机械模型文件（连杆、关节、质量），Open-source 和你各有一个 |
| MJCF | MuJoCo 原生模型格式；`mjcf_builder` 负责把 URDF 转成它并加控制件 |
| freejoint / floating | 机身相对世界自由浮动（6 自由度），仿真机器人不被焊在地上 |
| 执行器 actuator | 仿真里的"电机"，给关节加力矩；开源 4 个、你 6 个 |
| LQR | 一种"状态反馈最优控制"：测出偏了多远，按算好的增益给出纠正力矩 |
| pitch | 绕轮轴方向的前后倾倒角（你往前栽就是 pitch 变大） |
| roll | 左右侧倾角（你往左歪就是 roll 变大） |
| 虚拟控制量 | 先算"前进总力矩/左右差动力矩"这种抽象量，最后再分给具体电机 |
| VMC | 虚拟模型控制：把"让腿到达目标高度"的需求翻译成每个关节的力矩 |
| IK（逆运动学） | 已知"脚/轮在哪"，反推每个关节该转多少度 |
| 雅可比 J | 关节角度变化 → 轮子位置/速度变化的换算表；重力前馈要用它 |
| LUT | 查表：把腿高预先算成一串电机角，运行时插值，省实时计算 |
| 站姿律 | 你的腿多一个自由度，人为规定"髋+膝怎么配合"的那条规则 |
| 相位机 | 跳跃各阶段（站/蹲/蹬/飞/落）的状态机，避免控制器互相打架 |
| 前馈 FF | 预先把"克服重力/加速需要的力量"直接给出去，反馈只补误差 |
| keyframe | 模型的"初始姿势存档"，STAND 用它把机器人摆正再启动控制 |
| ctrlrange | 执行器力矩限幅；开源腿电机 ±12.5 N·m，控制内层再限 ±3.5 N·m |

---

## 7. 提前知道能少走弯路的坑

1. **CAD 质量不一定等于实机质量**：你的 URDF 总质量 ≈ 17 kg、机身 10 kg，多半是 SolidWorks 估算。仿真阶段可以先用，真机前必须用称重值替换，否则 LQR/重力前馈全部按错质量算。
2. **轮半径与轮体**：模型里"轮"（`link_004/007`，各 2.5 kg）其实是轮+轮毂/电机整体，网格外径约 0.07 m。`WHEEL_RADIUS` 必须用真实滚动半径。
3. **符号全靠实验定**：关节/轮子的正方向很容易差个负号，症状是"一开平衡就往一个方向栽"。阶段 4 第一件事就是验证 `WHEEL_FORWARD_SIGNS`。
4. **冗余自由度会打架**：髋、膝都去"抢"腿高，会互相顶牛。阶段 3 的站姿律就是为消除这个矛盾。
5. **奇异点**：腿完全打直时 IK 接近奇异，VMC 前馈会爆炸。限位 + LUT 范围都留 5~10% 安全边距。
6. **改代码前先备份**，出问题就用备份回退，别在坏代码上反复试。
7. **两个工程别混淆**：工作区里其他机器人文件夹与本指南无关，不要从那里复制参数/几何到 `robot_sim`。

---

## 8. 总检查清单（每完成一项打勾）

- [ ] 阶段 0：原版能站、能走、能跳；`check_stand.py` 输出对照值
- [ ] 阶段 1：`robot_sim` 已创建；你的 URDF 纯加载通过；viewer 可见完整模型
- [ ] 阶段 2：模型带 6 个执行器、无 equality、有 stand keyframe、无 NaN
- [ ] 阶段 3：双腿能跟踪高度方波，误差 < 5 mm，无抖无越限
- [x] 阶段 4：12 秒 STAND 不倒；稳态 pitch=平衡角、roll≈0；双轮接地
- [x] 阶段 5：前进/转向/调高/复合/10m 直线全部复现；无头指标达标
- [ ] 阶段 6（可选）：跳跃相位机完整走一遍

---

## 9. 下一步（你现在该做的第一件事）

按阶段 0 的命令跑通原版，或者直接执行阶段 1 的复制+替换命令，然后把：

1. `robot_sim` 复制是否成功；
2. 纯加载命令的输出；
3. 任何报错；

发给我。我们从"你的模型被 MuJoCo 加载"这个可见的小胜利开始，逐步走到"站起来"。
