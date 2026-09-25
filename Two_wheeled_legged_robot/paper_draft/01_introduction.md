# 第 I 章 Introduction 草稿

> 本文件放 Introduction 的**开头段（¶1）**与**结尾段（¶4 贡献）**的中英对照草稿。
> 中间两段（¶2 相关工作、¶3 缺口）等文献读完再写，见 `PAPER_GUIDE.md` 2.2 节。
> 中文稿先写、英文稿后续润色；英文稿里的 `[1], [2]` 是引用占位，等文献定了再补。

---

## 一、开头段（¶1 背景）怎么写

开头段只干一件事：**在 4 句之内把读者从"这个领域"带到"这个设计问题"**。
它不写你的机器人、不写方法、也不写文献综述——那三样分别是 ¶2/¶3 和后面章节的事。

四句的分工：

| 句 | 任务 | 注意 |
|---|---|---|
| 1 | 轮腿机器人是什么、为什么有价值 | 领域级陈述，挂 1~2 个平台类引用 |
| 2 | 一台实用的轮腿机器人要在同一套硬件上做哪些事 | 用"多种模态"把范围铺开 |
| 3 | 这些模态对同一组关节的要求**互相冲突** | **这一段最关键的一句**，要给出具体冲突，不要写"很有挑战性" |
| 4 | 由此引出一个设计问题 | 提出问题即可，**不要把"别人没做"写在这里**（那是 ¶3） |

第 3 句是这段的灵魂。写"多模态控制具有挑战性"是空话；
写"平地上腿只要保持固定高度，单侧越障要求两条腿取不同长度，跳跃要求腿在 0.2 s 内爆发式蹬伸"
——读者立刻明白难在哪，也自然接上你后面要讲的三种机制。

### ¶1 中文草稿

> 轮腿机器人把轮式平台的高能效与足式系统的地形适应性结合在一起，在非结构化环境中具有明显的运动优势[1], [2]。一台真正实用的轮腿机器人需要在同一套硬件上完成多种任务：在平地上站立与行驶、原地转向、跟随起伏地形，以及越过轮子无法直接爬上的障碍。这些任务对同一组腿关节提出的要求是**相互冲突**的：平地上腿关节的主要职责是把机身保持在恒定高度；单侧越障时，两条腿必须取不同的腿长，机身才能保持水平；而跳跃时，腿必须在极短时间内快速蹬伸，把整机抛离地面。因此，如何用**一套不针对每种模态分别整定的控制器**覆盖上述全部运动，是一个具有实际价值的设计问题。

### ¶1 英文草稿（后面再润色）

> Wheel-legged robots combine the energy efficiency of wheeled platforms with the terrain
> adaptability of legged systems, which makes them attractive for locomotion in unstructured
> environments[1], [2]. A practically useful platform of this kind is expected to perform several
> tasks with the same hardware: standing and driving on flat ground, turning in place, following
> uneven terrain, and crossing obstacles that the wheels cannot climb directly. These tasks impose
> conflicting requirements on the same set of leg joints. On flat ground, the leg joints mainly have
> to hold the torso at a constant height, whereas crossing an obstacle on one side requires the two
> legs to adopt different lengths, and jumping requires a rapid extension that throws the whole
> body into the air. How to cover all of these behaviours with a single controller, without
> retuning it for each mode, is therefore a design problem of practical interest.

### 自检

- 有没有在第 1 句就讲自己的机器人？（应该没有）
- 第 3 句有没有给出**具体的**冲突？（不是"很有挑战"这种形容词）
- 有没有出现"然而，目前研究不足"？（有就删掉，那是 ¶3 的活）
- 全段 4 句、约 110 词。

---

## 二、结尾段（¶4 贡献）怎么写

结尾段的任务：**把研究重点变成"可以被检验的贡献"**，每条贡献后面跟一个数字。

三条规矩：

1. **每条贡献 = 做法 + 效果（数字）**。只写"提出了一个统一框架"是研究重点，不是贡献。
2. **贡献的顺序 = 后面章节的顺序**，方便审稿人对照。
3. **这一段不放文献**（文献在 ¶2/¶3），也不放公式。

### ¶4 中文草稿

> 本文的主要贡献如下。
>
> （1）**统一的运动控制框架**：用单个相位状态机、单一腿控制器与同一组控制参数，覆盖平地行驶、原地转向、单腿变高度越障、波浪路地形跟随与动态跳跃五种模态；模态之间的切换只改变参考量与输出限幅，不重新整定控制器参数。
>
> （2）**解析式重力支撑前馈**：给出不依赖仿真器内部量的支撑与推力前馈表达式，并指出腿连杆自重项在重型串联腿构型下不可省略（本机每腿占整车质量的 21%）。前馈修正后，65 mm 单轮坡上的机身侧倾峰峰值由 8.38° 降至 0.07°。
>
> （3）**镜像关节轴的共模抵消与差分姿态驱动**：指出左右髋关节轴镜像时，同号关节力矩在世界系中互相抵消、无法调节机身姿态，并给出以关节力矩之差驱动的正确形式。修正后，空中俯仰峰值由 36.0° 降至 11.4°，每次跳跃的偏航漂移降至 0.0°。
>
> （4）**跳跃能力的三分解**：把越障能力拆成三个可独立设计的量——质心弹道由蹬伸冲量决定（133 mm，此时膝关节电机峰值力矩已用到 97%）；越障净空由腾空收腿决定（213 mm，不收腿时为 114 mm，且**不增加任何力矩需求**）；速度保持由着地相位时长决定（2.5 m/s 行驶中跳跃的 2 s 行程保持率为 85%，落地最低速仍有 +1.46 m/s，全程不倒滑）。

### ¶4 英文草稿

> The contributions of this paper are as follows.
>
> (1) A unified locomotion control framework in which a single phase-based state machine, a single
> leg controller, and a single parameter set cover five locomotion modes: flat-ground driving,
> in-place turning, single-side obstacle crossing with asymmetric leg lengths, terrain following on
> undulating ground, and dynamic jumping. Switching between modes only changes the references and
> the output limits; the controller parameters are never retuned.
>
> (2) An analytic gravity support feedforward that does not rely on simulator-internal quantities,
> together with the observation that the leg-link self-weight term cannot be neglected for a
> heavy serial-leg design (21% of the total mass in this robot). With the corrected feedforward,
> the peak-to-peak roll angle over a 65 mm single-wheel step decreases from 8.38° to 0.07°.
>
> (3) The identification and correction of a common-mode cancellation caused by mirrored hip axes:
> when the two hip joints are mirror images, equal joint torques cancel in the world frame and
> cannot regulate the torso attitude; the correct form drives the hips differentially. After the
> correction, the peak aerial pitch angle decreases from 36.0° to 11.4°, and the yaw drift per jump
> becomes 0.0°.
>
> (4) A decomposition of jumping capability into three independently designable quantities: the
> centre-of-mass ballistic rise is set by the extension impulse (133 mm, at which the knee motors
> already deliver 97% of their peak torque); the obstacle-crossing clearance is set by leg tucking
> during flight (213 mm versus 114 mm without tucking, at no additional torque cost); and the
> velocity retention is set by the duration of the ground phases (at 2.5 m/s the robot retains 85%
> of its forward travel over 2 s after take-off, with a minimum forward speed of +1.46 m/s and no
> backward motion).

---

## 三、草稿里每个数字的出处（可以自己复现）

所有数字都由 `paper/run_cases.py` 生成，跑一条命令即可复现：

```bash
cd ~/robot_ws/Two_wheeled_legged_robot/robot_sim
.venv/bin/python -m paper.run_cases --all     # 约 2 分钟
```

| 草稿里的数字 | 场景名 | JSON 字段 |
|---|---|---|
| 侧倾 8.38° → 0.07°（65 mm 坡） | `ff_ramp65_scale0p0` / `ff_ramp65_scale1p0` | `windows.obstacle.roll_p2p_deg` |
| 空中俯仰 36.0° → 11.4° | `jump_attitude_samesign` / `jump_attitude_diff` | `jump0_flight_pitch_peak_deg` |
| 偏航漂移 0.0° | `jump_amp_1p00` | `jump0_yaw_drift_deg` |
| 弹道 133 mm / 膝力矩 97% | `jump_amp_1p00` | `jump0_ballistic_mm` / `jump0_extend_leg_torque_peak` |
| 净空 213 mm vs 114 mm | `jump_tuck_on` / `jump_tuck_off` | `jump0_wheel_clearance_mm` |
| 行程保持率 85%、最低速 +1.46 m/s | `jump_run_2p5` | `jump0_travel_retention_2s` / `jump0_v_min_after_takeoff` |

### 两点说明

1. **归档里的 9.31° / 0.65° / 0.30° 与本文件的 8.38° / 0.44° / 0.07° 是两套口径**（场景时序略有差别）。
   论文里**只用一个来源**，建议用本文件的（因为一条命令可复现）。写法上可以只给首尾两个数
   （8.38° → 0.07°），中间那档留给图里的三条曲线。
2. **归档里的"飞行俯仰 4.7°"是"不收腿"工况下的数**；本文件的 11.4° 是**含收腿**的默认工况。
   贡献（3）和（4）都提到空中姿态，统一用同一工况的数（`jump_attitude_diff` = `jump_amp_1p00`）。

---

## 四、接下来怎么接

¶1 和 ¶4 写完之后，中间两段这样接：

| 段 | 内容 | 从哪来 |
|---|---|---|
| ¶2 相关工作 | 分类控制 / 轮腿平台 / 相位式跳跃三小块 | 文献矩阵表的横向归纳 |
| ¶3 缺口 | "共同做了什么" + "共同没做什么" | 文献矩阵表的"没做什么"那一列 |

顺序建议：先写 ¶2、¶3（等文献读完），再回头改 ¶1 和 ¶4 的措辞，
让四段之间不重复——**¶1 提问题、¶2 讲现状、¶3 指缺口、¶4 给答案**。
