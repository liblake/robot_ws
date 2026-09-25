# 论文写作手册（方案 A：纯仿真，6~8 页会议论文）

> 这份文档分三块：**第一部分**讲第一次写会议论文要知道的规矩，**第二部分**是逐章写作指导
> （每章该写什么、给可改的英文句式），**第三部分**是图表规范、时间表和自检清单。
> 目标是 7 页（IEEE 双栏，6~8 页弹性），5 章、6 图、5 表、正文约 3400 英文词。
> 会议格式与投稿杂事见 `PAPER_PLAN.md`。

---

# 第一部分：先建立正确预期

## 1.1 会议论文和学位论文的区别

| | 学位论文 | 会议论文（本篇） |
|---|---|---|
| 篇幅 | 几十上百页 | 6~8 页，含图表 |
| 读者 | 答辩委员会，会通读 | 审稿人只有 30~60 分钟，快速找"有没有新东西、数据可不可信" |
| 背景铺垫 | 可以写十几页综述 | 最多一段，直接进主题 |
| 失败尝试 | 可以写"试了 A 不行改 B" | **不能写**，只保留最终有效的方法 |
| 数据 | 越全越好 | 挑能支撑主张的，其余进附录或不写 |
| 图 | 可多可杂 | 每张图必须被正文引用，且只讲一件事 |

**最重要的一条**：会议论文不是实验日志。写作时你写的每一句话，要么是在**提出主张**，
要么是在**給出证据**，要么是在**解释证据**。凡是"过程叙述"（我先做了什么、后来发现不行、
又换了个方案）都要删掉。

## 1.2 这篇文章要论证的核心命题

> 一个统一的 LQR–VMC 控制框架（单一状态机 + 单一腿控制器 + 单一参数集），
> 可以覆盖双轮腿机器人的五种运动模态——平地行驶、转向、单腿变高度越障、波浪路地形跟随、跳跃——
> 并且在仿真中给出完整的能力包线。

全文所有章节都在为这一句话服务。**写每一段之前问自己：这段在支撑这句话吗？**不在就删。

## 1.3 写作的总体流程

```text
定框架（已完成，就是第二部分）
   ↓
先写 III 控制设计  →  再写 IV 结果  →  再写 II 系统与框架
   ↓
然后写 I 引言      →  最后写 V 结论和摘要
   ↓
填图表、补文献、语言润色、按格式自检
```

为什么这个顺序：控制设计和结果是你最熟、最有把握的内容；系统描述依赖前两章的符号；
引言要等全文写完才知道贡献怎么表述；摘要是全文的压缩，必须最后写。

## 1.4 第一次写论文最容易犯的八个错误

1. **把"我们做了什么"当贡献**。贡献是"读者能学到什么"，不是"我们干了多少活"。
2. **数字孤立地出现**。写"roll 峰峰 0.30°"必须带对照（"相比改造前的 9.31°"），否则读者不知道好坏。
3. **图没被正文引用**。每张图在正文里至少要有一句 "Figure 3 shows ..."。
4. **摘要里出现公式、引用或缩写未展开**。
5. **术语不统一**。同一个量一会儿叫 leg height、一会儿叫 body height、一会儿叫 CoM height，读者会混乱。
6. **时态混乱**。方法用现在时，具体实验操作用过去时（见 3.1 节）。
7. **limitation 不写**。审稿人一旦自己发现你没提，会认为你不可靠；主动写反而加分。
8. **中文思维直译**。"the robot very stable" 应写成 "the robot remained stable"。

---

# 第二部分：逐章写作指导

## 2.0 全文章节与篇幅分配

| 章 | 标题 | 页 | 词数 | 承载的命题 |
|---|---|---|---|---|
| I | Introduction | 0.8 | 450 | 为什么要做、做了什么 |
| II | System and Unified Control Framework | 1.4 | 650 | 平台是什么、框架"统一"在哪 |
| III | Multimodal Control Design | 2.0 | 950 | 五种模态各怎么控 |
| IV | Simulation Validation | 2.2 | 1100 | 五种模态各做到什么程度 |
| V | Discussion and Conclusion | 0.6 | 300 | 能推广吗、边界在哪、下一步 |
| — | Abstract + 关键词 | — | 150 | 全文压缩 |
| 合计 | | **7.0** | **3450** | 6 图 5 表 |

记住这个比例：**III 和 IV 合计占 60%**，它们是论文的主体。很多新手会花 3 页写背景，
结果核心内容只剩 2 页，审稿人会觉得"没东西"。

---

## 2.1 Title / Abstract / Keywords

### 标题

要求：**不能有副标题**，不能有特殊符号或公式，长度控制在 15 词以内，要包含"对象 + 做了什么"。

推荐（三选一）：

> A Unified LQR–VMC Framework for Multimodal Locomotion of a Serial-Leg Wheel-Legged Robot
>
> Unified LQR–VMC Control of a Serial-Leg Wheel-Legged Robot for Multimodal Locomotion and Jumping
>
> Multimodal Locomotion Control of a Serial-Leg Wheel-Legged Robot Based on a Unified LQR–VMC Framework

### 摘要（约 150 词，5 句，不要公式和引用）

摘要就是全文的压缩，用固定的五句结构，每句一句英文：

| 句 | 作用 | 模板（把方括号里的内容换成你的） |
|---|---|---|
| 1 | 背景（1 句） | Wheel-legged robots combine the energy efficiency of wheeled platforms with the obstacle-crossing capability of legged systems. |
| 2 | 缺口（1 句） | However, most reported controllers are validated for a single locomotion mode, and it remains unclear how far one controller can cover flat-ground driving, turning, asymmetric leg-length obstacle crossing, and dynamic jumping. |
| 3 | 本文做什么（1~2 句） | This paper presents a unified LQR–VMC control framework for a 17 kg serial-leg bipedal wheel-legged robot, in which a single phase-based state machine, a single leg controller, and a single parameter set cover five locomotion modes. |
| 4 | 结果（1~2 句，必须带数字） | In simulation, the robot tracks speeds up to 2.5 m/s with 0.001 m/s steady-state error, keeps the body level within 0.37° of roll while one wheel crosses a 65 mm step, and achieves a 133 mm center-of-mass ballistic rise with 223 mm wheel clearance during jumping. |
| 5 | 意义（1 句） | A control-loop delay budget of 8 ms, together with insensitivity to ±30% mass variation, indicates that the framework is implementable on a physical platform. |

**写完摘要后自检**：① 有没有出现公式？② 有没有出现参考文献编号？③ 有没有出现没解释的缩写？
④ 五句话是不是都在支撑同一个命题？

### 关键词（5~6 个）

> wheel-legged robot; LQR; virtual model control; multimodal locomotion; jumping control; obstacle crossing

关键词要能被检索到，用领域里的常见词，不要自创术语。

---

## 2.2 I. Introduction（450 词，四段）

### 这一章要回答的问题

① 这个方向为什么重要？② 别人做到哪一步了？③ 还缺什么？④ 你补上了什么？

### 段落安排

**¶1 背景（3~4 句）**

> 主题句模板：Wheel-legged robots combine the energy efficiency of wheeled locomotion with the terrain adaptability of legged systems, and have been demonstrated on platforms such as [平台 A], [平台 B], and [平台 C] [1]–[3].

接着一两句说明"多模态"为什么难：坐立、行驶、转向、越障、跳跃对控制器提出的需求不同，
平地上好用的参数到了跳跃就可能失稳。

**¶2 相关工作（5~6 句）**

分三个小块写，每块 1~2 句，每句都要带引用：

- 轮腿平台：Handle、Ascento、ANYmal on Wheels 等。
- 分层平衡控制：LQR 或 MPC 做平衡、VMC 或关节 PD 做腿，速度/航向用外环。
- 跳跃控制：相位划分 + 轨迹跟踪 + 腾空姿态调节。

**¶3 缺口（3~4 句，这一段是全文最关键的一段）**

> 模板：These works, however, evaluate [某种模态] in isolation. It remains unclear whether a single controller, with one set of parameters, can cover [模态 1]、[模态 2]……. In particular, [具体难点 1]，[具体难点 2]。

写这一段时要具体。比如："在单侧轮子骑上台阶时，两条腿的目标高度必须不同，而跳跃所需的腿长轨迹又与地形跟随的目标冲突。"

**¶4 贡献（5~6 句，逐条列）**

> 模板：The contributions of this paper are as follows. First, we present a unified LQR–VMC framework in which ... . Second, we show that ... . Third, we quantify ... . Finally, we report a control-loop delay budget of 8 ms ... .

**每条贡献后面必须跟一个数字或一个可验证的结论**，否则读者不知道这条贡献值多少。

### 文献怎么用（第一次写最卡的地方）

先纠正一个概念：**会议论文的 Introduction 不是"文献综述"**。综述是学位论文的一章，
要穷尽、要分类、要评述；会议论文里的文献只有一个用途——**证明缺口真实存在**。
在你这个定位下，文献只需要证明两件事：① 分层控制与相位式跳跃都有人做过（说明你的方法是合理路线）；
② 没有人用同一套控制器与同一组参数同时覆盖你要写的这几种模态（说明缺口存在）。

**需要几篇**：正文至少引用 5 篇（格式硬要求），实际建议读 15~25 篇、引用 8~15 篇。
原则是**每一篇被引文献都要在正文里承担一句话的论证责任**，凑数的直接删。

**怎么找**（Google Scholar / IEEE Xplore / Scopus / arXiv，按下面四类关键词组合检索）：

| 类别 | 检索词示例 |
|---|---|
| 平台 | `wheel-legged robot`、`wheeled biped robot`、`wheeled-legged balancing` |
| 分层控制 | `LQR balance control wheel-legged`、`virtual model control legged robot`、`hierarchical locomotion control` |
| 跳跃 | `jumping control legged robot`、`flight phase attitude control`、`landing impact absorption` |
| 地形 | `rough terrain locomotion wheeled`、`asymmetric leg length control`、`undulating terrain following` |

筛选顺序：年份（近 5 年优先）→ 有无 DOI → 期刊/会议层次 → 摘要里有没有你要的那类结果。

**怎么读**（不要逐字读，用三遍法）：

1. 只读标题 + 摘要 + 结论，判断"做了什么、做到什么程度"；
2. 看系统构型图与主要结果图，判断它验证了哪些模态；
3. 只有决定引用它时，才读方法细节。

**读完要产出一张文献矩阵表**，这是写缺口段最直接的素材：

| 文献 | 平台/构型 | 方法 | 验证过哪些模态 | 没做什么 |
|---|---|---|---|---|
| [1] | 双轮腿，并联腿 | MPC + 阻抗 | 平地行驶、跳跃 | 无地形适应、无单腿变高度 |
| [2] | …… | …… | …… | …… |

把最后一列竖着看，缺口段基本自动成型：如果几篇都没做"单腿变高度越障"，那一栏就是你的缺口。

**引用怎么写**：

- 上标方括号：`... has been demonstrated[1], [3].`
- **成组引用**：一句话挂两三篇（`[1]–[3]`），不要一句一篇地罗列，那样一眼就是凑数。
- 时态：`Several works have demonstrated ...`、`Most existing controllers focus on ...`。
- 只保留"做了什么 / 没做什么"，不要在引言里评价别人的参数细节。

**三个可以直接改的句式**：

> 归纳（¶2）：Prior work on balancing control of wheel-legged robots has largely followed a
> hierarchical structure, in which an LQR or MPC regulates the pitch dynamics while a leg
> controller tracks the leg length[3]–[5].

> 转折（¶2 末）：Jumping has been addressed by phase-based trajectory planning[6], [7];
> however, these studies evaluate a single locomotion mode in isolation.

> 缺口（¶3 核心句）：To the best of our knowledge, no prior work reports a single parameter set
> that covers flat-ground driving, single-side obstacle crossing, and dynamic jumping on the
> same platform.

**关于中文文献**：格式要求"被引作者来自 3 个以上国家"，中文文献最多放 1 篇作为同路线工作，不能用来凑数。

### 常见错误

- 第一段就开始讲自己的机器人（应该先讲领域）。
- **把 Introduction 写成读过的论文清单**（那是学位论文的综述章，不是会议论文）。
- 引用只挂在句子末尾、不与论证发生关系（"某某做了甲[1]，某某做了乙[2]"）——
  要写成"这一类工作共同做了什么、共同没做什么"。
- 相关工作写成文献罗列，没有指出"别人没做什么"。
- 贡献写得太虚（"提出了一个先进的控制框架"）。要写成"提出了 X，它在 Y 条件下把 Z 从 A 提升到 B"。
- 把"研究重点"直接等同于"贡献"：说"本文研究多模态运动"只是研究重点，
  说"一个控制器覆盖五种模态，且单腿变高度越障时侧倾保持在 0.37° 以内"才是贡献。

---

## 2.3 II. System and Unified Control Framework（650 词）

### 这一章要回答的问题

① 这台机器人长什么样？② 控制框架分几层？③ **"统一"具体体现在哪？**（第三点最重要）

### 段落安排

**A. 平台描述（约 200 词）**

机构构型（左右各一条串联 2 自由度腿，末端装驱动轮）、质量分布、连杆长度、腿高区间、
电机规格。这里配 **Table I**（平台参数表）和 **Figure 1**（机器人 + 坐标系 + 腿几何）。

> 句式模板：As shown in Figure 1, the robot consists of a torso and two serial 2-DOF legs. Each leg is actuated by a hip joint and a knee joint, with a driven wheel mounted at the end of the shank.

一句话交代设计细节（这种"设计理由"很讨审稿人喜欢）：轮心相对髋关节后移 75 mm，
使整机质心落在轮轴上方，否则静态平衡角约 13°。

**B. 单腿运动学与雅可比（约 150 词）**

这一小节是第 III 章所有公式的基础，不能省：

1. 2R 平面逆运动学：给定腿高 $h$ 与轮心纵向偏移 $e$，解出髋、膝关节角。
   这一段只需要给出几何关系与符号，推导过程可以省略（会议论文不要求完整推导）。
2. 腿高雅可比 $J_j=\partial h/\partial q_j$，并给出本机的数值范围（髋约 0.075、膝 0.196~0.225 m/rad）。
3. 由虚功给出**竖向推力与关节力矩的换算关系**：竖向推力 $F$ 对应的关节力矩为 $\tau_j = F \cdot J_j$。
   这一句会在第 III 章的支撑前馈和跳跃蹬伸里反复用到。

配 **Figure 1**（腿几何标注）。

**C. 统一框架（约 250 词）**

这是本章的核心。三层结构 + 相位调度：

1. **命令层**：速度、转向、机身高度、跳跃触发。
2. **平衡层**：LQR 处理俯仰与侧倾，外环把速度/航向误差转成参考量。
3. **腿层**：VMC（虚拟模型控制）在关节空间跟踪腿长目标，含重力前馈。

**然后明确写出"统一"的三条定义**（这是本文的卖点，一定要写清楚）：

| 统一的维度 | 具体含义 |
|---|---|
| 同一个状态机 | 六个相位（STAND / CROUCH / EXTEND / FLIGHT / LAND / FALLEN）驱动全部模态 |
| 同一个腿控制器 | 平地调高、单腿变高度越障、跳跃蹬伸、落地吸收都用同一套关节 PD + 前馈，只换参考量 |
| 同一组参数 | 模态切换只改变参考量与限幅，不重新整定控制器参数 |

配 **Figure 2**（控制架构框图，带一条相位时间轴）和 **Table II**（模态 × 通道 × 参数复用表）。

> Table II 是"统一性"的最强证据，务必做。它长这样：

| 模态 | 平衡通道 | 腿通道 | 是否改变控制器参数 |
|---|---|---|---|
| 平地行驶 | LQR + 速度外环 | 腿高恒定 | 否 |
| 转向 | LQR + 航向外环 | 腿高恒定 | 否 |
| 单腿变高度越障 | LQR | 左右腿高差 | 否（只改参考） |
| 波浪路 | LQR | 地形跟随 | 否 |
| 跳跃 | 相位机接管 | 轨迹 + 姿态控制 | 仅相位内增益缩放 |

**D. 章节导引（约 50 词）**

一句话过渡：控制设计按模态组织，见第 III 章；验证见第 IV 章。

### 常错点

- 把"统一框架"写成口号，没有 Table II 这种可检验的东西。
- 平台参数表里单位不写（质量 kg、长度 m、力矩 N·m 都要标）。
- 漏掉腿的逆运动学与雅可比——后面所有前馈公式都要用它，读者会跟不上。

---

## 2.4 III. Multimodal Control Design（950 词，全篇最核心）

### 这一章要回答的问题

**每一种模态是怎么控制的？** 按模态分成四节，每节写清"控制目标 → 控制律 → 为什么这样设计"。

写这一章的三条纪律：

1. **每个公式都要被用到**。不要为了显得"数学"而堆公式；出现一个公式，后面结果里就要体现它。
2. **公式前后要有"散文"**。IEEE 论文的标准写法是：一句话引出公式，公式，一句话解释符号，
   再一句话说明它的作用。见下面的公式写作模板。
3. **每个设计选择都要给理由**。写"我们把 D 项增益降到 0.15 倍"不如写"我们把 D 项降到 0.15 倍，
   因为原始增益与关节—地面接触刚度形成两步极限环，实测力矩全程饱和"。

### 公式写作模板（照这个结构写，审稿人读起来最舒服）

> The joint torque required to support the vehicle is written as
>
>     τ_j = J_j·(m·g)/N + Σ m_i·g·(∂z_i/∂q_j)          (1)
>
> where m is the total mass, g is the gravitational acceleration, N is the number of legs,
> and J_j = ∂h/∂q_j is the Jacobian relating the leg height h to the joint angle q_j.
> The first term represents the load path from the vehicle weight through the wheels,
> and the second term accounts for the self-weight of the leg links.

（"一句话引出 → 公式 → 符号解释 → 物理含义"四步，全文公式都这么处理。）

### A. 平地行驶与转向（约 250 词）

**写什么**：

1. 平衡回路：状态 $[\phi-\phi_\mathrm{eq},\ \dot\phi]$，控制律 $\tau_w = -K[\phi-\phi_\mathrm{eq},\ \dot\phi]^\top$，
   给出 $K=[-45.0,-7.0]$。
2. **必须写模型标定这一段**（这是审稿人喜欢看的"工程严谨性"）：简化倒立摆模型预测的单步俯仰响应
   与实测差 3.6 倍（实测 +1 N·m 产生 −0.0243 rad/s，模型预测 −0.00675），
   原因是轮子与传动的瞬态以及接触摩擦延迟，因此增益按实测标定。
3. 速度外环：速度误差 → 目标俯仰（增益 0.35，限幅 ±0.2 rad），带积分门控抗饱和。
4. 航向外环：转向指令 → 差动轮力矩（阻尼 8、积分 6、输出限幅 2.0 N·m）。
   解释限幅理由：正常转向只需约 0.8 N·m，留量程给平衡通道。

**结果对应**：平地速度跟踪 rms 0.015 m/s、转向指令 0.5 rad/s 实测 0.500 rad/s。

### B. 单腿变高度越障（约 250 词）★ 本节的亮点

**模态定义**（先写清楚，别让读者猜）：一侧轮子骑上台阶或坡面、另一侧仍在平地时，
两条腿的目标高度必须不同，机身才能保持水平。

**控制律**：用左右轮心高度差生成腿高偏置

    Δh_L = −0.5·(z_L − z_R) + K_p·ψ + K_d·ψ̇ ,      Δh_R = −Δh_L          (2)

其中 ψ 是机身侧倾角。偏置经过 ±0.035 m 限幅后加到该侧腿高目标上，再送入腿层控制器。
这段要写清：**这一项是前馈（由地形几何直接算出），比例项默认置零**，因为实测反馈项会引入振荡。

**然后写本节最有价值的那个发现**（这是让审稿人记住这篇论文的地方）：

> 仅靠腿高偏置前馈，机身侧倾改善有限（65 mm 坡的 roll 峰峰 9.31°）；
> 真正起作用的是**让腿的关节角速度前馈生效**——腿的目标角速度由腿高变化率经雅可比换算而来，
> 它让腿主动去追移动中的地形目标，而不是被动地跟在后面。
> 恢复这一项后，同一个工况的 roll 峰峰降到 0.30°，降低约 30 倍。

**再补一句因果解释**：车身侧倾近似等于（左右轮高差 + 左右腿指令高差）/ 轮距，
所以"机身不水平"的本质是"腿没跟上指令"，而不是姿态参考设错了。

**结果对应**：65 mm 单轮坡 roll 峰峰 0.27~0.37°；波浪路 0→1.0 m/s 通过。

### C. 跳跃（约 300 词）★ 篇幅最大的一节

按"轨迹规划 → 推力前馈 → 腾空姿态 → 落地吸收"四步写，每步一到两句。

**1. 相位划分与轨迹**

六个相位：STAND → CROUCH → EXTEND → FLIGHT → LAND →（FALLEN）。
CROUCH 与 LAND 用五次多项式（两端位置、速度、加速度给定），EXTEND 用恒加速度轨迹，
目的是在有限行程内把能量尽量传递出去。给两个公式：

    v_to = sqrt(2·g·h_air) ,        T_ext = 2·Δh / v_to          (3)

**2. 动态推力前馈**（本节的技术核心）

把支撑前馈里的 g 换成 g + ḧ_ref，两项一起缩放，就得到蹬伸所需的推力前馈。
要写一句重要的实现细节：**这一项不按接触门控**——蹬伸到最后会瞬间离地，
而那正是最需要推力的时候。

然后给一个必需的比例关系（不写审稿人会问"目标 30 cm 为什么只跳 13 cm"）：

> 质心的竖向速度约为腿高变化率的 0.63 倍，因此实测弹道约为目标高度的 0.4 倍。

**3. 腾空姿态控制**

给出镜像关节的机理：左右髋关节轴在矢状面镜像，两侧施加**同号**关节力矩时，
世界系力矩互相抵消，只摆动腿而不改变机身姿态；正确做法是两侧施加**反号**力矩，

    τ_R = +τ_att ,   τ_L = −τ_att ,   τ_att = clip(−K_d·φ̇, ±6 N·m)          (4)

由此得到可推广的规律：**俯仰类控制量取髋关节力矩之差，竖直推力类取之和。**
再补一句实验佐证（很有说服力）：在此修正之前，俯仰阻尼增益从 1.5 提高到 4.5
对飞行俯仰几乎没有影响；修正后飞行俯仰峰从 12.4~14.9° 降到 4.7°。

另外说明**只加阻尼不加位置项**的理由：飞行期存在约 +2.3 N·m 的恒定偏置力矩，
加入位置项会形成正反馈（实测俯仰发散到 ±0.6 rad）。

**4. 腾空收腿与落地吸收**

收腿时序按归一化滞空时间 τ = t/t_f 分段：τ < 0.15 保持蹬直 → 0.15~0.45 收腿 →
0.45~0.62 保持蜷缩 → 0.62~0.92 展开 → 落地前到位。要写一句实现细节：
收腿 PD 的微分项必须带目标角速度前馈，否则阻尼项会对抗收腿运动本身，蜷缩深度损失约一半。

落地用独立的 PD 增益（$k_p=80$、$k_d=8$，对应阻尼比约 0.65），并按
"任务空间等效刚度"重新标定，而不是沿用平地增益。

**结果对应**：弹道 133.0 mm、蹬伸膝力矩峰 58.1 N·m、落地恢复 0.40 s、
飞行俯仰 4.7°、轮子净空 223 mm。

### D. 波浪路地形跟随（约 150 词）

这一节短，但要写清"为什么它属于统一框架的一部分"：波浪路连续改变轮子的等效高度，
等效于**持续的单腿变高度越障**，因此复用同一套解析前馈与腿高跟踪。

**对比句**（这种"改前 vs 改后"的句子是审稿人最想看的）：

> 在使用逆动力学前馈时，1.0 m/s 通过波浪路会失稳；改用解析式支撑前馈后，
> 同一工况可通过，轮子离地时间占比降到 18.9%。

### 常错点

- 只写公式不讲动机。
- 把四节写成四份独立的说明书，不说明它们共用了什么（那样"统一"就立不住了）。
- 忘记在每节末尾用一句话回扣主题："这一模态同样只使用第 II 章给出的同一组控制器参数。"

---

## 2.5 IV. Simulation Validation（1100 词）

### 这一章要回答的问题

**每一种模态到底做到了什么程度？** 以及"这些结论可信吗？"

### 写结果的通用结构（每小节都照这个套路）

```text
一句话交代工况（图/表引用）
   ↓
给出关键数字（并与对照对比）
   ↓
解释为什么是这个结果（一句因果）
```

英文模板：

> Figure 4(a) shows the roll angle while the left wheel crosses a 65 mm step at 0.3 m/s.
> The peak-to-peak roll angle decreases from 9.31° with the inverse-dynamics feedforward
> to 0.30° with the proposed analytic feedforward. The improvement is attributable to the
> joint-rate feedforward being effective, which allows each leg to track its moving
> terrain-induced height target instead of lagging behind it.

注意这个结构里没有任何"我们觉得很不错"的主观评价——**全部是数字和因果**。

### 小节安排

**A. 仿真设置与指标（约 150 词）**

写明：仿真器与版本、积分步长、控制周期、地形定义（单轮梯形坡 20/40/65 mm、波浪路、
垂直台阶 5/10/15 cm）、指标定义（速度跟踪 rms、roll 峰峰值、离地占比、质心弹道、
落地俯仰峰、蹬伸力矩峰）。配 **Table III**（指标定义表）。

一句技术性提醒也要写：所有前向速度都按本体坐标系投影计算，而不是直接取世界系速度分量。

**B. 平地与转向（约 200 词）**

配 **Table IV**。列：站立（俯仰峰 2.6°、漂移 8 mm）、速度跟踪（0.5 m/s 时 rms 0.015 m/s）、
加速与刹车（0.80 s / 279 mm）、转向（指令 0.5 → 实测 0.500 rad/s）、腿高跟踪（误差 ≤5 mm）、
速度扫描（0.8~2.5 m/s，跟踪误差 ≤0.001 m/s）。

最后加一句"天花板"：仿真在 3.0 m/s 仍稳定，但驱动电机的 350 rpm 限制对应 2.57 m/s，
这是平台的能力上限。**主动写上限是加分的**，说明你知道边界在哪。

**C. 单腿变高度越障（约 200 词）**

配 **Figure 4**：(a) 65 mm 单轮坡的 roll 时程三档对照（9.31° / 0.65° / 0.30°）；
(b) 波浪路离地占比 vs 车速。不同坡高 × 车速的完整矩阵并入 **Table IV**；
若版面紧张，正文只写 65 mm 这一组，其余用"20~65 mm 坡高范围内均通过"概括。

结论句模板：

> These results indicate that asymmetric leg-length modulation, combined with an accurate
> support feedforward, keeps the torso level within 0.4° even when one wheel crosses an
> obstacle four times taller than the leg-height tracking error.

**D. 跳跃（约 250 词）**

配 **Figure 5**：(a) 飞行俯仰时程（修正前 / 后）；(b) 收腿开关的轮子净空柱状对比。
**Table V** 给汇总：弹道 133.0 mm、蹬伸膝力矩峰 58.1 N·m（占电机峰值 97%）、
落地恢复 0.40 s、落地俯仰峰 4.1°、每跳偏航漂移 0.0°、三连跳一致。

然后写本节的核心结论（属于"跳跃能力三分解"）：

> 质心弹道受膝关节电机峰值限制（133 mm 时已用到 97%），
> 而轮子的越障净空可以通过腾空收腿进一步提高到 223 mm，**不需要增加任何力矩需求**。

**E. 鲁棒性与延迟预算（约 200 词）**

这是"可信度"小节，回答"换个世界还行不行"：

- 质量 ±30%、机身质量单独 ±30%、质心前后 ±5 cm、摩擦 ×0.5/×2、控制增益 ±20%、
  IMU 姿态噪声 ≤1.0°、编码器 12 bit：全部通过。
- 控制回路延迟：4/6/8/10/12/16/20 ms 对应的俯仰峰为 0.031/0.045/0.164/0.264/0.431 rad，
  20 ms 失稳（8 ms 以内全部合格）。

结论句模板：

> The framework tolerates ±30% mass variation and ±20% gain variation, which indicates that
> the parameter set is not sharply tuned; the dominant practical constraint is the control-loop
> delay, which must remain below 8 ms for the nominal performance reported here.

**F. 向实机的迁移可行性（约 100 词）**

方案 A 不写实机实验，但要给一段"诚实且有用"的收尾：执行器限幅用的是实机选型的电机规格，
时间尺度上控制周期为 2 ms；把 8 ms 延迟预算、接触检测需求、参数标定需求各写一句，
明确列为后续工作。**这段写得越具体，审稿人越不会追问"为什么不做真机"。**

### 常错点

- 表格里没有单位。
- 只报最好的一组数据，不报最差的一组（审稿人信不过）。
- 图里只画一条曲线，没有对照——**每条结论曲线都要有基线**。

---

## 2.6 V. Discussion and Conclusion（300 词）

分三段：可推广性、局限、结论与展望。

**¶1 可推广性（约 100 词）**

> 三条可迁移的结论：① 支撑前馈必须用解析式而非仿真器逆动力学；② 镜像关节腿的俯仰控制必须用差分驱动；
> ③ 跳跃高度、越障净空、速度保持分别由蹬伸冲量、收腿量、着地相位时长决定，三者可以独立设计。

**¶2 局限（约 100 词）**

老实写：全部结果来自仿真；落地冲击导致的约 16% 速度损失尚未消除；
控制器目前依赖接触状态判定，实机需要另行实现；跳跃所需膝力矩达电机峰值的 97%，
连续跳跃需要降低幅度。

**¶3 结论与展望（约 100 词）**

> In this paper, we presented a unified LQR–VMC control framework ... . Simulation results
> show that ... . Future work will focus on task-space landing control to reduce the residual
> velocity loss and on porting the framework to a physical prototype.

结论章不要引入新数据、新公式、新引用，只做总结。

---

# 第三部分：规范、图表、时间表与自检

## 3.1 学术英语速查

### 时态

| 写什么 | 时态 | 例子 |
|---|---|---|
| 方法、架构、公式 | 现在时 | The controller consists of three layers. / The torque is computed as ... |
| 具体实验操作 | 过去时 | The left wheel was commanded to cross a 65 mm step at 0.3 m/s. |
| 图表呈现 | 现在时 | Figure 3 shows ... / Table II lists ... |
| 结果陈述 | 现在时 | The peak roll angle decreases from 9.31° to 0.30°. |
| 结论与展望 | 现在时 / 将来时 | Future work will focus on ... |

全篇保持一致，不要一段里来回换。

### 语态

IEEE 会议论文以**被动语态**为主（"The gain was tuned by ..."），但**主动语态更适合陈述贡献**
（"We propose ...", "This paper presents ..."）。简单原则：讲方法用被动，讲贡献和结论用主动。

### 数字写法

- 单位与数字之间留空格：`65 mm`、`0.3 m/s`、`58.1 N·m`（度例外：`4.7°`）。
- 数值精度要和测量精度匹配：腿高误差写 `≤5 mm` 而不是 `≤5.0000 mm`。
- 不要把小数写成中文式"0.3 m/s 左右"，要写 `approximately 0.3 m/s`。
- 比较句要给两个数：`from 9.31° to 0.30°`，不要只写 `0.30°`。

### 常见中式英语对照

| 别这么写 | 改成 |
|---|---|
| The robot is very stable. | The robot remained stable throughout the 12 s test. |
| We did a lot of experiments. | Five locomotion modes were evaluated in simulation. |
| The result is very good. | The tracking error is 0.015 m/s, one order of magnitude smaller than that of the baseline. |
| The robot can not fall down. | The robot did not fall in any of the 25 perturbation cases. |
| This method has many advantages. | This method reduces the peak roll angle by a factor of 30. |

**核心原则**：把形容词换成数字。审稿人只信数字。

### 缩写

首次出现给出全称，之后统一用缩写：`virtual model control (VMC)`、`center of mass (CoM)`、
`linear quadratic regulator (LQR)`。摘要和正文要各自独立给出一次（有些会议要求摘要也展开）。

---

## 3.2 图表规范

### 图表标题的写法（IEEE 规范，检查清单里明确要求）

- 图的标题写在图**下方**，用全称 `Figure 1.`（不能简写 Fig.），**结尾必须有句号**。
- 表的标题写在表**上方**，用全称 `Table I.`，**结尾必须有句号**，并且用罗马数字编号。
- 标题要能独立看懂，例如 `Figure 4. Roll angle while the left wheel crosses a 65 mm step at 0.3 m/s.`

### 六张图怎么画

| 编号 | 内容 | 画法要点 |
|---|---|---|
| **Figure 1** | 机器人 + 坐标系 + 腿几何 | 由 URDF 渲染或 CAD 截图；标注 $\theta_h$、$\theta_k$、$L_1$、$L_2$、$e$；**全部英文** |
| **Figure 2** | 控制架构框图 | 三层纵向排列，右侧画一条相位时间轴（STAND→CROUCH→EXTEND→FLIGHT→LAND） |
| **Figure 3** | 单腿变高度越障示意 | 画两条腿在台阶/坡两侧的构型 + 腿高差 Δh 与机身水平的关系 |
| **Figure 4** | 越障结果：(a) 65 mm 坡 roll 三档对照；(b) 波浪路离地占比 vs 车速 | 横轴时间 s / 车速 m/s，纵轴 roll (°) / 离地占比 (%)；三条线要同一量纲、图例写清 feedforward 类别 |
| **Figure 5** | 跳跃结果：(a) 飞行俯仰时程；(b) 收腿开关的净空柱状图 | (a) 两条线（修正前/后）+ 起飞与落地时刻的竖虚线；(b) 两柱 + 数值标注 |
| **Figure 6** | 行驶中跳跃的速度曲线 | 横轴时间，纵轴前向速度；两组曲线（着地相位 0.25 s vs 0.08 s），标注行程保持率 |

出图硬要求：**全英文标注、字号 ≥8 pt、导出 PDF + 300 dpi PNG**，图内不得出现中文。
所有曲线用同一套配色（建议 3 色以内），线宽 ≥1.2，虚线只用于对照。

### 五张表怎么做

| 编号 | 内容 | 列头建议 |
|---|---|---|
| **Table I** | 平台参数 | Parameter / Symbol / Value / Unit |
| **Table II** | 模态 × 通道 × 参数复用（统一性的证据） | Mode / Balance channel / Leg channel / Parameter change |
| **Table III** | 指标定义 | Metric / Definition / Unit |
| **Table IV** | 平地与转向性能 | Scenario / Metric / Value |
| **Table V** | 跳跃与越障性能 | Quantity / Value / Limit |

表格里所有数值都要带单位（写在列头里更省地方），并且**不要用截图**（会议要求表格可编辑）。

---

## 3.3 公式与符号

1. 公式必须用公式编辑器（Word 用 Office 公式编辑器或 MathType，LaTeX 用 `equation` 环境）。
2. 公式编号 (1)(2)(3) 按出现顺序，**编号要放在公式外面**，不能塞进公式编辑器里。
3. 每个符号第一次出现时必须定义，之后不要改含义。
4. 建议的符号表（全文统一）：

| 符号 | 含义 |
|---|---|
| $h$ | 腿高（机身原点到轮心的高度差） |
| $q_h, q_k$ | 髋关节角、膝关节角 |
| $J_j = \partial h/\partial q_j$ | 腿高对关节角的雅可比 |
| $m, N, g$ | 整车质量、腿数、重力加速度 |
| $\phi, \psi$ | 机身俯仰角、侧倾角 |
| $v$ | 机身前向速度 |
| $\tau_j$ | 关节电机力矩 |
| $\ddot h_\mathrm{ref}$ | 轨迹期望腿高加速度 |
| $t_f$ | 预估滞空时间 |

---

## 3.4 引用与文献

1. 正文里的引用用**上标方括号**：`... has been demonstrated on several platforms[1], [2].`
2. 每条文献必须有 **DOI 或链接**，必须有**年份**。
3. 检查清单要求**至少 5 条**，且**被引作者来自 3 个以上不同国家**。
   建议这样配：轮腿平台 2~3 篇（欧美）、分层控制 1~2 篇、跳跃控制 1~2 篇（美国/日本）、
   中文相关论文 1 篇。
4. **每条文献都要在正文中被引用过**，不能只挂在参考文献表里。
5. 用文献管理工具（Zotero / EndNote）导出，不要手打，容易掉 DOI。

---

## 3.5 写作时间表（按 9 个工作日排）

| 天 | 任务 | 产出 |
|---|---|---|
| D1 | 定稿图表数据的来源脚本；跑一遍确认数字（见 3.7） | 确认后的数字台账 |
| D2 | 写第 III 章 A、B 节（平地转向、单腿变高度越障） | 中文草稿 |
| D3 | 写第 III 章 C、D 节（跳跃、波浪路） | 中文草稿，第 III 章完成 |
| D4 | 写第 IV 章 A、B、C 节（设置、平地、越障） | 中文草稿 |
| D5 | 写第 IV 章 D、E、F 节（跳跃、鲁棒性、迁移） | 第 IV 章完成 |
| D6 | 写第 II 章（平台与统一框架，含 Table II） | 第 II 章完成 |
| D7 | 写第 I 章（引言四段 + 贡献） | 第 I 章完成 |
| D8 | 写第 V 章 + 摘要，全文译成英文 | 完整英文初稿 |
| D9 | 出图 + 补文献 + 语言润色 + 按格式自检 | 投稿版 |

如果先写中文再翻英文，注意别直译，尤其是"通过…实现了…"这类句式要改成主谓结构。

---

## 3.6 投稿前自检清单

### 内容层面

- [ ] 摘要五句话齐全，含至少两个数字，无公式、无引用、无未展开缩写
- [ ] 引言第 3 段（缺口）明确说出"别人没做什么"
- [ ] 每条贡献都能在结果章找到对应的小节和数字
- [ ] 每张图、每张表都在正文里被引用至少一次
- [ ] 每个结论句都有数字或对照支撑，没有"效果很好"这类主观表述
- [ ] 说明了哪些结论来自仿真、边界在哪里
- [ ] 局限性写了至少三条

### 语言层面

- [ ] 时态统一（方法现在时、实验过去时）
- [ ] 术语统一（腿高只用 leg height，前向速度只用 forward velocity）
- [ ] 没有中式直译的形容词堆砌
- [ ] 缩写首次出现有全称

### 格式层面

- [ ] 页面：Letter、双栏、页边距符合模板
- [ ] 篇幅：4 ≤ 页数 ≤ 20（目标 7~8）
- [ ] 图：全英文、高清、标题写在图下方、以 `Figure n.` 开头并以句号结尾
- [ ] 表：可编辑、标题写在表上方、以 `Table n.` 开头并以句号结尾、单位齐全
- [ ] 公式：用公式编辑器、编号在公式外部、按顺序编号
- [ ] 文献：≥5 条、每条有 DOI/链接、作者来自 ≥3 个国家、正文全部引用过

---

## 3.7 写作前要确认的数据（跑一遍再动笔）

1. **波浪路数字必须重跑**：`test_wavy_road.py` 目前没传 `terrain=`，跑的是平地。
   重跑后在脚本里加一句"地形 geom 存在"的断言，避免再犯。
2. **要画 roll 曲线就先修 `state.py::_roll_from_quaternion`**：遥测里 roll 与 pitch 互为相反数，
   不修的话 Figure 4(a) 是错的。
3. **弹道与净空按当前代码默认值重跑**：归档里收腿深度记 0.33，代码默认已是 0.35。
4. **容易记错的数字**：

| 项目 | 值 |
|---|---|
| 整车质量 / 每腿 | 17.0 kg / 3.49 kg（21%） |
| 大腿 / 小腿 / 轮半径 | 0.300 / 0.34325 / 0.070 m |
| 腿高区间 / 名义站高 / 轮心偏移 | 0.31~0.50 m / 0.37 m / −0.075 m |
| 电机峰值 | 膝 ±60 N·m、髋 ±90 N·m、轮 ±9 N·m / 350 rpm |
| 平衡增益 / 标定系数 | K = [−45.0, −7.0]；B 的 pitch 行 ×3.6、wheel 行 ×0.3 |
| 站立性能 | 俯仰峰 2.6°、漂移 ≤8 mm |
| 速度跟踪 | 0.5 m/s 时 rms 0.015 m/s；0.8~2.5 m/s 误差 ≤0.001 m/s |
| 刹车 | 0.83 s / 279 mm |
| 转向 | 指令 0.5 rad/s → 实测 0.500 rad/s |
| 腿高跟踪 | 稳态误差 ≤5 mm（最新 ≤1.1 mm） |
| 65 mm 单轮坡 roll 峰峰 | 9.31°（无角速度前馈）/ 0.65° / 0.30°（默认） |
| 波浪路 1.0 m/s | 旧方案失稳；新方案离地 18.9%、roll 0.2° |
| 前馈残差 \|qacc\|（站立/直行/转向） | 解析 0.00 / 0.07 / 5.86；旧 0.00 / 0.05 / 133.60 |
| 跳跃弹道 / 收腿净空 | 133.0 mm / 223 mm（不收腿 109 mm） |
| 蹬伸膝力矩峰 | 58.1 N·m（峰值 60） |
| 落地恢复 / 落地俯仰峰 | 0.40 s / 4.1° |
| 飞行俯仰峰（修正前/后） | 12.4~14.9° / 4.7° |
| 每跳偏航漂移 | +5.0° → 0.0° |
| 行驶跳行程保持率 | 0.5/0.8/1.5/2.0/2.5 m/s → 90/91/89/87/84% |
| 延迟预算 | 2/4/6/8 ms 全部合格；10/12/16 ms 最坏指标为门槛的 1.2/2.3/8.0 倍；20 ms 失稳 |
| 实机速度上限 | 2.57 m/s（350 rpm × 0.070 m 轮径） |

---

# 第四部分：中文初稿 → 英文翻译

先写中文再翻英文是可行的，但**中文稿的写法决定了翻译有多痛苦**。按下面的约束写，
翻译基本是"换语言"；不按约束写，翻译会变成"重写"。

## 4.1 中文初稿的六条约束

1. **一段一个主题句**。段首那句就是论点，后面全是证据。不要把三个论点塞在一段里。
2. **一句话只表达一个意思**。中文习惯用长句串联（"在…的情况下，通过…，实现了…，从而使…"），
   英文会变成无法拆解的怪物句。写短句。
3. **不写"过程"**。凡是"我们先试了 A，发现不行，后来改成 B"，只保留最终方案，
   最多在解释处补一句"相比逆动力学前馈，本文方法在转向工况下的残差从 133.6 降到 5.86 rad/s²"。
4. **数字写全，单位写全**。不要写"大概降低了 30 倍"，要写"从 9.31° 降到 0.30°"。
5. **术语固定**。同一个东西全篇只用一个中文词（见 4.2 的对照表），不要"腿长/腿高/机身高度"混用。
6. **图表的标题和坐标轴标签直接写英文**。因为最终必须全英文，中文标题最后都要重做，
   不如一开始就写英文。正文里引用时写成"如图 4(a) 所示"即可。

## 4.2 中英术语对照表（写中文稿时就按这个统一）

| 中文 | 英文 | 备注 |
|---|---|---|
| 双轮腿机器人 | wheel-legged robot | 首次出现后可简称机器人 |
| 串联腿 | serial-leg | 相对并联/四连杆 parallel-linkage |
| 机身 / 躯干 | torso | 不要用 body（body 在动力学里指刚体） |
| 大腿 / 小腿 | thigh / shank | 不要用 upper leg / lower leg |
| 髋关节 / 膝关节 | hip joint / knee joint | |
| 腿高 | leg height | 定义为机身原点到轮心的高度差，要在文中给出定义 |
| 轮心 | wheel center | |
| 质心 | center of mass (CoM) | |
| 逆运动学 | inverse kinematics | |
| 雅可比 | Jacobian | |
| 关节空间 / 任务空间 | joint space / task space | |
| 虚拟模型控制 | virtual model control (VMC) | |
| 线性二次型调节器 | linear quadratic regulator (LQR) | |
| 前馈 / 反馈 | feedforward / feedback | |
| 支撑前馈 | support feedforward | |
| 推力前馈 | thrust feedforward | |
| 重力补偿 | gravity compensation | |
| 相位 / 状态机 | phase / state machine | |
| 相位独占调度 | phase-exclusive scheduling | |
| 下蹲 | crouch | |
| 蹬伸 | extension | |
| 腾空 / 飞行段 | flight | 飞行姿态写 aerial attitude |
| 落地 | landing | 落地吸收写 landing impact absorption |
| 收腿 | leg tuck（或 leg retraction） | 全篇统一用 tuck |
| 越障 | obstacle crossing | |
| 单腿变高度越障 | asymmetric leg-length obstacle crossing | 不要直译成 single-leg height change |
| 波浪路 | undulating terrain（或 washboard road） | 统一用一个 |
| 梯形坡 / 台阶 | trapezoidal ramp / step | |
| 俯仰 / 侧倾 / 偏航 | pitch / roll / yaw | |
| 稳态误差 | steady-state error | |
| 峰峰值 | peak-to-peak value | |
| 离地占比 | airborne fraction | |
| 行程保持率 | travel retention | |
| 质心弹道 | ballistic rise of the CoM | |
| 阻尼比 | damping ratio | |
| 限幅 / 饱和 | saturation（或 limit） | 电机到上限写 motor saturation |
| 标定 | calibration | |
| 打滑 | slip | |
| 鲁棒性 | robustness | |
| 延迟预算 | delay budget | |
| 控制周期 | control period | |
| 迁移可行性 | implementation feasibility | |

## 4.3 翻译阶段怎么做

1. **不要逐句直译**。先把一段中文读懂，用英文重新说一遍，再对照中文检查信息有没有丢。
2. **把"通过…实现了…"改成主谓宾**。例：
   - 中文：通过引入解析式支撑前馈，实现了越障时侧倾的大幅降低。
   - 直译（差）：Through introducing the analytic support feedforward, the large reduction of roll during obstacle crossing was realized.
   - 地道（好）：The analytic support feedforward reduces the peak roll angle by a factor of 30 during obstacle crossing.
3. **方法用被动、贡献和结论用主动**（见 3.1 节）。
4. **数字与单位检查一遍**：单位前留空格、度数直接用 `°`、量纲一致。
5. **术语回查 4.2 表**，确保全篇一致。
6. 译完后**朗读一遍**：读起来别扭的句子，审稿人也会觉得别扭。

## 4.4 现在就并行做的三件事（别拖到最后）

1. **收文献**。这是唯一无法在最后一天赶出来的东西：至少 5 条、作者来自 3 个以上国家、
   每条带 DOI、正文全部要引用到。建议用 Zotero 建一个库，边写边插引用。
2. **先做符号表**。把第 3.3 节的符号表抄到论文模板里，写作时保持同一套符号，
   否则写到第 IV 章会发现公式里的符号和前面不一致。
3. **把 Figure 4/5/6 的数据导出**（越障、跳跃、行驶跳），因为写结果章时会反复引用它们，
   数据没定下来就会反复改文字。
