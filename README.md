# robot_ws — 双轮足机器人（wheeled-legged robot）

一个双轮足机器人的 **MuJoCo 仿真 + 控制** 工程。机器人左右各一条四连杆并联腿，腿端各装一个
驱动轮；控制上用 **LQR + VMC** 组合，能自平衡站立、行进与转向、原地调腿高，也能起跳越障、
跟随波浪路面。附带论文用的可复现场景运行器与出图脚本。

主项目在 `Two_wheeled_legged_robot/robot_sim/`，本仓库其他目录是文档与参考资料。

## 快速开始

```bash
cd Two_wheeled_legged_robot/robot_sim

uv sync --extra figures                                   # 装依赖（Python 3.13）
uv run python test_stand_wheel_balance.py --seconds 3      # 无头冒烟测试
uv run python -m src.launch_mujoco --flat-ground           # 打开可视化仿真（Linux）
uv run mjpython -m src.launch_mujoco --flat-ground         # 打开可视化仿真（macOS）
```

手柄默认启用（左摇杆前后、右摇杆转向、RT/LT 调腿高、A 键跳跃）；没接手柄也能正常跑。

换电脑从零复现、环境排错、完整参数表见 **[`Two_wheeled_legged_robot/复现指南.md`](Two_wheeled_legged_robot/复现指南.md)**。

## 仓库结构

| 路径 | 说明 |
|---|---|
| `Two_wheeled_legged_robot/robot_sim/` | **主项目**：仿真、控制器、论文场景与出图 |
| `Two_wheeled_legged_robot/` | 项目文档：复现指南、迁移指南、结果归档、论文指南、程序讲解 |
| `Two_wheeled_legged_robot/paper_draft/` | 会议论文中文初稿（Markdown）与配图 |
| `wheel_legged_robot_sim-main/` | 上游同名仓库的快照，仅作对照 |
| `TITA_Description-main/` | TITA 双轮足 ROS2 描述包（URDF/xacro），参考用 |
| `assets/`、`control/`、`text/`、`lqr_balance*.py`、`*scene*.xml` | 早期实验残留 |

> `wheeled-leg-robot/` 是**别人的仓库**（`fernandomierhicks/wheeled-leg-robot`），不属于本项目，
> 已在 `.gitignore` 中排除。

## 文档索引

- [`Two_wheeled_legged_robot/复现指南.md`](Two_wheeled_legged_robot/复现指南.md) — 换电脑从零搭建、运行、排错
- [`Two_wheeled_legged_robot/robot_sim/README.md`](Two_wheeled_legged_robot/robot_sim/README.md) — 主项目：算法简介、运行命令、参数
- [`Two_wheeled_legged_robot/robot_sim/程序讲解.md`](Two_wheeled_legged_robot/robot_sim/程序讲解.md) — 代码逐模块讲解
- [`Two_wheeled_legged_robot/robot_sim/paper/README.md`](Two_wheeled_legged_robot/robot_sim/paper/README.md) — 论文场景运行器与出图
- [`Two_wheeled_legged_robot/MIGRATION_GUIDE.md`](Two_wheeled_legged_robot/MIGRATION_GUIDE.md) — 模型/算法迁移记录
- [`Two_wheeled_legged_robot/RESULTS_ARCHIVE.md`](Two_wheeled_legged_robot/RESULTS_ARCHIVE.md) — 历史实验结果归档
- [`Two_wheeled_legged_robot/PAPER_GUIDE.md`](Two_wheeled_legged_robot/PAPER_GUIDE.md) — 论文写作与图表指南

## 复现实验数据

论文用的场景都有名字，一条命令跑一个并落 CSV + JSON：

```bash
cd Two_wheeled_legged_robot/robot_sim
uv run python -m paper.run_cases --list              # 看全部 33 个场景
uv run python -m paper.run_cases --group terrain     # 跑一组
uv run python paper/figures/fig5_terrain.py          # 出图
```

汇总指标 JSON（`paper/data/*.json`、`index.json`）已入库，是论文数字的出处；
逐步遥测 CSV 体积大且可重建，已在 `.gitignore` 中排除。

## 来源与许可

主项目 `Two_wheeled_legged_robot/robot_sim/` 基于 **Apache-2.0** 许可，
衍生自 [Edmounds/wheel_legged_robot_sim](https://github.com/Edmounds/wheel_legged_robot_sim)
（Copyright 2026 Edmounds），原许可证见 `Two_wheeled_legged_robot/robot_sim/LICENSE`，
本仓库在此基础上做了模型、控制器、场景与文档上的修改。

`wheeled-leg-robot/`（`fernandomierhicks/wheeled-leg-robot`）、
`TITA_Description-main/`、`wheel_legged_robot_sim-main/` 是各自的第三方作品，
版权归原作者，仅作参考未在本仓库修改。
