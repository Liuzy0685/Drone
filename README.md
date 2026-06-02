# 复杂建筑群环境下无人机自主避障与三维仿真系统

> 大学生创新创业训练计划项目

## 项目简介

本项目针对复杂建筑群环境下的无人机自主飞行问题，研究三维场景建模与还原、无人机自主避障算法设计与优化，实现复杂环境下无人机飞行路径规划、实时避障与三维飞行仿真，开发仿真系统验证算法可行性与飞行安全性。

## 技术栈

| 类别 | 技术 |
|------|------|
| 仿真平台 | AirSim / Webots |
| 核心算法 | C++（A\*、人工势场法、RRT\*） |
| 仿真控制 | Python（AirSim API） |
| Web可视化 | Vue 3 + Three.js + FastAPI |
| 版本管理 | Git + GitHub |

## 目录结构

```
├── README.md                   # 项目说明
├── CONTRIBUTING.md             # 贡献指南（提交规范）
├── LICENSE                     # MIT 许可证
├── .gitignore                  # Git 忽略规则
│
├── docs/                       # 文档
│   ├── 论文笔记/                # 参考文献阅读笔记
│   ├── 技术方案/                # 技术选型与方案文档
│   └── 开题报告/                # 开题相关材料
│
├── simulation/                 # 仿真相关
│   ├── airsim/                 # AirSim 配置和场景
│   │   └── settings.json       # 无人机参数配置
│   └── environments/           # 自定义场景模型
│
├── algorithm/                  # 算法实现
│   ├── path_planning/          # 全局路径规划（A*、RRT*）
│   ├── obstacle_avoidance/     # 局部避障（人工势场法）
│   └── cooperative/            # 多机协同（博弈论）
│
├── control/                    # 飞行控制
├── web_display/                # Web可视化界面
│   ├── frontend/               # Vue 3 + Three.js
│   └── backend/                # FastAPI 数据推送
│
├── data/                       # 实验数据
│   ├── flight_logs/            # 飞行日志
│   └── results/                # 仿真结果
│
└── scripts/                    # 工具脚本
```

## 快速开始

> 待补充：环境搭建步骤

```bash
# 克隆仓库
git clone https://github.com/Liuzy0685/Drone.git
cd Drone
```

## 团队成员

| 姓名 | 职责 |
|------|------|
| 刘子逸（组长） | 项目管理、算法设计 |
| （待补充） | （待补充） |
| （待补充） | （待补充） |

## 参考文献

1. Zhang Z, et al. "Cooperative Task Allocation and Path Planning for Multi-UAVs in Low-Altitude Urban Intelligent Transportation Systems." *IEEE TITS*, 2026.
2. Shen Y, et al. "Deep Reinforcement Learning-Based Adaptive Collision Avoidance Method for UAV in Joint Operational Airspace." *Defence Technology*, 2026.

## 许可证

MIT License
