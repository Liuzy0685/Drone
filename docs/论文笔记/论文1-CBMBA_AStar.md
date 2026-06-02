# 论文1: 低空城市智能交通系统多无人机的协作任务分配与路径规划

- **标题**: Cooperative Task Allocation and Path Planning for Multi-UAVs in Low-Altitude Urban Intelligent Transportation Systems
- **作者**: Z. Zhang, J. Jiang, K. V. Ling, X. Wang, W.-A. Zhang
- **机构**: 浙江工业大学、南京航空航天大学
- **期刊**: IEEE Transactions on Intelligent Transportation Systems, Vol. 27, No. 4, April 2026
- **DOI**: 10.1109/TITS.2026.3667967

---

## 核心方法

### 1. 进化势能博弈 (Evolutionary Potential Game)

将多无人机任务分配与路径规划建模为势能博弈：
- 每个无人机是一个博弈参与者
- 策略 = 任务选择 + 飞行路径
- 势能函数 = 总奖励（任务完成收益 - 飞行代价 - 碰撞风险）
- 证明该势能函数存在纳什均衡

### 2. 改进对数线性学习算法 (ILLA)

- 保证以概率1收敛到最优纳什均衡
- 推导了合适的玻尔兹曼参数
- 分布式决策，无需中央协调器

### 3. 约束多层双向自适应 A* (CBMBA A\*)

- 多层搜索：在不同高度层之间切换
- 双向搜索：从起点和目标点同时搜索
- 自适应权重：根据环境密度动态调整启发式

---

## 仿真结果

| 指标 | 相比基线方法 |
|------|------------|
| 任务奖励 | ↑ 11.67% |
| 任务执行时间 | ↓ 37.41% |
| 运行时间 | ↓ 61.02% |

---

## 对我们的启发

1. CBMBA A\* 的多层双向搜索思想可直接借鉴到建筑群避障
2. 博弈论方法适合未来扩展为多无人机协同
3. 自适应启发式权重对复杂环境的效率提升显著
