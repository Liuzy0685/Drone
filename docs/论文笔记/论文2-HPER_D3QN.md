# 论文2: 基于深度强化学习的联合作战空域自适应避障方法

- **标题**: Deep Reinforcement Learning-Based Adaptive Collision Avoidance Method for UAV in Joint Operational Airspace
- **作者**: Yan Shen, Xuejun Zhang, Yan Li, Weidong Zhang
- **期刊**: Defence Technology, February 2026
- **DOI**: (KeAi / ScienceDirect)

---

## 核心方法

### 1. HPER-D3QN 框架

- **HPER**: 分级优先经验回放 (Hierarchical Prioritized Experience Replay)
  - 根据碰撞风险等级分配采样权重
  - 高风险样本被更频繁回放训练
- **D3QN**: Dueling Double Deep Q-Network
  - Dueling架构分离状态价值与动作优势
  - Double Q-learning解决高估偏误

### 2. 动态威胁评估

- 基于相对距离、速度、航向的实时威胁建模
- 自适应碰撞风险等级划分（高/中/低）

### 3. 消融实验结论

- HPER 机制对模型性能贡献最大
- 去除HPER后成功率显著下降

---

## 仿真结果

- 避障成功率: 99.95%
- 相比 PPO 和 DQN 基线，收敛速度更快、成功率更高

---

## 对我们的启发

1. 深度强化学习适合处理**环境不确定性**高的场景
2. 经验回放的优先级设计（分险分级）是关键技巧
3. 如果在A\*+APF基础上加入学习能力，可参考HPER机制
4. 威胁建模的"距离-速度-航向"三维度可融入人工势场法的斥力场设计
