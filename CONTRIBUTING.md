# 贡献指南

## Commit Message 规范

所有提交必须遵循统一格式，便于追踪变更历史。

### 格式

```
[模块] 简短描述（中文）

类型: feat | fix | docs | refactor | test | chore
```

### 模块前缀

| 前缀 | 含义 | 示例 |
|------|------|------|
| `[sim]` | 仿真环境、场景搭建 | `[sim] 导入校园建筑群三维模型` |
| `[algo]` | 路径规划与避障算法 | `[algo] 实现A*算法三维栅格搜索` |
| `[ctrl]` | 飞行控制 | `[ctrl] 实现PID姿态控制器` |
| `[web]` | Web可视化界面 | `[web] 添加Three.js飞行轨迹渲染` |
| `[docs]` | 文档、论文笔记、报告 | `[docs] 添加论文1方法摘要` |
| `[data]` | 实验数据、仿真结果 | `[data] 添加场景A避障测试数据` |
| `[infra]` | 仓库配置、脚本 | `[infra] 添加.gitignore规则` |

### 类型说明

| 类型 | 含义 |
|------|------|
| `feat` | 新功能、新算法 |
| `fix` | 修复bug、纠正错误 |
| `docs` | 文档变更（README、注释、论文笔记） |
| `refactor` | 重构代码（不改变功能） |
| `test` | 添加或修改测试 |
| `chore` | 构建、依赖、配置等杂项 |

### 示例

```bash
git commit -m "[algo] 实现A*算法三维搜索

类型: feat"
```

```bash
git commit -m "[web] 修复飞行轨迹渲染卡顿

类型: fix"
```

```bash
git commit -m "[docs] 添加开题报告初稿

类型: docs"
```

### 约定

- 标题不超过50字
- 中文描述，简洁清晰
- 一次提交只做一件事
- 大改动先开Issue讨论

## 分支规范

| 分支类型 | 命名格式 | 示例 |
|----------|---------|------|
| 功能分支 | `feat/功能名` | `feat/a-star-algorithm` |
| 修复分支 | `fix/问题名` | `fix/collision-detection` |
| 文档分支 | `docs/内容` | `docs/paper-notes` |

## Pull Request 流程

1. 从 `main` 创建功能分支
2. 在分支上开发并提交
3. 推送到 GitHub
4. 发起 Pull Request
5. 至少1人Review后合并
