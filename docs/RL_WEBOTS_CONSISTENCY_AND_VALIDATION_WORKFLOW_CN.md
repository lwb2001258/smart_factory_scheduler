# RL–Webots 一致性与可信评估 Workflow

## 1. 目标与当前基线

本 workflow 用于确保 SARSA、Double-DQN 和 pairwise PPO 在训练、standalone
仿真和 Webots 中使用同一份状态、动作、约束与评价语义，并确保实验名称与
真正做出决策的算法一致。

项目已通过 Double-DQN 使用广义的 Q-Learning 方法，但没有实现可独立选择的
经典表格 Q-Learning 调度器。SARSA 虽然保存 Q-table，但其
TD 目标使用当前策略选择的下一动作 `Q(s', a')`，属于 on-policy SARSA；经典
Q-Learning 使用 `max_a Q(s', a)`，两者不得混称。Double-DQN 也不得标记为
表格 Q-Learning。

当前正式支持范围：

| 算法 | 状态 | 正式动作合同 |
|---|---|---|
| SARSA(0) | 已实现 | 机器人–任务对 + NO_OP |
| Double-DQN | 已实现 | 机器人–任务对 + NO_OP |
| PPO | 已实现 | 机器人–任务对 + NO_OP |
| 旧 PPO | 仅兼容复现 | 8 动作、只选择机器人，不进入新评价 |
| Q-Learning 方法族 | 通过 Double-DQN 使用 | 不存在独立调度器名称 |
| 经典表格 Q-Learning | 未独立实现 | 无 |

## 2. 全局完成定义

只有以下条件全部满足，才可以声称某个 RL 模型完成了可信 Webots 评价：

1. checkpoint 算法、环境版本、状态维度和动作维度通过严格校验；
2. checkpoint SHA-256、Git commit、seed、运行模式和 Webots 版本被保存；
3. 训练与部署对同一 snapshot 产生相同 observation 和 action mask；
4. 任务流、初始位置和初始电量可由 seed 确定性重放；
5. 结果记录实际 policy decision、policy commit 和 fallback 数；
6. 启动回退或运行期回退超过门限的结果不得进入 RL 排名；
7. 抽象、standalone 和 Webots 三级评价均通过；
8. 全量测试、两轮 code review 和步骤验收报告全部完成。

## 3. 每一步的强制质量门禁

每个步骤必须依次执行：

1. 实现及局部静态校验；
2. 第一轮 code review：功能正确性、接口、错误处理和边界条件；
3. 修正第一轮问题；
4. 单元测试、合同测试及固定 seed 测试；
5. 第二轮 code review：回归风险、实验污染、可重复性、安全和性能；
6. 修正第二轮问题；
7. 专项测试和全量回归；
8. 写出验收证据，未通过不得进入下一步。

两轮 review 必须使用不同检查清单，不能把同一次检查重复记为两轮。

## 4. 分步实施

### 步骤 1：算法与模型注册表、真实性基础门禁

- 建立权威算法注册表，明确 Q-Learning 方法族由 Double-DQN 覆盖，同时没有
  独立的表格 Q-Learning 调度器；
- 校验算法身份、环境版本、状态/动作维度和 legacy 状态；
- 生成 checkpoint SHA-256；
- 新实验默认拒绝旧 126×8 PPO；
- 提供可被 PowerShell/CI 调用的 JSON 审计命令。

验收：三种 pairwise 模型通过，旧 PPO 默认失败、显式复现模式才能通过，
算法错配和环境版本错配必须失败。

### 步骤 2：统一环境合同

- 将 observation、slot 顺序、action codec、mask、路径成本和电池派单条件
  固化成一个版本化合同；
- Webots adapter 和所有训练器只调用该合同；
- checkpoint 保存归一化和合同元数据。

验收：至少 100 个固定 snapshot 的 observation、mask、可行 assignment
逐元素一致。

### 步骤 3：调度相关的 Webots 行为代理

- 加入转向/加减速时间、预约等待、动态绕行、路径拒绝、连续充电、任务中止、
  重规划、死锁和近失事件；
- 参数由 Webots 轨迹标定，不直接拟合最终算法排名。

验收：保留 seed 上的旅行时间、电量变化和事件率误差满足预先声明阈值。

实现状态：已建立 `webots-dispatch-proxy-v1`，其基础速度、角速度、到达容差、
仿真 tick、耗电和充电率直接取自 Webots 控制器/共享配置。当前尚未把历史聚合
结果反向拟合为校准参数，因为旧结果没有逐段路径、等待和转向标签；用聚合任务
完成时间拟合会把拥堵等待错误归入运动速度。后续评价会先记录可分解 route trace，
再校准 `turn_time_weight` 和动态等待模型。

### 步骤 4：统一场景与随机性

- A/B/C 使用相同机器人数量、停车位、任务流、优先级和初始电量；
- 拆分训练、验证、最终测试 seed，禁止选择集泄漏；
- 保存可重放场景清单。

验收：三种运行模式的输入事件文件哈希一致。

实现入口：`experiment_manifest.py` 和
`scripts/generate_experiment_manifest.py` 生成版本化、带 SHA-256 指纹的 A/B/C
输入，包括机器人数量、停车位、初始电量以及按 16 ms tick 量化的完整任务流。

### 步骤 5：统一奖励与事件账本

- 统一 assignment、commit/reject、pickup、delivery、charge、wait、replan、
  deadlock、near-miss、collision 和 fallback 事件；
- 在线训练奖励与 Webots 离线重放奖励共用一个纯函数。

验收：固定事件轨迹的在线/离线累计奖励完全一致。

### 步骤 6：训练–部署差分测试

- 比较同一 snapshot 的 observation、mask、action 和 assignment；
- 比较静态场景完成顺序；
- 对动态场景使用事件和统计容差，不宣称刚体轨迹逐点相等。

验收：合同字段零差异，代理行为指标在标定阈值内。

### 步骤 7：重新训练

- 只训练 SARSA、Double-DQN 和 278×161 pairwise PPO；
- 旧 PPO 只用于标明为 legacy 的复现实验；
- checkpoint 晋级只使用 validation seeds。

验收：训练可恢复、同 seed 可重复、模型元数据完整、held-out 指标通过。

### 步骤 8：三级评价

依次运行抽象环境、standalone shadow evaluation 和 Webots A/B/C 多 seed
评价。前一级失败则不启动下一级。

验收：生成带置信区间的结果，并单独报告 domain gap。

### 步骤 9：结果真实性与发布门禁

- 保存 checkpoint hash、代码版本、环境版本、seed、Webots 版本；
- 保存 policy/fallback/invalid/commit 计数；
- fallback 超限或缺少 provenance 的结果自动标记无效；
- 不允许把 fallback 的 Hungarian 成绩归入 RL。

验收：机器可读报告能够确定每个已完成任务实际由谁选择。

## 5. 每步验收记录模板

```text
步骤：
改动范围：
静态校验：
Code Review 1 结论与修正：
专项测试：
Code Review 2 结论与修正：
全量回归：
遗留风险：
验收状态：PASS / FAIL
```

任何 `FAIL`、未解释的 fallback、陈旧结果复用或测试缺失都必须停止流程。
