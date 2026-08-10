# SA、GA、SARSA、DQN、PPO 自动优化与评价执行指南

## 1. 优化目标与选择原则

当前 standalone 训练几何版本为 `rl-scheduling-v2-factory-astar`：

- DQN/SARSA 机器人位置只从 `PARKING_SPOTS` 和 `REST_NODES` 采样；
- 任务端点只使用 Webots 一致的 `WS1～WS6`、`S1～S8`；
- robot→pickup 与 pickup→delivery 均使用 0.25 m 占用栅格 Grid A* 距离；
- 不可达 robot-task pair 返回无穷代价并由 action mask 删除；
- DQN/SARSA 使用 3/5 → 5/8 → 8/12 的课程难度；
- PPO 和端到端 standalone 使用 `PARKING_SPOTS`，优先协同 Grid A*，失败才回退拓扑 A*；
- SA/GA 超参数搜索使用同一套真实工厂场景与 Grid A* 代价。

旧版 `rl-scheduling-v1` checkpoint 会被拒绝加载，必须重新训练，避免把“随机矩形位置+欧氏距离”模型误用于新版评价。

最终模型不按训练 reward 单独排名，而按以下词典序选择：

1. 碰撞与最小间距违规必须为 0；
2. 最大化任务完成率和吞吐量；
3. 最小化 P95 等待时间、平均完成时间和空驶距离；
4. 最小化机器人任务数 CV（负载不均衡）；
5. 调度 P95 延迟满足实时预算。

建议综合分数仅用于安全约束全部满足后的次级排序：

```text
score = 0.35 * completion_rate
      + 0.25 * normalized_throughput
      - 0.15 * normalized_p95_wait
      - 0.10 * normalized_empty_distance
      - 0.10 * workload_cv
      - 0.05 * normalized_scheduler_p95
```

## 2. Reward 与 penalty 比例

DQN/SARSA 使用如下基准：

| 项目 | 权重 | 目的 |
|---|---:|---|
| 任务完成 | +10.0 | 主目标 |
| 合法分配 | +0.5 | 提供稠密反馈，但不压过完成奖励 |
| 任务优先级 | +0.5 | 处理紧急任务 |
| 有界任务老化 | +0.5 | 防止饥饿，最多按 2 个单位计算 |
| 距离 | -0.08 | 降低空驶 |
| 归一化等待 | -0.03 | 减少平均等待，修复旧版正向等待奖励 |
| 非法动作 | -5.0 | 强化 action mask |
| 无意义 NO_OP | -2.0 | 有工作时避免空等 |
| 实际碰撞 | -100.0 | 不允许用吞吐交换安全 |

比例设计为：一次任务完成约等于 20 次合法分配奖励，碰撞惩罚约等于 10 次任务完成奖励。等待与距离必须先归一化，避免量纲随仿真时长增长。

PPO 当前使用独立运输近似环境，推荐比例为：完成 `+10`、每米空驶 `-0.1`、拥堵事件 `-0.5`、每空闲机器人每步 `-0.01`、均衡奖励 `+1.0`。正式论文比较时应明确 PPO 与 DQN/SARSA 的训练环境尚未完全统一。

## 3. 推荐参数

| 算法 | 推荐配置 |
|---|---|
| SA | `T0=10`, `cooling=0.985`, `Tmin=0.01`, `iterations=2000`, `5 ms` |
| GA | `population=48`, `generations=50`, `10 ms`；下一阶段加入 OX 交叉、锦标赛选择和 0.15→0.35 自适应变异 |
| SARSA | `alpha=0.05`, `gamma=0.98`, `epsilon=1.0→0.02`, decay `0.999`, 10,000 episodes |
| DQN | 2×256 MLP，`lr=1e-4`, `gamma=0.99`, batch 128, replay 100k, warmup 5k, target 500, epsilon `1.0→0.02`, 5,000 episodes |
| PPO | 2×256，`lr=3e-4`, `gamma=0.99`, GAE `0.95`, clip `0.2`, entropy `0.02`, batch 256, rollout 2048, epoch 8, 5,000 episodes |

训练采用 A→B→C curriculum。验证种子只选 checkpoint，测试种子只做最终报告。Webots 最终评价使用 A/B/C × 5 seeds；调参阶段不要使用测试种子。

自动搜索范围如下：

| 算法 | 搜索参数 |
|---|---|
| SA | 初温、cooling rate、最大迭代数、实时预算 |
| GA | population、generations、实时预算 |
| SARSA | alpha、gamma、epsilon decay、3 套 reward profile |
| DQN | learning rate、hidden size、batch、3 套 reward profile |
| PPO | learning rate、GAE lambda、clip、entropy、update epochs |

SARSA/DQN 的跨 reward-profile 晋级不直接比较 raw reward，因为不同权重会改变数值尺度。实际采用：

```text
validation_score = 100 × mean_completed
                 - 100 × invalid_actions
                 + 0.01 × mean_reward
```

因此任务完成数占主导，非法动作受到硬惩罚，reward 只用于接近配置之间的破同分。

## 4. 一键执行

一键脚本现在先执行自动超参数搜索：

```text
Stage 1：低预算训练全部候选
    ↓ validation 排名，保留前 1/3（至少 2 组）
Stage 2：提高预算继续训练晋级候选
    ↓ validation 再排名
Champion：使用冠军参数完成正式训练
    ↓ 冻结模型
Test/Webots：A/B/C × 独立随机种子最终评价
```

搜索报告保存在 `results/hyperparameter_search/optimized_v2/search_report.json`，冠军模型保存在其 `best/sarsa`、`best/dqn` 和 `best/ppo` 子目录。SA/GA 的冠军参数也会由一键脚本自动读取并注入 Webots 实验。

在项目根目录 PowerShell 中运行快速验证：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_optimized_training_evaluation.ps1 -Profile Quick -StandaloneOnly
```

正式训练并调用 Webots 评价：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_optimized_training_evaluation.ps1 -Profile Full
```

指定 Webots：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_optimized_training_evaluation.ps1 -Profile Full -Webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
```

已有模型、只重新评价：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_optimized_training_evaluation.ps1 -Profile Full -SkipTraining
```

## 5. 分步命令

```powershell
python .\scripts\train_scheduler.py --algorithm sarsa --episodes 10000 --seed 42 --checkpoint-dir .\results\models\optimized_v2\sarsa --evaluation-interval 100 --lr 0.05 --gamma 0.98 --epsilon-end 0.02 --epsilon-decay 0.999

python .\scripts\train_scheduler.py --algorithm dqn --episodes 5000 --seed 42 --checkpoint-dir .\results\models\optimized_v2\dqn --evaluation-interval 50 --lr 0.0001 --gamma 0.99 --hidden-size 256 --batch-size 128 --replay-capacity 100000 --warmup-steps 5000 --target-update 500 --epsilon-end 0.02 --epsilon-decay-steps 50000

python .\scripts\train_ppo.py --episodes 5000 --save-dir .\results\models\optimized_v2\ppo --seed 42 --lr 0.0003 --gamma 0.99 --gae-lambda 0.95 --clip 0.2 --entropy 0.02 --value-coeff 0.5 --epochs 8 --batch-size 256 --buffer-size 2048 --hidden-size 256
```

资源预估：Full 模式包含 15 次 SA/GA Webots 运行和 15 次 RL Webots 运行，单次仿真 1800 秒；即使 fast 模式也可能需要数小时。建议先执行 Quick，确认环境和模型加载无误后再运行 Full。
