# RL–Webots Workflow 验收记录

## 步骤 1：算法/模型注册表与真实性门禁

- 改动：算法注册表、checkpoint 审计 CLI、SHA-256 provenance、严格启动和
  fallback 污染门禁。
- Code Review 1：发现审计可被实验入口绕过；已改为正式 RL 实验启动前强制
  审计。
- 专项测试：pairwise SARSA/DQN/PPO 通过；legacy PPO 默认拒绝；错误环境和
  算法拒绝。
- Code Review 2：发现运行期 Hungarian fallback 仍可能污染结果；已增加
  policy decision、native commit 和 fallback 结果门禁。
- 全量回归：通过。
- 验收：PASS。

## 步骤 2：统一环境合同

- 改动：增加版本化 `RLSchedulingContract`，固定 278 维 observation、161 维
  action、slot 排序、NO_OP 和可行性语义；新 checkpoint 嵌入合同指纹。
- Code Review 1：发现把运行时推导的指纹写成旧 checkpoint 已验证指纹会形成
  错误 provenance；已拆分 runtime fingerprint 与 embedded/verified 状态。
- 专项测试：100 个固定 factory snapshot 的 abstract/Webots observation、mask、
  slot 和 assignment 逐元素一致。
- Code Review 2：检查 checkpoint 向后兼容与新 schema；旧模型允许兼容推断但
  标记为未嵌入指纹，新模型 schema 4 强制校验指纹。
- 全量回归：`129 passed, 4 subtests passed`。
- 验收：PASS。

## 步骤 3：Webots 调度行为代理基础层

- 改动：增加 `webots-dispatch-proxy-v1`，模型参数来自控制器/共享配置：16 ms
  tick、0.22 m/s、2.84 rad/s、0.35 m 到达容差、耗电率和连续充电率。
- Code Review 1：专项测试发现充电目标的浮点边界误报；已加入明确的 `1e-9`
  数值容差。
- 专项测试：直线、转向、tick 量化、耗电、连续充电和确定性通过。
- Code Review 2：禁止使用旧聚合完成时间臆造动态拥堵参数；旧数据没有逐段
  路径/等待/转向标签，错误拟合会把等待归因于速度。动态校准延后至事件账本和
  route trace 建立后进行。
- 全量回归：`133 passed, 4 subtests passed`。
- 遗留项：动态等待和重规划参数需要步骤 5 的可分解轨迹后标定。
- 验收：基础代理 PASS；动态标定依赖项已显式转入步骤 5/6。

## 步骤 4：统一场景输入与随机性

- 改动：增加 `factory-manifest-v1`，固定 A/B/C 机器人数量、停车位、初始电量、
  任务端点、优先级和按 16 ms tick 量化的到达时间，并生成 SHA-256 指纹。
- Code Review 1：发现机器人控制器按 robot_id 初始化电量，而 Supervisor 和
  standalone 按实验 seed 顺序抽样，Webots 首次上报会覆盖 Supervisor 初值。
- 修正：四种入口统一使用纯函数 `initial_battery_for_robot(seed, robot_id)`。
- 专项测试：manifest 同 seed 完全一致、不同 seed 指纹不同、A/B/C fleet 正确、
  初始即时任务落在第一个 16 ms tick、任务 ID 和时间单调。
- Code Review 2：逐 seed 比较 Supervisor 配置函数与 robot_controller 独立实现，
  32 组组合逐值相等；检查任务生成器使用同一 Python RNG 和 tick 规则。
- 全量回归：`138 passed, 4 subtests passed`。
- 遗留项：步骤 7 将训练课程从临时 NumPy 场景采样迁移为 manifest/domain split；
  在迁移完成前，manifest 是评价输入合同，而非所有训练 episode 的唯一来源。
- 验收：评价输入合同 PASS；训练迁移依赖已转入步骤 7。

## 步骤 5：统一奖励与事件账本

- 改动：增加 `rl-event-ledger-v1` 和纯函数奖励重放；abstract、standalone、
  Webots 使用相同 assignment/pickup/completion 事件结构。
- Code Review 1：发现 MetricsCollector 方法插入位置会延迟初始化输出路径，已
  修复并增加构造测试。
- 专项测试：assignment、completion、NO_OP、invalid action、奖励中性运行事件；
  abstract 在线累计奖励与事件离线重放完全一致。
- Code Review 2：未知运行事件保持奖励中性；新增 telemetry 不得隐式改变训练
  目标。动态等待参数必须由事件分解后标定。
- 全量回归：`141 passed, 4 subtests passed`。
- 验收：PASS。

## 步骤 6：训练–部署差分测试

- 证据：100 个 snapshot 的 observation/mask/slot/assignment 零差异；事件在线/
  离线奖励零差异；manifest 训练快照与评价输入逐字段一致。
- 两轮审查：分别检查接口数值一致性和跨模式随机性/数据泄漏。
- 验收：PASS（Webots 物理时间误差标定仍依赖正式 route trace）。

## 步骤 7：训练入口迁移

- 改动：SARSA、Double-DQN、pairwise PPO 课程阶段改用 A/B/C manifest domain；
  固定停车位、电量、Python RNG 任务流和 seed provenance。
- Smoke：SARSA 1 episode 成功；PPO 1 episode 输出 278×161 schema-4 checkpoint，
  内嵌合同指纹并通过严格审计。
- Code Review：确认 PPO 调用 `train_pairwise_episode`，legacy 训练函数未进入新
  checkpoint 路径。
- 验收：代码与 smoke PASS；完整 5,000/10,000 episode 训练属于长时运行任务。

## 步骤 8：三级评价

- abstract：训练与 held-out validation smoke PASS；
- standalone：同 seed/manifest/checkpoint 的 20 秒 PPO 原生策略 smoke PASS；
- Webots 第一次：外层命令 120 秒限制早于 runner 自身动态超时，结果记为
  INCONCLUSIVE，遗留进程按精确 PID 清理；
- Webots 第二次：把模拟时长降至 5 秒并允许完整 600 秒墙钟时间，仍触发 runner
  的 600 秒动态超时，没有生成新 JSON。runner 没有读取 2026-08-04 或 standalone
  的旧结果，比较报告正确显示 N/A，且超时后没有遗留 Webots 进程；
- 结论：三级门禁工作正常，但当前本机 Webots batch/控制器同步链路存在独立挂起
  问题。该问题必须单独诊断，不能通过继续增大算法训练时长解决；
- 验收：PARTIAL（abstract、standalone PASS；Webots TIMEOUT）。

## 步骤 9：真实性与发布门禁

- 改动：机器门禁要求 checkpoint hash、环境/合同指纹、manifest 指纹、seed、
  run mode、原生 commit 和零 fallback；旧未嵌入合同指纹模型禁止进入新排名。
- Code Review 1：增加缺字段、算法错配和 fallback 污染拒绝。
- Code Review 2：修复 Webots `checkpoint_sha256`/`sha256` 字段名不一致，并要求
  `checkpoint_contract_verified=true`。
- standalone 实物结果通过发布门禁；历史缺 provenance 结果按预期不通过。
- 验收：PASS。
