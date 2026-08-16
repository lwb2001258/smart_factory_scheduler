# 优先级让行、等待清障与安全恢复 Workflow

> 状态：待实施
> 目标：在现有联合冲突处理基础上，增加“显式优先级让行 -> 转向避让 -> 等待高优先级通过 -> 从当前站位安全重规划”的确定性闭环。
> Webots 路径：`C:\Program Files\Webots\msys64\mingw64\bin\webots.exe`

## 1. 结论

方案可行，但不是替代当前系统，而是对当前实现的分层增强。

当前代码已经具备：

- `controllers/factory_supervisor/factory_supervisor.py:2102` 的 10 s 轨迹预测与冲突检测。
- `controllers/factory_supervisor/factory_supervisor.py:3322` 的 90° 对向让行。
- `controllers/factory_supervisor/factory_supervisor.py:3259` 的全向 standoff escape。
- `controllers/factory_supervisor/factory_supervisor.py:3746` 的 escape 后恢复原业务目标。

当前实现的主要缺口：

1. 没有显式“等待高优先级机器人真正通过”的恢复门控。
2. 优先级仍以机器人 ID、目标距离等启发式为主，缺少统一可解释的优先级比较器。
3. escape 后立即请求 fresh joint plan，可能在高优先级尚未通过时过早恢复。
4. 没有把“从当前让行站位恢复，而不是回到原冲突点”固化为明确协议。
5. 恢复前缺少对全部活跃机器人的预测冲突复验和失败回滚闭环。

因此，本方案优于当前 `_joint_head_on_yield` 的点在于补上确定性的 **yield -> wait-for-clear -> revalidate -> resume**，可显著减少让行后再次进入同一冲突簇的概率。它不替换 `plan_grid_lifelong()`、联合规划事务和最后安全兜底。

## 2. 最终验收标准

最终 Webots 运行必须满足：

- 场景 C、8 台机器人、FCFS、seed 42。
- 仿真时长 `SMART_FACTORY_SIM_DURATION=1800`，即 Webots 仿真 30 分钟。
- 正常达到 1800 s 并保存 `experiment_C_FCFS_*.json`。
- `summary_metrics.pair_distance_violations == 0`。
- `safety_events` 中 `event_type == "collision"` 的数量为 0；Phase 1 将 `safety_event_count` 输出到 `summary_metrics`。
- 控制器无异常退出，世界无加载错误。
- 非物理传送恢复为 0；`SMART_FACTORY_ENABLE_NONPHYSICAL_RECOVERY=0`；Phase 1 新增 `nonphysical_recoveries` 指标。
- 每个机器人至少完成 1 个任务。
- 任务执行期间不允许出现持续超过 10 s 的非计划静止。
- 新指标完整记录：
  - `yield_start`
  - `yield_standoff_selected`
  - `yield_wait_start`
  - `yield_wait_cleared`
  - `yield_wait_timeout`
  - `yield_resume_proposed`
  - `yield_resume_rejected`
  - `yield_resume_committed`
  - `yield_resume_rollback`
- 对比基线：开启新协议后的吞吐不能比关闭新协议的当前默认方案下降超过 10%；若吞吐下降更多，需记录原因并证明安全指标或低进展事件显著改善。
- 统计稳健性：单次 1800 s / seed 42 是最终硬性 certification；吞吐对比应额外跑至少 3 个 seeds（建议 42、123、456）的开关对照组。

## 3. 协议设计

### 3.1 冲突预测

- 沿用 `_predict_trajectory()`，最终目标为滚动预测 10 s，采样 0.5 s。
- 注意：当前 joint 模式下 `_joint_predictive_speed_shield()` 是 6 s / 0.25 s；`ENABLE_JOINT_RUNTIME=1` 时 legacy `_proactive_path_conflict_scan()` 不会运行。Phase 1 必须新增/复用专用 joint-mode 10 s predictor。
- 预测前保留 hold、dispatch delay、controller pause 等合法等待。
- 冲突对先构成冲突图；同一连通分量一起处理。

### 3.2 显式优先级

新增统一优先级比较器，替代零散的“ID 小者胜”和纯目标距离启发式。

建议初始规则：

1. `RETURNING_TO_CHARGE`、紧急制动/低电量机器人默认具有让行豁免，不允许作为主动 yielder。
2. 正常任务按任务优先级高者保持路径。
3. 优先级相同时，距离目标更近者保持路径。
4. 仍相同时，`robot_id` 较小者保持路径，保证确定性。

该规则必须经过两轮 review，避免高优先级任务造成低优先级机器人饥饿。

### 3.3 让行腿

- 对选中 yielder 在 90° 左右方向生成候选 standoff。
- 候选必须满足：
  - 与所有 peer 当前位点最小净空不小于 0.80 m。
  - `motion_coordinator._segment_clear()` 通过。
  - `plan_grid_lifelong()` 可规划。
  - 不进入货架、工位、充电站禁区。
- 只选择“moving standoff”，不允许在任意走廊中制造新的静止障碍。
- 通过版本化 `_install_runtime_plan()` 下发，记录 `recovery_active=True` 与 `recovery_resume_goal=original_goal`。

### 3.4 等待清障门控

让行机器人到达 standoff 后，不立即恢复。新增 `YIELD_RESUME_MIN_CLEARANCE` 和 `YIELD_RESUME_HORIZON` 状态机：

- 先由现有 _joint_runtime_watchdog() 的 0.5 s tick 驱动；若需 0.25 s 高节奏，必须设置类似 PROACTIVE_SCAN_BUDGET_SECONDS 的算力预算，确保单次检查不阻塞 Webots step。
- 同时满足以下条件才允许恢复：
  1. 预测未来 `YIELD_RESUME_HORIZON` 内两机器人最小距离始终大于 `YIELD_RESUME_MIN_CLEARANCE`。
  2. winner 已越过原冲突路径段，或已与 yielder 保持横向分离。
  3. 对 yielder 的原业务目标规划新路径时，与所有其他活动机器人轨迹无预测冲突。
- 默认建议值：`YIELD_RESUME_MIN_CLEARANCE=0.80`，`YIELD_RESUME_HORIZON=3.0`。
- 超时保护：若等待超过 `YIELD_RESUME_TIMEOUT`，转入 fresh joint plan 或现有 escape 兜底，不无限等待。

### 3.5 从当前站位恢复

- 恢复位置必须使用 yielder 当前实测位置，不是原碰撞点，也不是旧 waypoint。
- 原业务目标通过 `recovery_resume_goal` 保存。
- 在 joint runtime 下，将恢复目标交回 `_refresh_joint_grid_candidate()`，由全队滚动规划生成协调路径。
- 在非 joint/legacy 模式下，才允许单机 `plan_grid_lifelong()` 直接恢复。
- 明确禁止机器人主动驶回原冲突点后再重规划，避免重新制造冲突。

### 3.6 事务提交与回滚

- 让行腿、等待恢复和恢复路径都必须走 `_install_runtime_plan()` 的版本化事务。
- 发送失败时恢复旧 plan、旧 `route_write_owner`、旧 reservation 和旧速度。
- `recovery_active` 状态必须与 `route_write_owner` 生命周期一致。
- 新路径提交后，`joint_speed_until`、`_joint_escape_until`、`_proactive_replan_cooldown_until` 等状态必须重置到确定值。

## 4. 实现步骤与 Code Review

每个实现阶段必须经过两轮 code review。Review 2 必须在 Review 1 的修正提交完成后进行。

### Phase 0：基线冻结与可观测性

- 内容：用当前代码跑 Scene C / FCFS / seed 42 / 600 s 基线，记录吞吐、重规划、低进展、间距违规、让行次数。
- Review 1：确认基线命令、日志和 JSON 字段可复现。
- Review 2：确认指标口径与最终验收一致，补充当前缺失的 yield 指标。

### Phase 1：特性开关与指标

- 内容：新增 `SMART_FACTORY_ENABLE_PRIORITY_YIELD_RESUME`，默认 `0`；增加 `MetricsCollector` 的 yield/resume 事件记录。
- Review 1：开关默认关闭，旧行为不变；事件记录线程安全且不会丢字段。
- Review 2：检查 JSON schema、事件上限裁剪和单位。

### Phase 2：优先级比较器

- 内容：实现统一 `_yield_priority_key()` 与 `_select_yielder()`，替换 `_detect_head_on_pair()` 中的 ID 规则。
- Review 1：优先级规则无歧义、确定性，charging/emergency 豁免正确。
- Review 2：多机器人冲突链中比较器稳定，不产生环或抖动。

### Phase 3：让行腿选择

- 内容：复用 `_joint_head_on_yield()` 的 90° standoff 生成，抽出可复用函数，增加禁止区域校验和 standoff 净空检查。
- Review 1：几何净空、货架/工位禁区、`_segment_clear()` 覆盖。
- Review 2：候选排序、路径规划和版本化下发的失败回滚。

### Phase 4：等待清障状态机

- 内容：实现 wait-for-clear；等待 winner 通过；超时/异常转入 fresh joint plan 或 escape。
- Review 1：预测轨迹复用 `_predict_trajectory()`，对 hold/emergency 等合法静止建模正确。
- Review 2：边界条件：winner 停止、yielder 被第三方阻断、winner 目标变化、消息乱序、超时重试。

### Phase 5：安全恢复与事务回滚

- 内容：从当前站位恢复原业务目标；恢复前全队预测冲突复验；失败回滚旧计划。
- Review 1：确认不会驶回原冲突点，不会绕开 `route_write_owner` 权限。
- Review 2：确认恢复路径的 space-time reservation、速度恢复、`recovery_active` 清理和 metrics 完整。

### Phase 6：联合模式集成与回归测试

- 内容：在 `_joint_try_escape_component()`、`_joint_runtime_watchdog()` 中接入新状态机；添加单元测试与协议测试。
- Review 1：新旧路径互斥，关闭开关时行为完全不变。
- Review 2：测试覆盖多机器人冲突、超时、回滚、乱序消息、紧急制动。

### Phase 7：Webots 180 s 冒烟

- 内容：场景 C / FCFS / seed 42 / 180 s，开启新协议，关闭非物理恢复。
- Review 1：无异常退出、无碰撞、无间距违规。
- Review 2：检查 yield 事件是否按预期发生，非计划静止为 0。

### Phase 8：Webots 600 s 预验收

- 内容：场景 C / FCFS / seed 42 / 600 s。
- Review 1：安全指标为 0，吞吐和低进展与基线对比可接受。
- Review 2：复核所有硬指标，任一项失败则回到对应 Phase 修复，不得跳过。

### Phase 9：Webots 1800 s 最终验收

- 内容：场景 C / FCFS / seed 42 / 1800 s。
- Review 1：自动解析日志与 JSON，逐项对照第 2 节验收标准。
- Review 2：复核 `PASS` 或 `FAIL_CONTINUE` 判定；失败需列出首个失败时间、机器人、任务状态、route owner 和触发源。

## 5. 代码审查强制项

每次 review 必须执行：

```powershell
git diff --check
python -m py_compile controllers/factory_supervisor/factory_supervisor.py
python -m py_compile controllers/factory_supervisor/config.py
python -m py_compile controllers/robot_controller/robot_controller.py
python -m pytest -q
```

审查重点：

- 不允许绕过 `route_write_owner`。
- 不允许引入非物理 teleport 作为正式验收路径。
- 不允许在任务执行中制造无 wait-action 的静止等待。
- 不允许恢复路径回到原冲突点。
- 所有等待、恢复、回滚必须有 metrics。
- 关闭特性开关时，行为必须与基线一致。

## 6. 最终 Webots 1800 s 验收命令

```powershell
$env:SMART_FACTORY_SIM_DURATION='1800'
$env:SMART_FACTORY_AUTO_STOP='1'
$env:SMART_FACTORY_ENABLE_JOINT_RUNTIME='1'
$env:SMART_FACTORY_ENABLE_PRIORITY_YIELD_RESUME='1'
$env:SMART_FACTORY_ENABLE_PROACTIVE_JOINT_SPEED='0'
$env:SMART_FACTORY_ENABLE_RHCR='0'
$env:SMART_FACTORY_ENABLE_NONPHYSICAL_RECOVERY='0'

python scripts/run_experiments.py `
  --scenario C `
  --scheduler FCFS `
  --seeds 1 `
  --seed-values 42 `
  --webots 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe'
```

结果检查：

- `results/experiment_C_FCFS_*.json` 必须为完整运行。
- stdout/stderr 无 controller exception、world load error、supervisor crash。
- `summary_metrics.pair_distance_violations=0`。
- `safety_events` 中无 `collision` 类型事件；`summary_metrics.safety_event_count=0`。
- 新指标 `summary_metrics.nonphysical_recoveries=0`。
## 7. 不建议做的事
- 不要让低优先级机器人回到原冲突点再重规划。
- 不要用固定 sleep 代替预测清障门控。
- 不要在 8 机器人 Scene C 中默认开启实验性 RHCR/CBS 全量批下发。
- 不要把紧急制动、后退和传送恢复算作预规划成功。
- 不要用短时运行或平均值替代单次 1800 s 硬指标验收。
