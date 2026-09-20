# 机器人连续执行授权与同时空 Hold 修复 Workflow

状态：`APPROVED / READY_TO_IMPLEMENT`

日期：2026-09-14

上位 workflow：`ROBOT_RUNTIME_EFFICIENCY_RECOVERY_WORKFLOW_CN.md`

## 1. 最高优先级目标

优先消除机器人在没有真实物理 blocker 时的大面积停车。Webots 仿真倍率是必须保持的硬门禁，但不能替代任务吞吐、逐机器人连续运动和 block 指标。

已冻结的失败证据：

- 录像：`results/webots_diagnostic_C_FCFS_20260914_180205.html`；
- 录像运行：180 s，tasks 0/18、有效等待 98.08 s、3 次非阻塞停顿；
- 无录像运行：180 s，tasks 0/18、有效等待 101.44 s、4 次非阻塞停顿、dispatch 179；
- 16 个 planned-wait episode 中，11 个为 `joint_advance_grant`、4 个为 `joint_window_endpoint`，大多约 4 s；
- 同期发生 36 次 soft replan、19 次 hard escape，证明存在“grant 等待 -> stall -> replan/escape -> 新 epoch -> grant 等待”正反馈。

## 2. 不可改变的规则

1. 首次运动仍必须完成 `plan -> validate -> prepare -> arm -> commit -> activate`。
2. 未提交 candidate/pending 路线不得驱动电机。
3. stale pose、实际动态风险、terminal ownership、reverse edge、预留超期继续 fail-closed。
4. 正面冲突优先级、低优先级侧向避让和 terminal egress 优先级不变。
5. 距离违规、teleport/nonphysical recovery 均为 0。
6. 录像运行只诊断机器人效率；性能验收必须关闭录像并使用 fast/no-rendering。

## 3. 目标执行合同

### 3.1 已提交 prefix 即执行授权

- transaction `activation_confirmed` 后，完整 waypoint/time-offset prefix 成为不可伪造的执行授权。
- 控制器不再因为缺少逐 waypoint 正向 `advance_grant` 而停车。
- 机器人只能在 active epoch、path version、reservation deadline 全部匹配时使用该授权。
- prefix 尾部仍执行 successor handoff，不能越过已提交路线。

### 3.2 动态 Hold 是撤销而不是重复授权

- Supervisor 仅在 fresh evidence 表明当前 segment 的实际同时空风险时发送 `advance_hold`。
- hold identity 至少包含 epoch、waypoint index、valid-until、validator evidence 和 peer/resource identity。
- hold 有限期；到期后若没有新的有效 hold，控制器自动继续同一 segment，不重置 epoch/index。
- BRAKE/EMERGENCY 可立即停车；CAUTION 只调速。

### 3.3 同时空验证

冲突必须同时满足空间包络和时间区间重叠。验证覆盖：vertex、opposite edge、turning swept occupancy、terminal residence、fresh-pose uncertainty 和 late-arrival margin。不同时间经过同一 cell 不构成冲突。

## 4. 实施步骤

### Step A：提交 prefix 执行授权

1. 在 controller 明确 active joint authorization identity。
2. 移除 `compute_control()` 中“缺 advance grant 即停车”的条件。
3. 保留 reservation expired、epoch barrier、window endpoint 和 local emergency stop。
4. 保留旧 `advance_grant` 命令的兼容解析，但不再作为运动必要条件。

退出条件：协议专项证明未提交路线不能运动、已提交路线无需额外 grant 连续运行、超期路线仍停车。

### Step B：同时空动态 Hold

1. Supervisor 从当前 active segment 和 committed peer trajectory 构造重叠时间区间。
2. 只对同一时间区间内的 vertex/edge/swept/terminal 冲突发送 hold。
3. hold 去重为状态边沿；同一 identity 不逐帧重发。
4. hold 到期自动恢复；风险持续时必须由 fresh evidence 续期。

退出条件：同空间不同时间不 hold；同空间同时间必须 hold；stale/terminal/reverse-edge 保持 fail-closed。

### Step C：等待与停滞恢复隔离

1. 有效动态 hold 在期限内暂停 physical progress lease。
2. hold 超期、unknown zero、route-less 和 endpoint successor failure 继续进入 liveness。
3. 单一等待 episode 不得同时触发 soft replan 和 hard escape。
4. 记录每机器人 hold reason/duration/blocker/epoch，状态边沿记录，禁止逐帧磁盘 I/O。

退出条件：合法 hold 不触发 replan；无证据零速超过 2 s 必须被记录并恢复。

### Step D：递增 Webots 验收

1. 30 s 录像 smoke；
2. 180 s 录像诊断；
3. 180 s 无录像性能测试；
4. 前三层通过后执行 900 s 无录像效率验收。

任何层失败立即停止更长测试并保留 JSON、录像、命令及首个失败证据。

## 5. 每一步三轮 Review/Test

1. R1：授权身份、状态机、旧协议兼容；targeted tests。
2. R2：安全、时间冲突、terminal、stale、回滚；相关模块组合测试。
3. R3：连续运动、block、吞吐、Webots 热路径；全套测试、compile、diff check 和对应 Webots gate。

## 6. 硬门禁

### 机器人效率（第一优先级）

- 30/180 s：`joint_advance_grant` 固定等待为 0；无 blocker 的连续零速 episode 为 0。
- 180 s：必须出现业务阶段进展；任何机器人 clear-motion duty 不低于 95%。
- 900 s：tasks completed >= 15、dispatch/task 中位数 <= 3、route dispatch <= 480、rapid override <= 50。
- 全局 clear-motion duty >= 98%，逐机器人 >= 95%；未知/非阻塞连续停止不得超过 2 s。
- terminal service complete 后必须清场，不得永久占站。

### 安全

- pair distance violations = 0；min distance >= 0.50 m。
- motion without committed plan = 0；unauthorized route writes = 0。
- unknown/nonphysical recovery = 0。

### Webots 性能

- 无录像 sim-to-wall ratio >= 1.10，且不低于同机基线。
- whole-step P99 <= basic timestep 50%。
- joint planning P99 <= 50 ms、max <= 100 ms。
- status drops = 0；录像开销不得计入性能结果。

## 7. Workflow Review

### Review 1：授权与安全边界

发现：直接删除 grant 会把动态风险保护一并删除。

修订：只有 transaction activation 构成默认授权；动态 hold、本地制动、stale、terminal 和超期保护全部保留。旧 grant 仅兼容解析。

结论：`PASS AFTER REVISION`。

### Review 2：时间语义

发现：现有 grant validator 使用 peer 当前点与当前 target 的空间距离，可能把不同时间占用误判成冲突；只使用计划时间也会漏掉迟到机器人。

修订：使用 segment 时间区间交集，并以 fresh pose、当前 waypoint 和 late-arrival uncertainty 扩大时间包络；只有时空同时重叠才 hold，stale relevant peer 继续 fail-closed。

结论：`PASS AFTER REVISION`。

### Review 3：活性与恢复

发现：若 hold 自动到期但风险扫描没有及时刷新，可能误放行；若合法 hold 继续累计 stall clock，会重现反馈循环。

修订：短 TTL + fresh evidence 续期；controller 本地风险层始终保留；有效 hold 暂停 stall，超期 hold 和 unknown zero 恢复 liveness。

结论：`PASS AFTER REVISION`。

### Review 4：效率、观测与 Webots 性能

发现：逐帧 hold/grant 日志或录像与性能混测会制造错误结论；900 s 直接运行会延迟失败反馈。

修订：hold 只记状态边沿，metrics 内存累计、结束落盘；录像与性能分跑；30 -> 180 -> 180 performance -> 900 分层门禁。机器人效率先于倍率报告。

结论：`PASS AFTER REVISION`。

## 8. Step E：录屏关联诊断与 successor omission 连续执行修复

180 s 录像与运行账本已通过 `scripts/analyze_webots_stops.py` 自动按机器人和时间关联。实证显示机器人 8 在 97.728–167.008 s 近乎静止 69.280 s，机器人 4 在 122.496–148.096 s 静止 25.600 s；两者没有有效动态 hold，且期间 waypoint index 始终为 0、plan epoch 被连续替换。

修复规则：滚动 successor 只是候选，候选暂时省略机器人或产生 wait-only 成员时，不得撤销或短暂停止该机器人仍有效的 predecessor committed route；只有不存在可执行 committed route 时才允许 0.5/0.6 s 有界 hold。现有 reservation expiry、动态时空 hold、terminal ownership、紧急制动及冲突优先级全部保留。

验收顺序仍为 targeted/combined/full 三轮 review/test，然后 30 s 录像、180 s 录像、180 s 无录像性能；前三层通过后才运行 900 s。重点比较长停车段、逐机器人 motion duty、block 数、dispatch/task 与任务吞吐，Webots sim-to-wall ratio 仍为硬门禁。

## 9. 完成定义

只有 Step A-D 均完成三轮 review/test，且 900 s 同时通过机器人效率、安全和无录像 Webots 性能门禁，才可标记 `COMPLETE`。
