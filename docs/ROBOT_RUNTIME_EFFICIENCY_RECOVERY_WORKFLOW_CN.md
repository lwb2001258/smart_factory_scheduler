# 机器人运行效率与 Webots 性能恢复 Workflow

状态：`APPROVED / NOT STARTED`

日期：2026-09-13

## 1. 唯一目标

在不降低现有物理安全边界、任务优先级和 terminal 清场规则的前提下，恢复并显著提高机器人任务执行效率，同时保证 Webots 仿真运行性能。本 workflow 不以“单测通过”“短测无停滞”或调度器局部 P99 代替真实吞吐和仿真性能。

## 2. 方案三轮 Review

### Scheme Review 1：目标、基线与因果隔离

发现：原方案直接使用历史 `15 tasks/600 s` 作为基线，但当前工作区包含大量未提交修改，历史 JSON 不能证明对应代码可重建；一次 600 s 结果也受墙钟负载影响。继续叠加优化将无法判断收益来自哪里。

修正：先冻结当前工作区 hash、resolved flags、world hash 和测试环境；撤销或隔离尚未通过长测的 5C-19 候选；分别建立“当前状态”和“最近可重建高效状态”的同 seed、同环境成对基线。每次只允许一个独立变量进入 A/B gate，不合格立即局部回滚。

结论：`PASS AFTER REVISION`。

### Scheme Review 2：机器人效率与安全活性

发现：完全取消周期刷新可能让有限 rolling prefix 耗尽；仅做等价路径去重仍可能保留低效旧路线；局部 component 重规划若遗漏外部机器人轨迹可能引入二次冲突。

修正：采用“事件驱动 + prefix 耗尽前 deadline”双触发；路线替换同时检查旧路线安全性、目标身份、前缀等价、预计完成时间收益和转向代价；局部 component 规划必须把外部机器人的已提交时空轨迹作为只读约束，并由统一 validator 复验。安全盾只调速，除非达到明确 liveness deadline，否则不得自行升级为 route rewrite。

结论：`PASS AFTER REVISION`。

### Scheme Review 3：Webots 性能与可测量性

发现：现有 `scheduling_latency_p99_ms` 只测任务调度，不包含 joint planning、Supervisor 全步、JSON 通信、peer broadcast 和日志；单次 `sim_to_wall_ratio` 在 0.98--1.28 之间波动，不能可靠归因。要求“倍率大幅提高”必须先建立同机成对统计口径。

修正：新增整步 wall-time、joint planning wall-time、通信序列化时间和消息量指标；性能 A/B 至少 3 次交替运行并报告中位数与最差值。默认关闭逐 waypoint/正常状态日志，事件只在状态边沿记录。优化不得增加同步 planner、采样循环、device read、IPC 或热路径磁盘 I/O。

结论：`PASS AFTER REVISION`。

## 3. 硬门禁

### 3.1 600 秒机器人效率门禁

- Scene C / FCFS / seed 44 完成任务数 `>=15`；相对当前失败基线 10 tasks 至少提升 50%。
- `route_dispatches <=480`，相对 1601 至少下降 70%。
- `rapid_route_overrides <=50`，且每任务 route dispatch 中位数 `<=3`。
- `unplanned_task_stops == 0`，任一机器人无有效安全/资源证据的连续停止不得超过 2 s。
- 全局 `clear_motion_duty_cycle >=98%`，任一机器人 `>=95%`。
- `pair_distance_violations == 0`、`min_pair_distance >=0.50 m`、unknown/nonphysical recovery 均为 0。
- 首次运动前必须已有 validator 通过且完成 prepare/arm/commit/activation 的路线；`motion_without_committed_plan == 0`。
- `unauthorized _terminal_egress_retry write == 0`、`terminal_egress_retry_failed_without_successor == 0`、服务完成后 2 s 内必须存在已验证 egress successor 或带具体 blocker/epoch 的有效等待证据。

### 3.2 Webots 性能门禁

- fast-no-rendering、无并行 Webots 进程下，3 次交替 A/B 的 `sim_to_wall_ratio` 中位数 `>=1.10`，最差值 `>=1.00`，且相对成对基线中位数不得退化。
- Supervisor whole-step P99 不得超过 Webots basic timestep 的 50%；最大值不得连续两个 timestep 超过 basic timestep。
- joint planning P99 `<=50 ms`，单次硬上限 `<=100 ms`；超预算候选必须取消，不能阻塞仿真步。
- controller status drop 为 0；peer broadcast/状态消息量和 JSON 编解码 wall-time 必须报告。
- 900 s 和最终多 seed 验收继续适用全部安全、吞吐和性能门禁。

### 3.3 Webots 录屏机器人效率门禁

- 机器人运行效率结论必须来自 Webots animation 录屏（JSON/X3D/HTML）与同次 experiment JSON 的时间关联；无录屏运行只用于测量 Webots 计算性能，不能用于证明“没有大面积停车”。
- 每次行为验收至少执行一次 180 s 录屏诊断；进入最终验收后，900 s 也必须保留一轮录屏结果。录屏与无录屏性能测量分开执行，避免将录像开销计入基础仿真倍率。
- `scripts/analyze_webots_stops.py` 必须同时报告：逐机器人停车段、最大同时停车数、三台及以上同时停车累计时长，以及剔除已证实计划等待/恢复动作后的异常并发停车指标。
- 硬门禁：不得存在三台及以上异常机器人同时停车；即 `max_concurrent_abnormal_stops < 3` 且 `abnormal_mass_stop_seconds == 0`。任何未解释停车都必须能从同次日志定位机器人、时间、任务状态、路线版本和触发事件。
- 录屏分析不得只使用平移距离下结论：后续分析器需区分原地转向与真正静止；两者都计入效率统计，但只有平移和朝向均无有效变化才标记为完全停车。

任何硬门禁失败：保留 JSON、命令、hash 和首个失败证据；停止更长测试；仅反向撤销本步骤补丁；不得用平均值、其他 seed 或短测掩盖。

## 4. 实施步骤

### Step 0：退出旧候选并建立可重建双基线

1. 记录 commit、dirty patch、关键文件 SHA-256、world hash、resolved flags、Python/Webots 版本。
2. 将尚未完成 600 s 的 5C-19 代码候选局部撤销或隔离，保留文档和失败证据。
3. 建立当前低效基线；从审计记录定位并重建最近完成 15 tasks 的高效状态，不得使用不可复现 JSON 充当代码基线。
4. 使用相同启动命令交替运行至少 3 次 90 s 和 3 次 600 s，记录墙钟负载及完整指标。

退出条件：两套基线均可由 manifest 重建；若 15-task 状态无法重建，则以当前基线为唯一比较对象，但最终门槛仍保持 15 tasks。

### Step 1：建立真实性能与路线抖动观测

1. 测量 Supervisor whole-step、joint candidate、transaction、通信序列化/发送和 metrics 的 wall-time P50/P95/P99/max。
2. 为每个业务 leg 记录 route dispatch、epoch replacement、首段方向变化、grant hold、replan/recovery 和完成时间；只记录状态边沿。
3. 区分 `business_goal_change`、`prefix_exhaustion`、`safety_invalidated`、`liveness_deadline`、`equivalent_refresh` 和 `candidate_improvement`。

退出条件：事件—汇总严格对账；指标开销使倍率下降超过 2% 时必须优化或移除观测实现。

### Step 2：等价前缀保持与路线替换收益门槛

1. 若目标、成员和安全身份不变，且新旧路线前 2--3 个物理 segment 等价，不更换 epoch/index，只安全延长尾部。
2. 非等价候选只有在旧路线不安全/耗尽，或预计完成时间改善至少 15% 且转向代价计入后，才允许提交。
3. 已进入当前 segment 的机器人保持该 segment，除非 BRAKE/EMERGENCY/stale/validator 明确否决。
4. 首次运动严格执行 plan -> validate -> prepare -> arm -> commit -> activate；任何 provisional/candidate/pending 路线都不得驱动电机。
5. 运行中继续执行当前已验证 prefix，同时预规划 successor；successor 未验证或提交失败时保留旧路线，成功提交后在 segment 边界无扰切换，不得重置当前 segment、制造反向首段或原地乱转。

退出条件：专项测试证明真正冲突仍替换；600 s dispatch/rapid override 达到门禁且 tasks 不下降。

### Step 3：事件驱动 rolling planning

1. 删除普通固定 2 s 全车 route replacement；保留轻量 deadline 检查。
2. 仅由业务目标变化、prefix 剩余时间不足、validator 失效、明确安全冲突或 physical-progress deadline 触发候选。
3. 合并同一 timestep/同一 component 的重复触发，一个事件只产生一个 generation。
4. terminal service complete/charge complete 必须作为最高业务级别的 successor planning event；它改变 joint planning goal 为有限 egress prefix，但不允许未经 transaction 验证的直接 writer 抢占。

退出条件：无 prefix exhaustion 停车；route dispatch 相对当前基线下降至少 70%；600 s tasks `>=15`。

### Step 4：冲突 component 局部重规划与单一干预所有者

1. 冲突只重规划实际 component；其他机器人的已提交时空轨迹作为不可写约束。
2. 每台机器人同一时刻只有一个 intervention owner：SAFETY、TEMPORAL_WAIT、PRIORITY_YIELD、TERMINAL_EGRESS 或 STALL_RECOVERY。
3. safety shield 默认只调速；只有统一 liveness deadline 才能升级路线写入。
4. `_terminal_egress_retry` 不再直接与 joint writer 竞争：retry session 只请求 joint successor generation。成功后原子切换；失败按有界退避保留 session 和 blocker evidence，直到 fresh pose 证明物理清场。
5. terminal egress successor 的局部 component 拥有业务清场优先级，但仍受 static/grid/peer/swept/reservation validator 约束；禁止通过强制抢写、teleport 或冻结全车队实现清场。

退出条件：组外二次冲突为 0；重复 recovery/route writer rejection 显著下降；安全门禁全部通过。

### Step 5：按动作标定连续时间模型

1. 从 Webots fresh pose 采集直行 cell、90°、180°、减速和 terminal approach 的 P95/P99。
2. 用动作相关 duration 替代每个 cell 固定 4.75 s 最坏情况；保留反应和通信 margin。
3. vertex、同步 edge、旋转 swept occupancy、goal residence 和 late-arrival extension 统一使用同一时间合同。

退出条件：路口利用率、planned wait 和任务完成时间改善；任何距离违规立即回滚。

### Step 6：稳定控制目标与连续曲率执行

1. 对已提交路线使用稳定 lookahead；尾部刷新不得改变正在执行的 segment target。
2. 将转向代价纳入候选评分，避免相邻 epoch 首段左右摆动。
3. 保留所有 safety stop 和 grant fail-closed 语义。

退出条件：无理由原地转向/停止均为 0；任务吞吐和路径长度不退化。

### Step 7：Webots 热路径优化

1. peer 信息按邻域/路线相关性广播；全局低频、近邻高频，stale threshold 与周期共同标定。
2. 压缩状态消息；缓存不变 JSON 字段；禁止正常 waypoint 和重复 conflict 控制台输出。
3. metrics 内存累计、结束时落盘；规划移出同步热路径或严格分片预算。

退出条件：3 次成对 600 s 倍率中位数 `>=1.10`、最差 `>=1.00`，status drop 为 0，机器人效率指标不退化。

### Step 8：递增与最终验收

顺序固定为 targeted tests、完整 pytest、30 s、90 s、180 s 录屏行为验收、3×600 s A/B、900 s（含一轮录屏）、至少 10 fixed seeds × 1800 s。前一层未通过不得进入后一层。

### Step 6A：高失败 seed 的候选失败隔离

1. 先按 seed 独立比较 `attempted/validated/failed/timed_out/cache_hits`，不得跨 seed 将行为差异归因于录像或代码回退。
2. 完整联合候选失败时保留已提交 predecessor 继续运动；只有实际 liveness failure 才允许启用既有安全验证过的恢复路径，禁止因一次候选失败全车停车。
3. 若采用部分 successor，未纳入成员的已提交时空轨迹必须作为 timed blockers 输入，且 transaction validator、最小距离和 terminal 所有权规则保持不变。
4. 对连续失败使用事件驱动、有界退避和失败原因计数；禁止逐 timestep 重试，也禁止扩大同步墙钟预算掩盖不可行问题。

Review 1（正确性）：完整候选失败不等于现有 predecessor 失效；继续执行仍在有效窗口内的已提交路线符合“先规划可行再运动”的授权模型。部分 successor 只有显式预留遗漏成员轨迹才可接受。`PASS`。

Review 2（安全性）：不得降低 0.70/0.75 m 规划间距、不得跳过 transaction validator、不得覆盖 recovery lease 或 terminal egress priority；无安全候选时保持旧路线或进入已有受审恢复。`PASS`。

Review 3（性能）：失败处理必须局部、事件触发并复用候选缓存；新增诊断只累计 O(1) 计数。验收同时检查 dispatch/replan/escape、录屏完全静止并发、planning/step P99 和 sim-to-wall ratio。`PASS`。

### Step 6B：录屏驱动的命令—姿态失配恢复

1. 录屏同时关联 controller target、target distance、heading、线/角速度与真实平移/旋转；分别统计完全静止、低进度爬行和原地转向，不再混用“停车”。
2. 只有单台机器人在运动命令持续存在、没有 validated hold/emergency、目标未切换且姿态进度连续超时后，才允许触发局部恢复；不得全局修改正常曲率或速度曲线。
3. 恢复必须复用现有授权 route transaction/replan 机制，有界、边沿触发，并保留 terminal、charging、priority、minimum-distance 和 recovery lease 规则。
4. 录屏硬门禁同时比较最长单车低进度、三台以上低进度累计时长、目标距离收敛、tasks 和安全；无录屏 gate 单独比较 planner/step P99 与 sim-to-wall ratio。

## 5. 每步骤强制 Review/Test 合同

### Step 6C：频繁合法等待不得抹除物理停滞债务

录屏 `20260915_042532` 中 Robot 8 在约 125.824–180.000 s 仅平移不足
3 cm，控制器长期输出运动命令，且没有普通 block；与此同时路线 epoch 被频繁替换，
却没有产生对应的 progress-lease 升级事件。代码审查确认路线 epoch 本身不会重置
lease，但 watchdog 会在每个合法等待采样上把 `last_progress_at` 写成当前时间，频繁
短 hold/window 因而可能抹除等待前已经累计的物理停滞。

三轮方案审查后的实施边界：

1. 先增加低频 pause/joint-wait/dispatch-hold 遥测；不增加控制消息、录屏帧率或规划频率。
2. 合法等待改为显式 pause 区间：进入只记开始时间，退出只扣除真实等待时长，不清零
   等待前的停滞债务。
3. lease 仍只能由实测平移或目标距离改善续租；不改变碰撞、优先级、路线、终点、
   充电站及正常运动控制规则。
4. 先用录屏验证 Robot 8 类长停与 pause 的关联，再实施行为修复。修复后录屏必须降低
   最长和并发低进度时间，且任务数和最小安全距离不退化；随后以无录屏测试检查
   planning P99、real-time ratio 和 supervisor step 性能。

### Step 6D：首路线提交与 fallback 热路径

1. 录屏必须分别报告任务分配、首次 validated dispatch 和首次真实位移时间；首路线迟交
   不能归类为 block，也不能通过把 active 改记为 idle 隐藏。
2. 无 committed predecessor 的新成员只允许使用经时空障碍校验和 transaction validator
   提交的首路线；外部已提交轨迹必须作为 timed blockers，禁止抢写正在执行的路线。
3. 已失败的 partial-successor/predecessor 隔离方案不得重做。候选必须以首次 dispatch
   P95、初始三台以上低进度时间、tasks 和安全距离证明实际收益。
4. fallback 规划热路径不得执行逐次同步磁盘日志；诊断复用低频 metrics，移除 I/O 后以
   targeted/full pytest、compileall、无录屏 P99/ratio 和录屏行为门禁复核。

### Step 6E：联合网格 waypoint 近点物理卡滞推进

录屏显示最长的 command–pose mismatch 多数停在当前 0.25 m 网格 waypoint 的
0.226–0.343 m 范围；联合路线阈值固定为 0.22 m，而普通导航阈值为 0.35 m。禁止直接
全局放宽阈值，以免连续跳过网格占用。

1. 仅同一非最终 target 持续至少 2 s、平移不足 1 cm、距离在 0.22–0.35 m 时成为候选。
2. 必须没有 peer/path conflict、emergency 或 validated advance hold，并已达到 waypoint
   not-before；每次只推进一个既有 validated waypoint，不生成或改写路线。
3. target 改变立即重置；相同 target 的 rolling epoch 替换不得清除真实停滞时间。
4. 只维护 O(1) controller 本地状态，不新增通信/规划/日志。录屏门禁比较 command–pose
   mismatch 累计、最长区间、tasks、最小距离和 reservation-expired movement；无录屏
   门禁检查 ratio 与 planner/step P99。

### Step 6F：电机出口同比例限幅与真实命令遥测

联合 pure-pursuit 分支提前返回，绕过末尾的轮速限幅；例如录屏命令
`v=0.159 m/s, ω=1.106 rad/s` 要求右轮约 9.63 rad/s，超过 Webots 电机上限
6.67 rad/s。必须在唯一电机出口统一处理：

1. 任一轮超限时两轮同比例缩放至 `MAX_SPEED`，保持差速曲率、符号和零命令。
2. motion-state、linear/angular telemetry、segment departure 与 Webots motor 必须使用同一
   个实际限幅后命令，禁止继续上报名义不可执行速度。
3. 不改变 planner、priority、terminal、reservation、minimum-distance 或避让规则；不采用
   已失败的“限制内轮为正”曲率实验。
4. O(1) 本地算术，不新增通信/日志。录屏比较 command–pose mismatch、tasks、安全距离和
   路线稳定性，无录屏检查 ratio/P99。

### Step 6G：reservation cell 与共线控制目标分离

保留每个 0.25 m cell 的 reservation/到达/上报语义，但在连续共线段上允许 pure-pursuit
朝更远 cell 转向，减少每 0.25 m 重新瞄准造成的高角速度和走停。

1. 当前 cell 的到达距离、index、grant、not-before/deadline 和 segment audit 完全不变；
   lookahead 只能改变本周期 steering target，禁止调用额外 advance。
2. 仅联合路线、非首/末 cell、前一段与后一段严格共线同向时启用；重复等待 cell、拐角、
   反向、terminal 和 partial endpoint 均使用当前 target。
3. lookahead 有固定小上限，沿途每个 cell 仍由控制器逐个消费，动态 hold/emergency 优先于
   steering；不改变 planner、priority、terminal 或安全距离规则。
4. O(1) 小窗口几何检查，不新增通信/规划/日志。录屏硬门禁比较 tasks/完成时间、最长与
   并发低进度、unplanned stop、最小距离及 reservation-expired movement；无录屏复核
   ratio/P99。

### Step 6H：陈旧 priority-yield 不得劫持业务到达

Webots 录屏 `20260915_045134` 显示 Robot 2 在 50.032 s 进入 yield，之后无 clear/resume；
168 s 后已物理到达 S2，但 pickup 一直未提交。根因是 `_handle_goal_reached()` 将任何
非空 `priority_yield_state` 都当作 standoff 到达并立即返回。

1. yield 只在状态为 `standoff_enroute`、standoff 存在且 fresh pose 在原到达容差内时拥有该到达事件。
2. `waiting_clear`、缺失 standoff 或位置不匹配的标记先清理原 yield/route-writer lease，再继续原业务到达逻辑。
3. 不改任务优先级、路线规划、冲突判定、终点/充电站或安全距离；原业务目标物理距离复核仍为最终门禁。
4. 仅新增单次 O(1) 状态/距离判断，不增加规划、通信或逐帧日志。录屏门禁必须证明 Robot 2 的 pickup 可提交、目的地不再被无效占用，且 tasks、低进度、安全距离不退化；然后用无录屏运行复核 Webots ratio/P99。

Review 1（正确性）：事件归属同时依赖 yield leg 状态与物理 standoff；不再由历史标记决定。`PASS`。

Review 2（安全/兼容）：有效 standoff 仍走原 yield handler；无效标记清理后仍通过原业务距离校验，不改变原避让和终点规则。`PASS`。

Review 3（性能/可回滚）：修复仅一次 O(1) 几何判断，无新的 planner/message/I/O 热路径；可以独立回滚。`PASS`。

### Step 6I：命令—轮速—底盘进度闭环诊断

1. 复用已启用的左右轮编码器，以 Webots 仿真时间差计算实测角速度；首样本只建立基准。
2. 通过现有 status 包和 4 s metrics 快照携带实测轮速，不增加通信频率、规划或逐帧日志。
3. 非有限值、缺失传感器或 `dt <= 0` 只使诊断样本无效，不得影响电机命令、导航或触发 replan。
4. 录屏对齐最长低进度区间：命令高/编码器低定位 actuator/contact；编码器高/底盘低定位打滑/接触；两者都低再检查控制分支。本步只允许诊断，后续行为修复需重新三轮方案审查。

Review 1（正确性）：编码器角度差分使用仿真时间，禁止以墙钟代替。`PASS`。

Review 2（隔离性）：诊断值不参与任何控制、规划、优先级或安全决策。`PASS`。

Review 3（Webots 性能）：仅对已激活传感器做 O(1) 读取/差分，复用现有包和低频落盘。`PASS`。

### Step 6J：编码器证据触发的局部单轮打滑恢复

1. 仅同一 target >=1 s、位移 <2 cm、航向变化 <0.08 rad，且命令与编码器均显示一轮近零/另一轮高速时候选。
2. 只在 joint coordinated 的已验证 cell 内执行最长 0.75 s 对称原地转向；不改路线、index、epoch、priority 或 reservation。
3. emergency/hold/planned-wait 优先；target 变化或实测位移/转角恢复立即终止，并使用 2 s 冷却防止抖动。
4. O(1) 本地状态，不增加规划、通信和日志。必须录屏比较最长单轮空转、tasks、异常并发低进度、最小距离；任何端到端退化立即回滚本步。

Review 1（范围）：非全局曲率修改，必须同时有命令、编码器和 pose 三方证据。`PASS`。

Review 2（安全）：原地转向不越出已预留圆形 footprint，安全/等待状态具有前置否决权。`PASS`。

Review 3（效率/回滚）：恢复有界且仅 O(1)；不得用局部空转缩短掩盖 tasks/安全退化。`PASS`。

每个 Step 完成后至少执行三轮独立 code review 和三轮验证，并写入独立 acceptance log：

1. Review/Test R1：需求和改动边界；针对性单元/性质测试。
2. Review/Test R2：安全、状态机、并发身份、回滚和旧规则；相关模块组合测试。
3. Review/Test R3：机器人吞吐、路线稳定性、Webots 热路径复杂度和观测开销；完整 pytest、compileall、diff check及该步骤 Webots gate。

Review 必须记录具体发现、修复和复审结论，不能把三次运行同一命令冒充三轮 code review。任一 review 有未解决的高风险问题，该步骤状态为 `BLOCKED` 或 `FAILED`。

## 6. Workflow 三轮 Review

### Workflow Review 1：依赖、顺序与可回滚性

发现：若先修改 planner 再补全性能观测，将无法定位收益；若直接清理工作区可能覆盖用户已有修改。

修正：Step 0 只做精确局部撤销/隔离并保存 patch/hash；Step 1 先建立观测；之后每步一个独立 patch 和证据目录。禁止 `git reset --hard`、整文件覆盖和跨步骤混合回滚。

结论：`PASS AFTER REVISION`。

### Workflow Review 2：安全、效率与停止条件

发现：只要求倍率和吞吐可能通过缩短安全等待获得虚假提升；只要求零碰撞可能继续保留低吞吐系统。

修正：每个 Webots gate 同时检查安全距离、unknown/nonphysical、逐机器人停滞、tasks、dispatch/override、motion duty、倍率和 whole-step P99；安全或效率任一失败都不允许发布。

结论：`PASS AFTER REVISION`。

### Workflow Review 3：统计可信度与成本

发现：单次 Webots 倍率易受机器负载影响，直接执行 10×1800 s 会消耗大量时间并延迟失败反馈。

修正：先用短 gate 排除功能错误，再用三次交替 A/B 600 s 判定性能；只有全部硬门禁通过才进入 900 s 和最终多 seed。保留最差单次结果，不允许只报告平均值。

结论：`PASS AFTER REVISION`。

### Workflow 增补 Review：预规划无扰执行与 terminal egress

- Review 1（执行原子性）：确认“先规划再执行”不能实现为停车等待每次重规划；首次路线必须先提交，运行中必须让旧 prefix 连续执行并并行准备 successor。未提交路线驱动为硬失败。`PASS`。
- Review 2（清场活性）：确认 `_terminal_egress_retry` 直接 writer 被 joint owner 拒绝会形成永久循环；改为 terminal event 驱动 joint successor，session 直到成功提交和 fresh pose 清场才结束。`PASS`。
- Review 3（安全/性能）：禁止强制抢写和全车冻结；复用 transaction/validator，事件边沿触发且有界退避，不逐 timestep 规划或写日志。增加 reject-without-successor、清场时限、dispatch 和 Webots ratio 门禁。`PASS`。

## 7. 完成定义

只有 Step 0--8 均具备三轮 review/test 证据，3×600 s、900 s 和 10×1800 s 同时满足机器人效率、物理安全及 Webots 性能硬门禁，才可标记 `COMPLETE`。任何短测、单 seed 最优结果或不可重建历史 JSON 都不能替代最终验收。
