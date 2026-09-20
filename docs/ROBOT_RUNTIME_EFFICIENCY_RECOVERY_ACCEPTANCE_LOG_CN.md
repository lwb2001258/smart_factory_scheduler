# 机器人运行效率与 Webots 性能恢复验收日志

## Step 0：可重建双基线（2026-09-13）

- 起始 commit：`3a0c5bcee808d68b7888dff41c8b5efe0f7680d7`；工作区 dirty，完整 `git status --short` 已在执行输出冻结，禁止整树回滚。
- 环境：Python 3.12.6；Webots R2023b；world SHA-256 `473DF96233F32E90850B044048CF3E44FBE0C5317967EDF3F1BD8A29F7B2849B`。
- 回滚前关键 hash：supervisor `758069BC07D3C6CB80CFFD8B4700A635A258BB3A331E0AF3A251DE0F6D9E38DC`；joint planner `AEFF72EDF53C0ACD3730FE75B3D5BE0F6D24E17B3A9CA77B427782DAF37E5715`；robot controller `647AE260B9D8318F6B29C37AC5ACEF644C6062D7C21B2BB031C63EBA46BDFFF6`；motion safety `B2B10F8F4BD6374047EFC9DA3938C294C435C83A3C030403AF523266D2466711`。
- 未完成候选隔离：仅撤销 5C-19 的 `_constant_velocity_min_distance`、grant temporal branch 和对应 3 个测试；保留 workflow、原验收日志和 Webots 失败证据。
- Review/Test R1：controllers/tests 中 5C-19 marker 为 0；advance-grant 专项 `6 passed`。结论 `PASS`。
- Review/Test R2：确认 joint planner、priority yield、terminal、安全指标及 robot protocol 未被误撤；组合 `191 passed`，diff check 仅既有 CRLF warning。结论 `PASS`。
- Review/Test R3：完整 pytest `355 passed + 4 subtests`，compileall 通过；无整文件覆盖、无无关文件撤销。结论 `PASS`。
- 历史高吞吐审计：仅找到 `experiment_C_FCFS_20260815_124642.json` 与 `experiment_C_FCFS_20260816_050849.json` 达到 15 tasks；文件缺少 commit/dirty/world/config/execution provenance。后者为 15 tasks、1506 dispatch、227 replans、225 requests、8 unplanned stops，不能作为安全、高效或可重建通过基线。判定 `HISTORICAL_REFERENCE_ONLY`。
- 基线策略：当前回滚状态为唯一可复现候选；执行同命令 3×90 s 与 3×600 s。最终效率门槛仍为 600 s `>=15 tasks`，不因历史基线不可重建而下调。
- 3×90 s：`203543/203705/203830`，倍率 1.118/1.061/1.223，中位数 1.118、最差 1.061；dispatch 238/211/197，rapid override 68/43/43；均为完整 90 s，环境性能可重复但路线抖动显著。
- 3×600 s：`204009/204847/205723`，tasks 9/7/5（中位数 7），倍率 1.153/1.167/1.153，dispatch 1477/1371/1400，rapid override 435/406/302，unplanned stops 3/8/5；最小距离 0.550/0.594/0.549 m，距离违规均为 0。结论：Webots 基础倍率稳定，机器人执行效率严重失败。
- terminal egress 证据：三轮 `_terminal_egress_retry` unauthorized reject 为 4/266/0，成功 dispatch 均为 0。第二轮 Robot 8 从 T=119.392 s 以同一 path_version=35/plan_epoch=148 按约 2 s 周期持续拒绝至接近结束，证明 writer 冲突可造成永久占站和热路径事件洪泛。
- Step 0 Review 1（身份）：6 个结果均核验 fresh timestamp、完整时长、Scene C/FCFS/seed 44；无并行 Webots。`PASS`。
- Step 0 Review 2（安全/效率）：三轮 600 s 均无距离违规，但 tasks/dispatch/stop 均未达门禁；明确记录为低效基线而非通过结果。`PASS`。
- Step 0 Review 3（性能）：90 s 倍率中位数 1.118、最差 1.061；600 s 中位数/最差均 1.153，说明后续必须保持该倍率，同时优先解决执行逻辑。`PASS`。
- Step 0 状态：`COMPLETE`。进入 Step 1；不得直接修改规划策略，先建立 whole-step、route-cause 和 terminal retry successor 对账观测。

## Step 1：真实性能与路线抖动观测（2026-09-13）

- Implementation Review/Test R1：新增 Supervisor 整步与 joint planning wall-time 内存样本，以及由既有 route events 派生的 source/override transition 汇总；不改变决策、不新增热路径日志。专项 `2 passed`。`PASS`。
- Implementation Review/Test R2：确认整步计时从 `Supervisor.step()` 前开始，覆盖 Webots 同步、通信与本步逻辑；组合测试 `177 passed`。`PASS`。
- Implementation Review/Test R3：600 s 约 3.75 万浮点样本，内存有界；完整 pytest `357 passed + 4 subtests`，compileall 通过，diff check 仅既有 CRLF warning。`PASS`。
- Webots 90 s `experiment_C_FCFS_20260913_211035.json`：倍率 1.122，较 Step 0 三轮中位数 1.118 无退化；0 unplanned stop、最小距离 0.754 m。Supervisor step P50/P95/P99/max 为 7.75/17.48/169.08/1583.56 ms；joint planning 163 samples，P50/P95/P99/max 为 129.59/291.65/356.18/391.68 ms，严重超过 50/100 ms 门禁。
- 路线归因：243 dispatch 中 `joint_grid_transaction=236`、`_joint_stall_recovery=7`；76 rapid override 中 joint->joint=72。90 s 理论固定 2 s tick 约 45 次，但实际 joint planning 163 次，证明重复 liveness/retry 和同源覆盖是控制尖峰及乱转的主因。
- 状态：`IN_PROGRESS`；计时观测自身通过开销门禁，但需补齐通信/消息量与 terminal retry successor 对账后才能进入 Step 2。
- 通信观测 Review/Test R1：新增 peer broadcast/command/receive 的 wall-time、消息数和字节数内存累计；专项 `3 passed`。`PASS`。
- 通信观测 Review/Test R2：发现精简 metrics 测试替身没有新方法会把正常 command send 误判为失败；改为 capability guard，并兼容旧 mock broadcast 返回值。组合 `186 passed + 4 subtests`。`FIXED / PASS`。
- 通信观测 Review/Test R3：确认消息内容、频率和决策不变；完整 pytest `358 passed + 4 subtests`，compileall/diff check 通过。`PASS`。
- Webots 30 s `experiment_C_FCFS_20260913_211447.json`：倍率 1.374、Supervisor step P99 20.11 ms、joint planning P99 37.89 ms；1875 step samples。peer broadcast 15000 messages/23,328,084 bytes，P99 0.978 ms；receive P99 0.942 ms；command 350 messages/52,824 bytes，P99 0.082 ms。route source 汇总 23/23 对账。
- 性能诊断：peer broadcast 数据率约 0.78 MB/s、每 32 ms 固定发送 8 条全量 peer JSON，属于明确的 Step 7 优化对象；但长窗口整步尖峰主要来自高频 joint planning，而非通信单次成本。
- Step 1 状态：`COMPLETE`。全部观测为内存累计、结束落盘，短测倍率无退化；进入 Step 2 等价前缀保持与替换收益门槛。

## Step 2：等价前缀保持与替换原子性（2026-09-14）

- 方案 Review 1（安全）：仅普通同目标刷新允许抑制；成员变化、目标变化、liveness、前缀耗尽或预留余量不足 1.5 s 均旁路。`PASS`。
- 方案 Review 2（等价性）：按 occupancy grid 比较 fresh pose 后最多 3 个去重 cell，要求成员集合完全一致；不以浮点 waypoint 相等冒充物理等价。`PASS`。
- 方案 Review 3（性能）：比较复杂度为 O(robots × 3)，无新 planner、I/O、设备读取或逐步日志；只新增一个整数汇总。`PASS`。
- Implementation Review/Test R1：发现旧实现会在候选产生前退休 activated transaction；改为保留旧事务直至候选成功。等价、近过期、方向变化、liveness 与失败候选专项 `6/6`。`PASS`。
- Implementation Review/Test R2：复审发现 prepare 本地预检失败时仍可能形成无事务窗口；将 retire 下沉至 `_begin_joint_plan_transaction`，在非空、非静止、writer preflight 全部通过后才原子替换。专项 `6/6`，supervisor `117/117`。`FIXED / PASS`。
- Implementation Review/Test R3：成员集合、预留期限、失败回滚、旧控制器路径连续性与指标开销复审通过；联合测试 `210/210`、全套 `217/217`，py_compile 与 diff check 通过（仅既有 CRLF warning）。`PASS`。
- Webots 30 s gate：`BLOCKED_BY_ENVIRONMENT`。三次启动均在 controller 产生输出/JSON 前由 `webots-bin.exe` 崩溃；Windows Application Event 1000/1001 指向 `Qt6Gui.dll 6.5.1.0`、exception `0xc0000005`、offset `0x00000000000b9ed0`。`--version/--help` 正常；软件 OpenGL、重建可恢复 `.wbproj`、官方 `--clear-cache` 均未消除崩溃。项目布局文件已恢复，未留下本次诊断性删除。
- Step 2 状态：`IMPLEMENTED / WEBOTS_GATE_BLOCKED`。不声称性能通过，不进入 90 s/600 s，也不在 Webots 硬门禁缺失时推进 Step 3 行为修改；恢复图形会话或重启 Webots 主机后，应首先重跑同命令 30 s gate。

## Step 2/3 恢复验证与事件驱动修正（2026-09-14）

- Webots 图形环境恢复后，10 s 探针成功；30 s `20260914_123213` 倍率 2.075、step P99 10.76 ms、joint P99/max 20.12 ms，安全与连续运动通过。
- 90 s `20260914_123241` 暴露 predictive shield 将普通调速误升级为全车 replan：211 dispatch、166 planning、0 tasks。修正为只有 `<0.70 m` 临界路线干预才请求 successor；专项 3/3、联合 212/212、全套 219/219。
- 90 s `20260914_123525`：dispatch 降至 163、planning 108，但合法 joint wait 仍被 progress lease 计入停滞。修正合法有界等待暂停物理停滞时钟，stale-wait watchdog 保持；专项 3/3、联合 214/214、全套 221/221。
- 90 s `20260914_123731` 暴露 endpoint status 每帧触发 planner，且 fresh-plan request 提前释放 activated txn。修正为 `(robot, epoch, reason)` 边沿一次触发，并保留旧事务到 successor preflight；专项 3/3、联合 216/216、全套 223/223。
- 删除固定 2 s 全车 planning tick，改为业务目标、endpoint、安全/liveness 和无可执行事务失败重试驱动。30 s `20260914_174420`：倍率 1.926、dispatch 14、planning 4、rapid override 0、stop 0、clear duty 100%、step P99 9.72 ms、joint P99/max 20.14 ms。专项 4/4、联合 218/218、全套 225/225。
- 单机器人 endpoint 仍会连续替换全体成员，故增加 transaction completion barrier。专项 3/3、联合 219/219、全套 226/226。
- 最新 90 s `experiment_C_FCFS_20260914_174724.json`：倍率 2.422、0 距离违规、min distance 0.833 m、clear duty 99.01%；但 tasks 0、dispatch 234、rapid override 70、planning 61、joint P99/max 285.99 ms，仍失败。route ledger 显示多数 epoch 各产生 8 次 dispatch，证明有限 horizon 的 all-active successor 架构仍是主要放大器。
- 状态：Step 2 原子保留和 Step 3 固定 tick 删除已实现，但总体效率门禁 `FAILED`。停止 600 s；下一步进入 Step 4 component-local successor：只改写实际耗尽/冲突 component，其他已提交时空轨迹作为只读约束，同时保留全局 validator 与 terminal priority。

## 可视化诊断与独立性能复测（2026-09-14）

- 新增默认关闭的 `SMART_FACTORY_DIAGNOSTIC_RECORDING`，调用 Webots R2023b `animationStartRecording/animationStopRecording`；诊断录制与无渲染性能验收严格分离。编译、相关 177/177、全套 226/226、diff check 通过。
- 180 s 诊断回放：`webots_diagnostic_C_FCFS_20260914_180205.html`，配套 `.json/.x3d/.css`；结果 `experiment_C_FCFS_20260914_180205.json`。tasks 0/18、有效等待 98.08 s、非阻塞零速 4.96 s、3 stop episodes、dispatch 176；Robot 1 duty 94.99%，Robot 2 duty 95.77% 且 3 次 stop。录像确认停车不是渲染假象。
- 180 s 无录制性能复测：`experiment_C_FCFS_20260914_180458.json`。倍率 1.878、安全距离 0.724 m、距离违规 0；但 tasks 0/18、有效等待 101.44 s、非阻塞零速 5.60 s、4 stop episodes、dispatch 179、planning 137、joint P99/max 257.69/315.52 ms。Robot 1 duty 94.35%、Robot 2 duty 95.56%（3 stops）、Robot 8 1 stop。
- 判定：`ROBOT_EFFICIENCY_FAILED / WEBOTS_RATIO_PASS / PLANNER_LATENCY_FAILED`。录制与无录制两次结果高度一致，证明大面积停车来自程序的 successor/事务等待，而非渲染或录像开销。禁止进入 900 s 验收，先修复 component 外机器人被单一事务状态影响及局部失败扩大后的重复规划。

## 录屏停车关联与 successor omission 修复（2026-09-14）

- 新增 `scripts/analyze_webots_stops.py`，从 Webots animation JSON/X3D 恢复逐机器人位置时间轴，并与 task state、planned/unplanned wait、progress lease、replan、escape、route dispatch、terminal handoff 关联；专项 3/3、组合 209/209、全量 379/379。
- 旧 180 s `181703` 自动定位 Robot 8 非计划静止 69.280 s、Robot 4 非计划静止 25.600 s；期间 waypoint index 固定为 0，epoch 连续替换。
- 修复 rolling successor 暂时省略成员时对仍有 committed predecessor 的机器人重复短 hold。180 s 录像 `183036`：tasks 0→2、上述两段长停消失、unplanned task stops 2→0、replan request 67→14、clear duty 97.76%→99.18%、逐机器人最低 96.77%、min distance 0.828 m、违规 0、倍率 1.446×。
- 180 s 无录像 `183305`：tasks 2、clear duty 98.90%、逐机器人最低 96.76%、违规 0、倍率 1.416×；但 planning P99/max 124.58/300.35 ms，性能门禁未全通过。
- 严格 50 ms 同步预算的两版实验均保留为失败证据：`184022` 为 tasks 2、duty 95.26%、dispatch 448；`184446` 为 tasks 1、duty 92.33%、dispatch 488、非计划停车 4、倍率 0.841×。已撤销该退化策略并恢复 `183036/183305` 对应的高运动效率实现。
- 当前判定：`ROBOT_EFFICIENCY_MAJOR_IMPROVEMENT / SAFETY_PASS / WEBOTS_RATIO_PASS / SYNCHRONOUS_PLANNER_P99_FAIL`。下一步必须采用跨 step 的增量候选规划，不能再通过削减求解时间牺牲机器人连续运动；900 s 继续禁止启动。

## 录屏并发停车硬门禁（2026-09-15）

- 验收口径修正：机器人效率必须使用 Webots 录屏和同次运行日志联合判断；无录屏运行仅测 Webots 基础性能。不能再以全局 motion duty 或零 `unplanned_stop_events` 代替画面中的并发停车检查。
- 180 s 录屏 `webots_diagnostic_C_FCFS_20260915_034154.json/.x3d/.html` 对应 `experiment_C_FCFS_20260915_034154.json`：51 个平移停车段，46 个发生于运动预期状态，17 个尚未被现有事件分类解释；普通同时停车峰值 4 台，三台及以上累计 26.208 s。
- 剔除已证实的计划等待及 progress recovery 后，异常同时停车峰值仍为 3 台；Robot 1/3/7 在 173.664--175.072 s 重叠停车 1.408 s。最长未解释段为 Robot 5 的 79.072--83.744 s（4.672 s），随后收到新路线；这证明当前版本仍未通过“大面积异常停车为零”的行为门禁。
- 分析器新增异常并发指标与回归测试。Review/Test R1：专项 `5 passed`；R2：协议/监督器组合 `165 passed`；R3：全量 `385 passed, 4 subtests passed`，`py_compile` 与 `git diff --check` 通过（仅既有 LF/CRLF 提示）。
- 当前判定：`RECORDING_BEHAVIOR_GATE_FAILED / WEBOTS_COMPUTE_GATE_SEPARATE`。在异常并发停车清零且区分原地转向/真正静止前，不用无录屏高倍率结果宣称机器人效率通过，也不进入 900 s 最终验收。

## 姿态感知录屏分析与 seed 44 对照（2026-09-15）

- 修复录屏分析器只读取 `translation`、忽略 rotation-only frame 的误差；现同时报告无平移片段、原地转向片段和位置/朝向均无变化的完全静止片段。旧录屏 `034154` 的 51 个无平移片段中有 23 个实际在转向；完全静止异常同时最多 2 台，三台以上累计 0 s，因此撤销此前基于纯平移作出的“该次存在三台同时停死”结论，但仍保留短时单车静止和转向效率问题。
- Supervisor 低频 time-series 现保存控制器 motion state、线/角速度、stop reason、local risk 和状态采样时间；字段复用既有 controller status，不增加通信频率、逐 timestep 日志或规划调用。
- 新 180 s 录屏 `webots_diagnostic_C_FCFS_20260915_035104.*`（seed 44）：tasks 1、dispatch 259、replan 40、escape 30、clear duty 97.21%；96 个无平移片段中 53 个为原地转向，完全静止异常峰值 3 台、三台以上累计 2.4 s，行为门禁失败。可明确识别 `emergency`、`advance_hold:swept_occupancy`、`local_planner_zero_replan` 及 `commanded_without_pose_progress`。
- 同 seed 44 的 180 s 无录屏对照 `experiment_C_FCFS_20260915_035451.json`：tasks 1、dispatch 259、replan 40、escape 30、clear duty 97.21%，与录屏行为一致；倍率 1.174×。录屏/无录屏 planning P99 分别为 391.25/362.52 ms，候选失败均为 100/194 量级。因此低吞吐和停车不是渲染或录屏导致。
- 不能将 seed 42 的 `034154` 与 seed 44 直接作为录像 A/B：seed 42 候选失败 8/134，而 seed 44 为 100/194。当前优先根因是 seed 44 的时空联合候选大量失败及其恢复风暴。
- 分析器与低频遥测实施 Review/Test R1：专项 9/9；R2：组合 198/198；R3：全量 `391 passed, 4 subtests passed`；编译和 diff check 通过（仅既有换行提示）。
- 当前判定：`SEED_44_ROBOT_EFFICIENCY_FAILED / RECORDING_CONFIRMS_PROGRAM_BEHAVIOR / WEBOTS_RATIO_PASS / PLANNER_LATENCY_FAIL`。下一步分析候选失败原因分布和失败后的 predecessor 连续执行，禁止通过降低安全间距或关闭规则换取吞吐。

## Step 6A 单成员 predecessor 隔离实验（已回滚，2026-09-15）

- 实验实现：完整候选首次失败后，下一规划事件省略一台拥有有效 predecessor 的最低优先级普通机器人，并将其剩余路径作为 timed blockers；充电、terminal egress、emergency/recovery 均禁止省略。每个事件仍只执行一次 planner 调用。
- 三轮实现测试在修正显式 recovery-window 保护后通过：专项 129/129、组合 `190 passed, 4 subtests passed`、全量 `393 passed, 4 subtests passed`。
- seed 44 无录屏 180 s `experiment_C_FCFS_20260915_040213.json`：候选失败 100→60、dispatch 259→219、replan 40→24、escape 30→17、planning P99 362.52→250.07 ms、倍率 1.174→1.534；安全距离 0.621 m、违规 0。但 tasks 仍为 1，unplanned stops 3→4，未达到机器人效率目标。
- 结论：该方案改善计算开销和恢复风暴，却没有改善吞吐并增加未计划停车，按硬门禁完整回滚行为代码及对应临时测试。回滚后专项 127/127、组合 `188 passed, 4 subtests passed`、全量 `391 passed, 4 subtests passed`，编译/diff check 通过。
- 下一方向：不再以“减少 planner 调用”作为成功标准；直接处理录屏已确认的长 `advance_hold/emergency` 与 `commanded_without_pose_progress`，要求 tasks、完全静止和 Webots 性能同时改善。

## 录屏优先的等价 successor index 实验（已回滚，2026-09-15）

- 录屏根因假设：联合路线等价性比较读取不推进的 Supervisor `current_waypoint_idx`，而不是 epoch 匹配的 controller index，可能导致 successor 重派发和 waypoint 0 重置。方案经正确性、安全性、性能三轮 review 后实施，并通过专项 129、组合 190、全量 393 项测试。
- 按“先录屏再判定”的要求运行 seed 44、默认 4.75 s 时间槽的 180 s 录屏 `webots_diagnostic_C_FCFS_20260915_041134.*`。与修改前 `035104` 比较：tasks 1、distance 76.752 m、dispatch 259、replan 40、escape 30、clear duty 97.21%、53 个原地转向片段、异常完全静止峰值 3 台及三台重叠 2.4 s 均完全相同；equivalent refresh suppressed 仍为 0。
- 结论：该字段问题不是此次效率故障的实际触发点，修改无行为收益，立即完整回滚。回滚后三轮验证为专项 127、组合 188、全量 391 项（另 4 subtests），编译/diff check 通过。
- 录屏进一步证明：多数机器人目标方向有效进度占比 78%--99%，并非普遍随机绕圈；异常是高路径版本 churn 下物理速度显著低于轮速命令。Robot 8 在 180 s 仅移动 2.77 m、距 WS1 仍 11.09 m，且多次出现 `commanded_without_pose_progress`，后续以该机器人为主要诊断对象。

## 低进度定义修正与曲率实验（曲率修改已回滚，2026-09-15）

- 分析器修复：旧 `pose_stationary` 只检查朝向且停车区间错误包含越过 3 cm 阈值的下一帧。现记录区间内最大平移，使用默认 5 mm + 0.10 rad 判定真正姿态静止，并保留 3 cm 低进度指标。专项 10、组合 199、全量 392 项通过。
- 重分析 `041134`：三台以上低进度累计 62.112 s、峰值 7 台，但三台以上完全静止为 0 s；此前“大面积停死”应更正为“大面积低速爬行/转向”。这不改变机器人效率失败结论。
- 新增原有 4 s 快照中的 controller target、target distance 和 heading，无新增通信或逐步日志；专项/组合/全量为 156/200/393。录屏 `042532` 捕获 Robot 8 在 125.824--180.0 s 持续 54.176 s 低进度：位置约 `(4.70,-0.89)`、目标 `(4.50,-1.00)`、目标距离约 0.23 m、heading error 约 0.56 rad，命令约 0.159 m/s + 1.108 rad/s。
- caster 低摩擦假设经录屏 `041851` 否证：轨迹、最终位置及全部行为 KPI 与 `041134` 完全相同，world 修改与临时测试已回滚。
- 中等转向曲率限制方案经三轮 review 后实验；测试 35/173/394 通过。录屏 `043122` 中 Robot 8 最长低进度由 54.176 降至 5.984 s，但全车 tasks 1→0、distance 77.27→63.64 m、clear duty 98.08%→94.94%、三台以上低进度 47.136→63.104 s、并发峰值 5→7，系统效率恶化，故完整回滚。
- 曲率回滚后三轮验证：专项 34、组合 172、全量 393 项（另 4 subtests），编译/diff check 通过。下一修复只能在已经观测到持续命令-姿态无进展的单车上局部触发，禁止全车改变正常转向曲线。
## Step 6C 录屏因果复核（2026-09-15 04:39）

- 观测字段专项/组合/全量测试为 156/166/393（另 4 subtests），compileall 通过；字段复用原有 4 s time-series，没有新增控制消息或 timestep 工作。
- seed 44、180 s 录屏 `webots_diagnostic_C_FCFS_20260915_043955.*` 对应 `experiment_C_FCFS_20260915_043955.json`。分析得到 96 个低进度区间、90 个 active 区间；三台以上低进度累计 62.112 s，异常低进度并发累计 27.552 s。该轮完成 1 task，最小实测距离 0.550 m。
- Robot 8 在 109.792--123.968 s 的低进度区间内，112.016 s 已产生 hard escape，之后恢复平移；4 s 快照未显示持续 controller pause/joint wait/hold。因此“pause 反复清零 lease”在本轮没有成立，Step 6C pause-accounting 行为修改暂缓，禁止在无录屏因果证据下提交。
- 更稳定的根因是首路线迟交：Robot 4/5/6/7/8 从仿真开始分别低进度约 21.504/24.192/44.640/27.168/53.120 s；首次 dispatch 分别为 21.120/23.616/43.904/26.528/52.032 s，其中 3/7/8 依赖 stall recovery 才取得首路线。结论：任务分配后没有及时获得已验证的可执行首路线，是本轮大面积停车的主要来源。
- 已复核此前 partial-successor/predecessor 隔离实验 `040213`：候选失败虽 100→60，但 tasks 保持 1 且 unplanned stop 3→4，不能重复作为首路线修复。下一候选必须专门处理“无 committed predecessor”的 route-less 新成员，并把外部已提交轨迹作为 timed blockers，仍经 transaction validator；仅改变 idle/active 记账不算效率提升。
- fallback 热路径同步写 `fallback_debug.log` 已移除；三轮验证为 146（另 4 subtests）/184（另 4 subtests）/393（另 4 subtests），compileall 与 diff check 通过。该修改不改变规划/安全规则。
- 同 seed 44 无录屏 180 s `experiment_C_FCFS_20260915_044514.json`：tasks 1、dispatch 259、replan 40、candidate failed 100、倍率 1.087×。行为与 `035451` 相同，倍率受墙钟噪声影响且没有提升（1.174×→1.087×）；因此只能判定同步 debug I/O 清理正确，不能宣称带来可测性能收益，更不能宣称机器人效率改善。主要开销仍是 194 次候选中的 100 次失败规划。

## Step 6D 首路线稳定性门修复（2026-09-15 04:51）

- 根因复审：新 route-less component 的 planning event 会在 `_refresh_joint_grid_candidate()` 中先后被 ordinary refresh cadence 和 active transaction unstarted-member 两道提前返回挡住；Robot 2 的旧首 dispatch 8.112 s 正好对应旧 transaction 的 prefix refresh interval。修复只允许与 active transaction 成员完全不相交的新 component 绕过这两道稳定性门；旧轨迹仍作为逐 slot blocker，新路线仍经原 planner 和 prepare/arm/activate transaction。
- 三轮 code review/test：新增旧代码会失败的定向测试；专项 129、组合 185（另 4 subtests）、全量 394（另 4 subtests），compileall/diff check 通过。相交请求、普通 refresh、优先级、terminal、最小距离和 controller 运动规则均未改变。
- 修改后录屏 `webots_diagnostic_C_FCFS_20260915_045134.*`：Robot 2/3/4/5/7 的 assignment→first validated dispatch 从 6.048/8.080/6.048/5.872/6.000 s 降至统一 0.608 s；候选失败 99→29、replan 40→19，首个任务完成 173.776→111.264 s，录屏倍率 0.971×→1.521×，最小距离 0.550→0.616 m。
- 同 seed 无录屏 `experiment_C_FCFS_20260915_045422.json` 复现：candidate attempted/validated/failed/timeout = 146/117/28/1，replan 19，倍率 1.467×；相对旧无录屏 `035451` 的 100 failures、40 replans、1.174×为稳定改善。
- 行为门禁仍未完全通过：tasks 仍为 1；录屏 active low-progress 85 段，三台以上低进度累计 62.848 s，异常并发累计 30.272 s。Step 6D 标记 `FIRST-DISPATCH_AND_WEBOTS_PERFORMANCE_PASS / FLEET_THROUGHPUT_PENDING`，下一步按录屏拆分 active 低进度原因。

## Step 6E 近点卡滞推进实验（2026-09-15 14:01，已回滚）

- 依据 `045134` 录屏提出仅对联合路线非最终 waypoint 的近点持续卡滞推进一个 cell；三轮方案审查覆盖同一 target/2 s/1 cm、0.22–0.35 m、not-before、无 conflict/hold/emergency、O(1) 本地状态。实现审查曾发现并修复“不合格状态沿用旧 anchor”和“同周期可能推进两点”两个缺陷。
- 修改前专项/组合/全量为 37/222/397（另 4 subtests），compileall/diff check 通过。
- seed 44、180 s 录屏 `webots_diagnostic_C_FCFS_20260915_140128.*`：tasks 仍为 1，但首任务完成 111.264→153.696 s，unexplained active 累计 206.240→219.840 s，abnormal mass stop 30.272→37.280 s，最小距离 0.616→0.574 m；虽 mass low-progress 62.848→51.840 s、replan 19→10、reservation-expired movement=0，机器人任务效率和安全裕量仍明显退化。
- 判定 `FAILED / FULLY_ROLLED_BACK`：controller 行为和三项新增测试已撤销，workflow 条目与失败录屏保留，禁止重做“近点自动跳 waypoint”方向。

## Step 6F 电机限幅/重复 speed-scale 隔离实验（2026-09-15 14:06--14:09，已回滚）

- 代码确认联合 pure-pursuit 可产生超过名义 `MAX_SPEED` 的轮速，并在 navigator 与主循环重复应用 `speed_scale`。方案三轮审查要求同比例限幅保持曲率、实际命令遥测一致、只应用一次 scale，不改变规划/避让规则。
- 合并版本录屏 `webots_diagnostic_C_FCFS_20260915_140633.*`：unexplained active 206.240→112.064 s、abnormal mass 30.272→23.008 s、最小距离 0.616→0.646 m，但 tasks 1→0，最长 unplanned stop 45.792 s，判失败。
- 隔离后仅取消重复 scale 的录屏 `webots_diagnostic_C_FCFS_20260915_140915.*`：unexplained active 115.424 s、abnormal mass 21.184 s、最小距离 0.683 m、倍率 2.032×，但首任务完成 111.264→162.832 s，unplanned stop 累计 42.496→91.040 s，端到端效率退化。
- 判定 `FAILED / FULLY_ROLLED_BACK`：同比例限幅和单次 scale 行为均撤销；不得用更好的局部停车统计掩盖更差的任务完成时间。Step 6D `045134/045422` 继续作为当前稳定基线。

## Step 6G 共线 steering lookahead 实验（2026-09-15 14:14--14:18，已回滚）

- 方案保留逐 cell reservation/index/arrival，仅在非首末、严格共线同向的小窗口中把 steering target 前移；三轮审查要求不跨拐角、重复等待、反向、terminal，O(1) 且 hold/deadline 优先。专项/组合/全量为 36/221/396（另 4 subtests）。
- 初版录屏 `webots_diagnostic_C_FCFS_20260915_141457.*`：mass low-progress 62.848→28.768 s、abnormal mass 30.272→5.920 s、候选失败 29→5、replan 19→9、最小距离 0.616→0.761 m、倍率 2.224×，但 tasks 1→0。复审发现可能旁路当前 cell 后无法进入 0.22 m 到达圆。
- 增加“沿 incoming 投影已越过且横向误差 <=0.10 m”的物理穿越判定，专项/组合/全量为 38/223/398（另 4 subtests）；修正版录屏 `webots_diagnostic_C_FCFS_20260915_141803.*` 仍为 0 task，未恢复端到端完成。

### Step 6H：陈旧 priority-yield 到达所有权修复

- 根因证据：基线录屏 `045134` 中 Robot 2 在 50.032 s 进入 `waiting_clear`，之后没有 clear/resume 事件；到达 S2 后 `_handle_goal_reached()` 仍被该陈旧标记直接返回，pickup 未提交。
- 实现：仅 `standoff_enroute` 且实测位置在 standoff 的 `2.5 * GOAL_TOLERANCE` 内时拥有 yield 到达；其余陈旧标记经现有 clear-state 清理后继续原业务到达逻辑。未改 priority/planner/terminal/safety 规则，仅 O(1) 距离判断。
- Review/Test R1：专项 21 passed。初次测试曾错误期望业务 leg writer 被保留；复核确认原 handler 必须释放已完成 leg，修正断言后通过。
- Review/Test R2：Supervisor + robot protocol 组合 163 passed；有效 standoff 仍被接管，`waiting_clear`/位置不匹配不再吞掉业务到达；`git diff --check` 无新错误。
- Review/Test R3：全量 397 passed + 4 subtests；录屏 `webots_diagnostic_C_FCFS_20260915_142742.*` 与结果 `experiment_C_FCFS_20260915_142742.json` 显示 Robot 2 在 164--168 s 从 `en_route_pickup/S2` 正常转为 `en_route_delivery/WS5`，而基线到 180 s 仍卡在 S2。异常三车以上低进度累计 30.272 -> 21.696 s，最小距离 0.616 m 持平，tasks 仍为 1（Robot 2 在窗口尾部 pickup，尚未来得及 delivery）。
- Webots 性能门禁：无录屏 `experiment_C_FCFS_20260915_143004.json` 的 sim-to-wall ratio 1.755x，高于无录屏基线 `045422` 的 1.467x；dispatch 268 -> 266、replan 19 -> 18，最小距离 0.616 m，无性能或安全退化。
- 判定 `FAILED / FULLY_ROLLED_BACK`：lookahead、穿越判定及四项测试全部撤销。低进度/安全/倍率改善不能替代任务完成，禁止继续扩大 cell 消费语义。
