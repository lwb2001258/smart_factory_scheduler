# 机器人避障、路线预规划与占道规划实现审查

## 1. 文档目的与审查边界

本文基于审查时的项目工作树，对机器人导航安全相关实现进行静态审查和已有 Webots 结果复核，覆盖：静态路径规划、路线预规划、空间/时空占道、联合规划、冲突预测、机器人端局部避障、让行、重规划和死锁恢复。

本次工作只形成分析意见，没有修改控制器、规划器、配置、世界文件或算法代码。文中“问题”包含确定的实现不一致，也包含需要通过专项实验验证的工程风险；后者会明确标注为风险而非已证实缺陷。

审查边界和证据限制：

- 审查时工作树包含未提交修改，本文未绑定一个可重建的 Git commit；因此“当前实现”仅指审查时读取到的工作树。
- Webots JSON 是历史实验结果。虽然文件内含 provenance hash，但本文尚未证明这些结果与审查时导航代码、世界文件及全部运行时开关逐项一致。
- 因此，静态代码问题可以直接确认；“某段当前代码导致某项历史实验指标”的表述只能视为相关性假设，除非另有相同版本、相同配置的对照实验。
- 后续复核应记录 `reviewed_commit`、dirty 状态、关键文件 hash、world/map fingerprint、最终解析的环境变量和实验输入文件清单。

## 2. 代码默认运行组合

未设置相应环境变量覆盖时，代码默认配置为：

- `ENABLE_JOINT_RUNTIME = true`：默认导航使用全活动机器人滚动时空联合规划。
- `ENABLE_RUNTIME_RHCR = false`：CBS/RHCR 运行时闭环默认关闭。
- `ENABLE_PROACTIVE_JOINT_SPEED = false`：旧的联合速度优化器关闭。
- `ENABLE_PRIORITY_YIELD_RESUME = false`：新的确定性优先级“让行—等待—安全恢复”状态机默认关闭。
- `ENABLE_LEGACY_INTERLOCK_RECOVERY = false`：旧联锁恢复关闭。
- `ENABLE_NONPHYSICAL_RECOVERY = false`：正式实验禁止瞬移恢复。

因此，在未被环境变量覆盖的前提下，默认主链路是：

```text
业务目标/任务分配
  -> JointGridPlanner 短窗时空路径
  -> Supervisor 两阶段 prepare/arm/commit/activate
  -> RobotController 按 waypoint 时间窗执行
  -> Supervisor 0.5 s watchdog + 6 s 轨迹预测速度盾
  -> 紧急联合重规划
  -> 长时间无进展时物理 escape，再回到联合规划
```

非联合模式下才主要使用 `plan_grid_lifelong()`、绝对时间资源预约、`_proactive_path_conflict_scan()` 和多层 legacy deadlock monitor。

## 3. 静态地图与 Grid A*

### 3.1 地图建模

`controllers/factory_supervisor/grid_planner.py` 将约 18 m × 11 m 厂房离散为 0.25 m 栅格，使用 8 邻接移动。地图包含：

- 物理障碍单元；
- 按机器人半径和额外安全裕量膨胀的禁止单元；
- 可通行但带惩罚的 avoid 单元；
- 厂房边界阻挡；
- 货架狭窄间隙的策略性禁行区。

当前额外静态膨胀 `SAFETY_MARGIN` 为 0.55 m，再叠加机器人半径 0.18 m，属于较保守的静态障碍策略。货架间 1 m 缝隙被主动封闭，机器人主要绕货架组东西两侧通行。

### 3.2 路径搜索和后处理

`GridAStar` 使用 octile 启发函数、对角防切角、Theta*-style 视线平滑和 waypoint 子采样。终点靠近工位或充电站时，会临时放松 endpoint 周围的 inflated cells，但不会放松真实 obstacle cells，并用 `try/finally` 恢复地图。

优点：

- A* 启发函数与八邻域代价匹配；
- 防止对角穿角；
- endpoint relaxation 有恢复保护，不会永久污染共享地图；
- 最终路线会重新栅格化和验证，而不是直接相信平滑前路径。

风险：

- endpoint relaxation 对普通终点周围放松 2 格、充电站周围放松 4 格，且放松的是固定方形区域而非连接终点与永久自由区的最小通道。虽然不穿越真实 obstacle，但充电站周围最多形成 9×9 cell 的临时自由区域，可能让路线进入原本用于机器人外形裕量的区域；这里依赖机器人端 LiDAR 和局部控制做最后保护。更稳妥的实现应只开放经验证的 terminal approach corridor 或最小连通 connector。
- `MotionCoordinator._segment_clear()` 另行硬编码了一套货架/工位矩形，没有直接复用 `OBSTACLE_BOXES`、边界和完整 grid 语义。地图配置改变后，A* 与最终线段验证可能产生漂移。
- 静态规划使用 0.25 m 栅格和较大膨胀，能够保守避开静态障碍，但动态机器人安全并不能从静态膨胀直接推出。

## 4. 单机器人路线预规划与占道

`MotionCoordinator.plan_grid_lifelong()` 是非联合模式及恢复路径的重要入口，其过程是：

1. 保留当前机器人的旧预约，建立可回滚快照。
2. 将其他机器人未来路径的近端前缀栅格化，并扩大为临时硬障碍。
3. 先规划空间分离路线；失败后把 peer 区域改为高代价区域再规划。
4. 仍失败时可退回不考虑 peer 空间占道的最短路线，再通过时间延迟串行化。
5. 对 lane separation、子采样后的最终控制器路线重新栅格化。
6. 为 grid node 和无向 grid edge 建立绝对时间预约；搜索 0～20 s 的 0.5 s 延迟。
7. 延迟仍不可行时使用 space-time detour。
8. 最终执行静态线段、空间覆盖和时间冲突审计；发送失败则恢复旧预约。

### 4.1 占道数据结构

项目同时存在三类占道信息：

- `_grid_path_reservations`：路线经过的栅格集合，带邻格 footprint 膨胀，偏空间占道。
- `ReservationManager`：带 `[start,end)` 的 node/edge/zone 绝对时间预约，支持按 owner 原子替换、快照和回滚。
- `LifelongPlanner` 的图节点时空预约：用于旧拓扑图 Cooperative A*/CBS 兼容路径。

`ReservationManager` 将正反方向 edge canonicalize 为同一资源，可以阻止迎面交换；`reserve_batch()` 先检查完整 batch，再替换 owner 旧预约，原子性设计合理。

### 4.2 主要风险

- 时间估计使用 0.22 m/s 和固定每次转向 0.5 s；实际纯跟踪转向、速度盾、LiDAR 减速和物理惯性不会严格匹配该模型。
- 虽然主循环每 1 s 根据实测进度刷新预约，但在刷新间隔内仍可能存在名义预约领先于真实机器人位置的窗口。
- 允许 unrestricted fallback 是活性优先策略。其安全性依赖后续时间预约、hold、Supervisor 预测和机器人端制动；任何一层延迟都会缩小余量。
- 近端 peer path 是硬障碍、远端共享靠时间串行化，这一混合策略合理，但 `hard_peer_prefix=4.0 m` 和延迟上限 20 s 都是经验值，未见基于速度误差分布的系统标定依据。

## 5. 全机器人滚动联合规划

### 5.1 JointGridPlanner

`joint_grid_planner.py` 实现基于优先顺序的短窗 space-time A*，不是最优 CBS。状态为 `(cell, time_slot)`；当前 `MOVES` 只允许等待和四向移动，不生成对角步。规划器检查：

- 同一时槽的中心距冲突；
- 反向 edge swap；
- 两条移动线段间的最小距离；
- 单元静态可通行性；
- 目标在剩余 horizon 内的持续占用安全。

MotionCoordinator 依次尝试四档规划器：

| 档位 | horizon | slot | 最小中心距 | 用途 |
|---|---:|---:|---:|---|
| wide | 8 | 1.2 s | 0.75 m | 优先使用，更大跟踪余量 |
| primary | 12 | 1.2 s | 0.70 m | 常规回退 |
| short | 4 | 1.2 s | 0.70 m | 搜索超时回退 |
| soft | 8 | 1.2 s | 0.70 m | 较软搜索参数，标记 relaxed |

搜索按一个或少量 priority order 顺序进行，按已有高优先级路径逐个预约。它能给出可验证的可行解，但不是 complete multi-agent planner：某个顺序失败不表示问题无解。

当前不存在“搜索生成对角步、validator 又拒绝”的运行时矛盾：`MOVES` 与 validator 都采用四邻域语义。代码仍保留不可达的对角代价分支和未调用的 `_diagonal_clear()`，属于残留代码和未来维护风险。如果以后重新启用对角移动，必须同时接入 corner-clear 检查并放宽 validator，增加“允许安全对角、拒绝切角”的成对测试。

此外，soft 档设置 `separation_cells=2`，但也设置 `minimum_distance_m=0.70`。冲突函数在米制阈值非空时不会使用 `separation_cells`，因此所谓 soft separation 实际没有按注释从 3 格降为 2 格；它与其他档位的主要差异只剩 horizon/搜索过程。该问题不会直接降低当前安全距离，但会使回退层级的行为和设计说明不一致。

### 5.2 共享目标的 staging

多个机器人目标相同时，离目标最近/优先级较高者使用真实 dock，其他机器人被分配附近 staging point，避免多个机器人同时长期占据同一终点。这是当前高密度任务流能够持续运行的重要机制。

### 5.3 原子下发协议

Supervisor 使用 `JointPlanTransaction`：

```text
preparing -> ready -> arming -> armed -> committing
-> committed -> activation_confirmed -> activated
```

所有机器人先接收计划、确认准备，再确认 armed，随后在共同 `activate_at` 提交并确认激活。任一机器人拒绝、消息发送失败或超时都会 abort，并向成员广播取消。路径带 `path_version`、epoch 和 route-writer generation，可防止旧消息覆盖新路线。

这是当前实现中最扎实的部分之一：它避免了多机器人计划只下发一半造成的瞬时不一致。

### 5.4 联合计划的局限

- 1.2 s slot 内假设从一个 0.25 m cell 进入相邻 cell；实际机器人最大速度 0.22 m/s，直线跨格约需 1.14 s，几乎没有转弯和控制误差余量。
- wide 0.75 m 计划失败后允许 0.70 m；实际结果显示中心距经常落到约 0.50～0.55 m，说明 nominal plan clearance 被跟踪、滚动切换或恢复过程显著消耗。
- 每 2 s 滚动刷新，而联合 horizon 为 4.8～14.4 s。旧计划在新 transaction 完整确认前继续执行，这有利于活性，但要求旧计划尾部和新计划起点的时空衔接始终可靠。
- 联合规划使用优先级规划，不具备 CBS 的 completeness；高密度状态下频繁 changing priority/order 可能形成规划抖动。
- soft candidate 的注释称由预测速度盾恢复物理安全裕量，但速度盾基于简化匀速轨迹，并不模拟 DWA、角速度、轮速加速度或通信延迟，因此只能视为风险降低，不能视为形式安全保证。

## 6. Supervisor 冲突预测与速度盾

联合模式下，`_joint_runtime_watchdog()` 默认每 0.5 s 执行：

1. 更新硬停滞和联合停滞时钟。
2. 调用 `_joint_collision_scan()`。
3. 检查 emergency、route-less、过期等待和 3 s 无进展。
4. 请求新的全活动机器人联合计划。
5. 长时间无进展时升级为物理 escape。

`_joint_predictive_speed_shield()` 对未来 6 s、每 0.25 s 采样轨迹，以 0.85 m 为预警距离。冲突时按 component 选择速度比例；低于 0.70 m 时尝试移动到安全点，失败则短 hold 并降速，然后立即安排联合重规划。

优点：

- 冲突按连通 component 合并，能处理三台以上链式冲突；
- 速度调整不重置路径版本；
- 紧急路线写入和普通联合路线有明确优先级；
- route-less、合法等待和真正停滞被区分；
- 路线 writer lease 防止多个恢复模块互相覆盖。

风险：

- 预测模型是 waypoint 上的简化速度模型，不含机器人实际角速度、轮速加速度、DWA 偏航和 LiDAR 触发后的轨迹变化。
- 0.5 s watchdog 周期相对 0.22 m/s 速度意味着两次检查间最多约 0.11 m 位移；对只有 0.05～0.10 m 剩余安全余量的场景偏紧。
- 配置与日志并不完全一致：启动日志仍写“1 s scan、0.5 s sample、0.5 m radius”，实际联合速度盾为 0.5 s watchdog、0.25 s sample、0.85 m predictor。运维人员可能根据错误日志判断系统状态。
- `collision_safety.py` 中已经存在 CPA/TTC 和制动距离模型，但当前没有被 Supervisor 或 RobotController 实际调用；生产冲突判断仍散落着多组硬编码阈值。

## 7. 机器人端局部避障

`WaypointNavigator` 的局部控制包含：

- 每帧 LiDAR 紧急制动；
- peer position 广播驱动的径向停止和路径投影减速；
- DWA 对 `(v,w)` 候选轨迹打分；
- dock zone 特殊 pure-pursuit；
- 侧移状态和无可行 DWA 轨迹时的原地旋转恢复；
- waypoint 时间窗、联合 epoch barrier 和 path-version 检查。

### 7.1 联合模式下的实际行为

当 `joint_coordinated` 或 `direct_navigation` 为真时，机器人端认为计划已经完成时空协调：

- 保留 LiDAR `<0.25 m` 紧急制动；
- 保留 peer 中心距约 `<0.55 m` 的硬停止；
- 跳过常规 peer 路径投影制动；
- 联合路径执行以 pure pursuit 为主，不让 DWA 偏离预约轨迹。

这种设计避免“中央预约路线被局部 DWA 改写”，但也带来关键权衡：联合计划必须足够准确，否则机器人端只有很薄的最后制动层。

### 7.2 参数不一致

- RobotController 最大线速度为 0.22 m/s；Supervisor `config.py` 另有 0.26 m/s 常量，而若干预测/预约又硬编码 0.22 m/s。
- RobotController 文件顶部重复定义 `MAX_LINEAR_SPEED`，当前值相同但容易未来漂移。
- Supervisor 预测阈值主要是 0.85/0.70/0.50 m，机器人联合模式径向停止约 0.55 m，非联合模式径向停止约 0.80 m。
- RobotController 注释仍按 32 ms/帧估算部分 cooldown，但 Supervisor 基础 timestep 是 16 ms；虽然对应 look-ahead 调用当前被禁用，这仍属于潜在恢复风险。
- DWA 静态 critical distance 为 0.40 m，而 LiDAR hard stop 为 0.25 m；二者与静态 Grid A* 的约 0.73 m 障碍膨胀含义完全不同，没有统一参数来源。

## 8. 让行、重规划与死锁恢复

### 8.1 优先级

统一优先级 key 综合：是否已进入关键区域、任务优先级、等待年龄、冲突距离和 robot ID。这样可避免只按 robot ID 永久饿死低优先机器人。

代码中已经实现完整的 priority-yield-resume 状态机，包括冲突分类、standoff 候选、让行路线、等待 peer clear 和安全恢复原路线；但默认开关为关闭，因此正式默认行为主要使用速度盾中的简化让行及 stall escape。

### 8.2 物理 escape

联合 watchdog 在持续无进展后选择一台机器人，清除 hold 状态，搜索多个角度和 0.3～1.5 m 距离的候选落点，要求：

- peer clearance 至少约 0.80/0.85 m；
- 目标 grid free；
- 直线段无静态障碍；
- 能通过 `plan_grid_lifelong()` 和最终 dispatch gate。

完成 escape 后，不直接恢复单机业务路线，而是重新加入下一轮全活动联合计划。非物理瞬移默认关闭，符合真实部署要求。

### 8.3 CBS/RHCR 与 legacy deadlock

项目包含完整 graph CBS、Lifelong Cooperative A*、循环等待检测、priority inheritance 和 RHCR 接口。但生产联合模式下，多数 legacy deadlock 路径被显式关闭；运行时 RHCR 默认也关闭，因为它可能阻塞同步 Webots controller loop，且当前候选未形成完整事务化采用闭环。

因此不能把“仓库里存在 CBS 实现”等同于“正式 Webots 正在使用 CBS 避障”。当前生产的核心仍是 prioritized joint grid planning。

## 9. 已有验证与实测证据

### 9.1 测试覆盖

相关测试覆盖了：

- Grid/时空联合规划的 vertex、edge 和线段冲突；
- transaction 不完整确认时不激活、超时回滚、abort 广播；
- route-writer lease 和未经授权的路线写入拒绝；
- stale path version 拒绝；
- 机器人 joint barrier、planned wait 证据和 emergency replan；
- 三机器人 conflict component 合并；
- speed transaction 失败回滚；
- progress monitor、stale wait 和恢复路线。

历史复核记录为：`84 passed, 4 subtests passed`。由于原记录没有保存精确 pytest 命令、测试文件清单、执行时间、代码版本、skip/xfail/warning 和报告文件，该数字当前不可独立复现，不应作为持续有效的测试基线。后续应把完整命令和测试报告与审查版本一同归档。

不足：

- 缺少独立的 `collision_safety.py` 测试文件，而且该模块目前未接入生产链路；
- 单元测试以协议和几何函数为主，不能覆盖 Webots 跟踪误差、通信抖动和多机器人高密度长期行为；
- 缺少针对统一阈值、制动可达性和 reservation conservativeness 的性质测试/参数扫描。

### 9.2 最近 20 个 Webots 结果

按文件 `LastWriteTime` 降序选择 `results/experiment_*.json` 的最近 20 个结果进行只读汇总；这些文件均为场景 C、DQN、约 1800 s 的运行。后续应固定为 JSON 内 timestamp 或显式 manifest 顺序，并保存输入文件名与 seed，避免文件复制或触碰时间改变样本集合。

- 最小 pair distance 平均约 `0.523 m`；
- 观测到的最低值约 `0.481 m`；
- 共记录 2 次 pair-distance violation；
- 每次 1800 s 运行平均约 415 次 replan；
- 每次平均约 383 次 physical escape；
- 每次 fleet run 的累计 `planned_wait_total_seconds` 平均约 937 robot-seconds；该值不是单机器人平均等待时间，多机器人等待区间也可能重叠。

这些结果中仅记录 2 次 0.50 m 门槛违规，未显示普遍失控，但已经不满足零违规验收目标，也不足以证明安全兜底总体有效。系统频繁进入恢复路径，说明 escape 已从“最终少见兜底”变成常见交通调节手段。需要注意，escape 后通常会主动安排 fresh joint plan，因此 `physical_escapes` 与 `replans` 在控制流上存在耦合；两者数量接近不能单独证明滚动 joint plan 与物理执行之间存在系统性偏差。应按触发来源拆分 replan，并用同版本轨迹误差数据验证这一假设。

## 10. 问题清单与风险等级

本文使用以下等级：P0 表示已观测到违反明确安全验收，或实现上存在可直接突破安全边界的问题；P1 表示高概率影响安全裕量、活性或核心协议正确性；P2 表示可观测性、可维护性或尚未证明会直接破坏安全的行为漂移。只有静态证据或历史指标相关、尚未通过同版本动态实验建立因果关系的项目，明确标为“待验证风险”。

### P0：必须优先验证的安全问题

1. **实际最小间距已经低于 0.50 m 门槛。** 最近结果最低约 0.481 m，并有明确 violation。当前不能宣称严格满足 0.50 m 安全边界。
2. **联合模式机器人端硬停止余量偏薄。** nominal joint clearance 0.70/0.75 m，但真实运行常降到 0.50 m 左右；机器人联合模式约 0.55 m 才径向停止，无法覆盖所有跟踪/通信/转向误差。
3. **时空模型未覆盖真实运动学。** 1.2 s slot 接近理论直线跨格极限，尚未系统包含转向、加减速和控制周期延迟。

### P1：高优先级架构问题

1. **安全参数分散且语义不统一。** 0.25、0.40、0.50、0.55、0.70、0.75、0.80、0.85 m 分布在多个文件，分别代表 LiDAR、DWA、硬边界、联合停止、规划和预测阈值，没有单一可追溯配置。
2. **恢复触发过于频繁。** 平均数百次 escape 表明恢复路径已经成为常见交通调节手段；“滚动 joint plan 与物理执行存在系统性偏差”是需要按 replan 来源和同版本跟踪误差进一步验证的假设，不能仅由 escape/replan 数量接近推出。
3. **静态几何存在双实现。** OccupancyGrid 与 `_segment_clear()` 的障碍数据源不同，布局更新可能导致验证盲区或误拒绝。
4. **已有 CPA/TTC 安全模型未接入。** 当前主要靠距离阈值，不能根据相对速度和实际停止距离自适应判断风险。
5. **高级让行状态机默认关闭。** 代码已经实现 standoff/wait/resume，但生产仍更依赖临时 speed shaping 和 escape，可能是恢复次数过高的原因之一；是否启用必须用 A/B 实验验证，不能直接改默认值。
6. **联合候选选择与类契约不一致。** 类说明称比较多个 priority rotations 并选择最低成本完整解，但实现遇到第一个通过验证的候选即返回；这会影响路线效率、公平性和稳定性，也会误导维护者与测试设计。
7. **soft 回退参数名义生效、实际未生效。** `minimum_distance_m` 覆盖了 `separation_cells`，实现行为与注释不一致；同时 `is_relaxed` 会让日志和指标误以为实际安全间距已经放宽。
8. **审查代码与实验结果缺少可重建版本绑定。** 当前工作树、world/map、运行时环境变量与历史结果 provenance 尚未形成闭合证据链，代码到实验现象的确定性归因不成立。

### P2：中优先级工程问题

1. 启动日志的扫描周期、采样间隔和冲突半径与实际联合逻辑不一致。
2. 16 ms timestep 与机器人端部分 32 ms 注释/估算不一致。
3. prioritized joint planning 不是 complete planner，失败原因目前更多按 timeout/no-solution 统计，缺少顺序敏感度和瓶颈 cell 诊断。
4. 预约刷新为固定 1 s，没有根据速度盾、DWA、hold 或状态新鲜度动态扩大窗口。
5. 多套 legacy/new 恢复代码同时保留，虽然开关隔离较清晰，但维护中容易出现参数和行为漂移。
6. `JointGridPlanner` 仍保留不可达的对角移动代价分支和未调用的 `_diagonal_clear()`；当前不是运行时安全缺陷，但重新启用对角移动时容易只修改一半语义。
7. “代码默认值”与“某次运行最终生效值”尚未在启动日志和实验结果中清晰区分。

## 11. 建议改进方案

### 阶段 A：建立统一安全契约，不改变算法结构

目标：先让所有层对同一物理含义达成一致。

1. 建立唯一 `MotionSafetyConfig`，统一机器人半径、最大实测速度、最大减速度、控制/通信延迟、hard collision、planning clearance、prediction clearance 和 stale-data 阈值。
2. 根据公式计算而不是手写停止距离：

   `d_stop = v * reaction_latency + v² / (2 * deceleration) + tracking_error_p99 + margin`

3. OccupancyGrid、JointGridPlanner、Supervisor predictor、RobotController 和指标门禁全部引用该契约。
4. `_segment_clear()` 改为复用统一 grid/障碍几何 API；保留独立 validator，但数据源必须一致。
5. 启动时打印最终解析后的全部安全参数和来源，并写入实验 provenance。

验收：参数一致性测试、地图/segment validator 差分测试、不同速度和延迟下的制动距离性质测试。

### 阶段 B：校准联合时空模型

目标：让正常 joint plan 自身可靠，降低 watchdog/escape 使用率。

1. 从 Webots 日志拟合直行、90°转向、短 waypoint 和拥堵减速的 P95/P99 traversal time。
2. 将 slot 时间从固定 1.2 s 改为动作相关持续时间，或至少提高到覆盖 P99 跨格时间。
3. 在 space-time edge 中加入旋转占用和到达不确定性 buffer。
4. 明确联合规划保持四邻域还是重新启用对角移动。若保持四邻域，删除不可达的对角代价分支和未调用的 `_diagonal_clear()`；若启用对角移动，搜索扩展必须调用 diagonal corner-clear，validator 同时允许通过该规则的对角相邻步，并增加“允许安全对角、拒绝切角”的双向测试。
5. 明确 soft tier 的真实目的：如果只想改变搜索 horizon 就删除误导性的 separation 参数；如果确需放松距离，则必须由统一安全契约规定下限，且不能低于正式安全门禁。
6. reservation refresh 使用实测进度预测剩余到达窗口，并在 stale/降速时只向未来延长，避免预约提前过期。
7. 滚动窗口提交时验证旧计划剩余段与新计划 prefix 的连续时空安全。

验收目标建议：在固定 10+ seeds、1800 s 场景 C 中，pair violation 必须为 0；P99 physical escape 和 replan 数至少下降一个数量级，同时完成率不退化。

### 阶段 C：使用 CPA/TTC 驱动动态安全盾

目标：替代散落的纯距离阈值。

1. 将 `collision_safety.assess_motion_risk()` 接入 Supervisor，并使用测量速度、状态时间戳和 braking distance。
2. 轨迹预测加入当前 heading、angular velocity、speed scale、hold、waypoint timing 和最大跟踪误差带。
3. CLEAR/CAUTION/BRAKE/EMERGENCY 映射为明确动作：保持、限速、协调停车、紧急停车/重规划。
4. RobotController 保留独立硬安全层，但阈值由统一安全契约生成。
5. 对消息陈旧设置 fail-safe：超过 caution age 降速，超过 stop age 停止，而不是继续相信旧 peer pose。

### 阶段 D：优化交通规则，减少 escape

目标：让冲突通过可解释的通行权和等待点解决。

1. 对 `ENABLE_PRIORITY_YIELD_RESUME` 做严格 A/B，不直接上线：同一 seeds 比较 violation、完成率、等待、escape、replan 和公平性。
2. 将货架两端、中央交叉口和 dock 入口建模为显式 zone resource，进入前预约，避免双方都进入狭窄区后再处理。
3. 对共享 dock 建立容量 1 的队列/staging reservation，提前排序而不是接近终点才动态 staging。
4. right-of-way 加 aging 上限，验证高优先任务和普通任务都不会饿死。
5. escape 只保留为最后门禁，记录触发原因、冲突 component、原计划误差和候选失败原因，驱动规划器改进。

### 阶段 E：规划器演进

1. 保留 prioritized planner 作为快速首选。
2. 对首选顺序失败的 component，尝试有限个确定性 priority orders 或局部 CBS，而不是立即软化 clearance。
3. CBS/RHCR 放入异步 worker 或跨 timestep 有预算的增量求解，避免阻塞 Webots 同步循环。
4. 只有通过同一 transaction 和独立 validator 的 CBS/RHCR candidate 才允许下发。
5. 不建议直接启用当前 runtime RHCR 开关，因为现有注释已明确 candidate adoption/同步阻塞问题尚未解决。

## 12. 推荐验证矩阵

每项改进均应至少包含：

- 单元：vertex、edge swap、交叉线段、旋转占用、endpoint relaxation、stale pose、制动距离。
- 协议：prepare/arm/commit 丢包、延迟、乱序、重复消息、机器人拒绝和 controller 重启。
- 场景：迎面、同向追尾、十字交叉、三机器人链式冲突、共享 dock、低电量机器人、静止机器人堵路。
- 压力：8 机器人场景 C，至少 10 个固定 seeds，每个 1800 s。
- 故障注入：定位噪声、pose 延迟、LiDAR 短时缺失、单控制器慢帧、planning timeout。
- 指标：最小距离、violation、TTC、emergency brake、replan、escape、planned wait、完成率、吞吐、延误和公平性。

还应增加以下故障和边界场景：Supervisor 暂停或重启时的 fail-safe 行为；controller 重启后旧 epoch、armed transaction 和预约的失效；peer pose 乱序和时钟偏差；失联/静止机器人持续占道；slot 0 已经发生距离冲突；机器人提前、滞后或偏离路线后的预约延长与释放；中心距恰好等于各级阈值时 `<`/`<=` 语义一致性；带载荷旋转时的 swept volume。最小间距指标还应报告采样周期，并用连续轨迹插值或更高频记录降低漏掉采样间极小值的风险。

正式安全门禁建议至少包括：

1. 所有验证 seed 的 `pair_distance_violations == 0`。
2. 不允许以平均值掩盖单 seed 安全退化。
3. 非物理恢复次数必须为 0。
4. emergency/escape/replan 应有明确上限并相对基线不退化。
5. 所有实验记录安全契约 fingerprint、地图 fingerprint、Webots 版本和完整开关组合。

## 13. 总体评价

当前项目已经具备较完整的多机器人安全工程框架：静态栅格、时空冲突、原子联合下发、路线写权限、机器人端独立制动和物理恢复均有实现，尤其 transaction/rollback 和 stale-version 防护值得保留。

主要问题不是“完全没有避障”，而是各层模型和阈值尚未统一，联合时空计划对真实跟踪误差估计偏乐观，导致系统在高密度 Webots 中频繁依赖 watchdog、重规划和 physical escape。下一阶段应优先降低这种恢复依赖，并在任何吞吐优化前先证明所有 seed 上 0.50 m 安全门槛不再被突破。

## 14. Webots 中大范围无意义绕行专项评审

### 14.1 结论

Webots 中观察到的“机器人为避让而大范围绕行”不一定只是控制跟踪误差。从静态代码逻辑看，共享目标 staging 评分、联合候选选择、滚动重规划稳定性和 physical escape 评分均可能主动生成安全但效率很低的路线。其中，共享业务目标的 staging 评分是最明确、最可能直接造成大范围绕行的代码点。

### 14.2 可能原因

#### 14.2.1 共享目标 staging 评分偏向远处

多个机器人共享一个工位或存储点时，当前逻辑只让 primary 机器人使用真实目标，follower 机器人使用最大半径 3.5 m 的临时 staging 点。候选评分可概括为：

`score = clearance + 0.4 * radius - 0.05 * robot_to_staging_distance`

该公式会奖励更大的候选半径，但对额外行驶距离的惩罚很小，也没有计算“当前位置→staging→业务目标”的完整增量路程。因此，机器人即使已接近工位，也可能被导向较远的空旷区域。如果 primary 身份在滚动规划中变化，还可能重复分配不同方向的 staging 点。

#### 14.2.2 联合规划返回第一个可行顺序，未选择全局最优候选

`JointGridPlanner.plan()` 会构造多个 priority rotation，但实际在第一个候选通过验证后就立即返回。这与“评估多个顺序并选择代价最低完整解”的类说明不一致。结果是规划器保证了可行性，但没有比较总路程、最大个体绕行、等待成本和路线变更成本，低优先级机器人可能承担过大绕行。

#### 14.2.3 代价函数缺少绕行和路线稳定性惩罚

时空 A* 的普通移动成本为 1.0，等待成本为 1.5。当等待比移动昂贵时，搜索在部分冲突场景中可能更愿意让机器人继续移动绕行，而不是原地短时错峰。当前成本还未显式包含：

- 相对旧路线的偏离成本；
- 更换走廊、掉头和连续转弯成本；
- 新旧路径差异或路线切换次数；
- 单机器人最大绕行率和多机器人公平性。

#### 14.2.4 滚动短窗可能造成走廊左右切换

当前联合规划使用 4、8 或 12 个时间片，每片 1.2 s，并持续从机器人实测位置重新生成短前缀。从实测位置重规划是正确的，但由于没有路径滞回和走廊保持条件，外部机器人位置的小幅变化可能让候选路线在货架左右两侧反复切换，累积后表现为 S 形、折返或大圈绕行。

#### 14.2.5 physical escape 和 stall recovery 允许较大偏移

`_joint_escape_robot()` 会在当前位置周围搜索 0.45～1.75 m 的临时目标。该评分考虑 peer clearance 和向业务目标的进展，但距离惩罚仅为 `0.04 * distance`，没有直接限制相对原路线的横向偏移。机器人到达 escape 点后才从新位置恢复到原业务目标，因此一次 escape 本身就可能引入一段明显偏离。如果 escape 被频繁触发，视觉上就会表现为持续无意义绕行。

stall recovery 的路线附近采样最远约 4 m，relocation 径向搜索最远约 3 m。路线附近恢复的排序本身相对合理，但如果其参考的当前路线已经被上一轮绕行改写，“回到原路线”仍不一定是业务层的最短或最稳定路线。

### 14.3 建议修复方案

#### 阶段 1：增加绕行门禁和可观测指标

对每个新候选路径计算：

- `direct_cost`：当前位置到业务目标的静态最短路径长度；
- `candidate_cost`：新候选前缀长度加候选末端到业务目标的静态剩余长度；
- `detour_ratio = candidate_cost / direct_cost`；
- `extra_distance = candidate_cost - direct_cost`。

可将 `detour_ratio <= 1.30`、拥堵状态逐级放宽至 1.50 作为待标定的实验候选值，而不是直接采用的默认硬门禁。单独使用 ratio 会在 direct path 很短时失真，也可能拒绝唯一安全可行的绕行。实际接受条件应联合 `extra_distance`、预测等待成本、TTC/冲突风险、业务目标距离下界和当前恢复状态，并为 `direct_cost` 设置分母下限。emergency escape 可临时超限，但应单独记录原因和增量距离；一般超限时优先比较短时等待、速度错峰和受约束局部绕行。

建议日志增加 `detour_ratio`、`extra_distance`、`corridor_switches`、`path_replacements`、`escape_lateral_offset` 和 `staging_radius`，以便区分“必要安全绕行”和“规划抖动”。

#### 阶段 2：重做 staging 候选评分和身份稳定

staging 评分应改为以完整增量路程为主：

`score = 安全余量奖励 - 到staging路径长度 - staging到终点路径长度 - 额外路程 - 掉头/换侧惩罚`

同时建议：

1. 正常 staging 先搜索 0.75～1.5 m，无解时才逐级扩展至 2.0 m 以上。
2. staging 点应优先靠近原进站路线，而不是单纯追求空旷。
3. staging 分配增加粘性；只要安全且目标占用者未变，就保留上一个 staging。
4. primary 机器人增加有限时长的租约，避免每个滚动窗口换人。
5. 长期方案是将共享 dock 建模为容量为 1 的队列资源，并为等待位设定固定或经验证的 staging zone。

#### 阶段 3：比较多个联合候选，不再只返回第一个可行解

在固定规划时间预算内保留并比较多个 priority rotation，建议使用如下舰队级目标：

`fleet_score = 总行驶距离 + 等待成本 + 最大单机绕行惩罚 + 路线切换惩罚 + 掉头/转弯惩罚`

除总成本外，必须对 `max_robot_detour_ratio` 设独立门禁，避免通过牺牲一个低优先级机器人来降低其他机器人的成本。可采用 anytime 策略：先保留首个可行解，剩余预算继续寻找更好解，预算到期后返回当前最佳候选。

#### 阶段 4：为滚动重规划增加路径滞回

新路线只在以下条件之一成立时替换当前安全路线：

- 旧路线已被预测为不安全；
- 新路线成本明显更低，例如 `new_cost < old_cost * 0.90`；
- 新路线明显降低冲突风险；
- 机器人已持续无进展；
- 当前 joint epoch 的前缀已耗尽。

新旧路线代价近似时，应保持当前走廊和转向选择。现有 transaction、epoch、旧路径快照和 route-writer 权限已提供所需的协议基础。

#### 阶段 5：限制 escape 为最后生存性手段

建议冲突处理顺序为：

`速度错峰 → 短时等待 → 受绕行门禁约束的局部重规划 → 路线附近避让 → 全方向 physical escape`

建议增加：

- 单机器人和同一冲突对的 escape 冷却及次数上限；
- escape 目标相对原路线的横向偏移上限；
- 正常 escape 优先限制在 1.0～1.25 m，只有确认无解时才扩展至 1.75 m；
- escape 完成后的方向锁定，防止立即选择相反方向的新 escape；
- 连续 escape 时升级为整个 conflict component 的协调规划，而不是继续单机反复恢复。

### 14.4 可行性与风险

| 方案 | 可行性 | 主要收益 | 主要风险 |
|---|---|---|---|
| 绕行指标与门禁 | 高 | 快速识别并拒绝明显不合理候选 | 门禁过严可能使拥堵状态无解，需分级放宽 |
| staging 评分和租约 | 很高 | 直接减少共享终点附近的远距离绕行 | 需验证 staging 点不阻塞其他路线 |
| 多 priority order 比较 | 中高 | 降低总路程和最大个体绕行 | 同步规划耗时上升，需严格预算 |
| 路径滞回 | 高 | 减少走廊切换、折返和路线抖动 | 旧路线锁定过强可能延迟必要避让 |
| escape 分级和限频 | 高 | 避免 escape 从最后兜底变成常规交通手段 | 限频前必须保证等待/局部重规划能提供生存性 |

上述修复不需要替换现有 Grid A*、JointGridPlanner 或联合事务协议，可以依托已有的静态路径代价、滚动规划、epoch、rollback 和 route-writer 逐步实施。总体可行性较高，建议优先顺序为：`staging 评分 → 绕行指标 → 路径滞回 → 多候选比较 → escape 分级`。

### 14.5 建议验证方法

修复后建议对每个阶段进行独立 A/B，固定地图、任务流和 seeds，至少检查：

1. `pair_distance_violations == 0`，安全指标不得因减少绕行而退化。
2. P50/P95/P99 `detour_ratio` 及单 seed 最大值。
3. 每任务额外行驶距离、走廊切换数和掉头数。
4. staging 平均/最大半径、重新分配次数和等待时间。
5. physical escape、replan、emergency brake 和 planned wait 数量。
6. 完成率、吞吐、平均延误和最差机器人延误。
7. 共享 dock、货架夹道、迎面、十字交叉和三机器人链式冲突的专项场景。

验收时不应只看平均路程，应同时限制单 seed 和单机器人的最大绕行，以免总体均值掩盖局部极端行为。
