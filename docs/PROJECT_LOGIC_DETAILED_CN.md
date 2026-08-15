# 智能工厂多机器人项目逻辑详解

> 本文依据当前工作区源码整理，重点描述“代码实际上如何运行”，而不是仅复述项目提案。最新核对日期：2026-08-15。

## 1. 项目定位

本项目是在 Webots R2023b+ 中运行的智能工厂多机器人运输与调度仿真。场景最多包含 8 台差速驱动机器人、6 个工位、8 个货架取放点和 2 个充电站。任务以随机过程到达，调度器决定“哪台空闲机器人执行哪个任务”，路径系统决定“如何安全到达”，机器人控制器执行最终运动。

系统采用两层控制：

```text
任务生成 -> 中央调度 -> 全局路径/时空协调 -> Emitter 下发路径
                                               |
Webots 场景 <- 轮速执行 <- 本地 DWA/航点控制 <- Robot Controller
     |                                         |
     +---- Supervisor 读取位姿 / Receiver 状态回报 ----+
```

中央 Supervisor 掌握业务状态、任务、全局路径和指标；每台机器人掌握传感器、轮速、本地避障与航点进度。二者并不是两个互斥的状态源：位姿可由 Supervisor API 和机器人 GPS 回报更新，行驶时的电量以机器人回报为主，停靠充电后的电量则由 Supervisor 模型维护。

## 2. 目录与职责

| 路径 | 职责 |
|---|---|
| `worlds/smart_factory.wbt` | Webots 世界、工位/货架/充电站、8 台物理机器人和中央 Supervisor |
| `protos/TurtleBot3Waffle.proto` | TurtleBot3 Waffle 的可复用模型定义 |
| `controllers/factory_supervisor/factory_supervisor.py` | 系统总入口、主循环、任务生命周期、充电、恢复与通信 |
| `controllers/factory_supervisor/config.py` | 场景几何、图节点、状态、阈值、实验和 RL 参数 |
| `controllers/factory_supervisor/task_generator.py` | 泊松到达任务及任务统计 |
| `controllers/factory_supervisor/schedulers.py` | 基线、组合优化和 PPO 调度器及统一契约 |
| `controllers/factory_supervisor/rl_environment.py` | DQN/SARSA 的抽象调度环境、状态、动作掩码和奖励 |
| `controllers/factory_supervisor/rl_agents.py` | NumPy 实现的 SARSA、DQN、经验回放和模型持久化 |
| `controllers/factory_supervisor/rl_schedulers.py` | DQN/SARSA 推理适配器与安全回退包装器 |
| `controllers/factory_supervisor/grid_planner.py` | 0.25 m 栅格、障碍膨胀、八邻域 A*、平滑与抽样 |
| `controllers/factory_supervisor/motion_coordinator.py` | 图 A*、栅格协同规划、预留、冲突与死锁处理 |
| `controllers/factory_supervisor/cbs_planner.py` | 空间—时间 A*、CBS 和持续任务 LifelongPlanner |
| `controllers/factory_supervisor/collision_safety.py` | 制动距离、CPA/TTC 与运动风险判定 |
| `controllers/factory_supervisor/safety_coordination.py` | 资源时间窗预留及优先级排序 |
| `controllers/factory_supervisor/startup_gate.py` | ROBOT_READY 集合校验与启动门控 |
| `controllers/factory_supervisor/metrics_collector.py` | 步进、任务、调度延迟和安全事件统计，输出 JSON |
| `controllers/robot_controller/robot_controller.py` | 单机器人设备初始化、通信、航点跟踪、DWA、同伴避碰和电量上报 |
| `controllers/robot_controller/sensor_diagnostics.py` | 启动时传感器有效性检查 |
| `scripts/run_experiments.py` | 启动无界面 Webots；无 Webots 时运行简化独立仿真 |
| `scripts/train_scheduler.py` / `evaluate_scheduler.py` | DQN/SARSA 抽象环境训练与冻结模型评估 |
| `scripts/train_ppo.py` | NumPy PPO 训练 |
| `scripts/run_*.ps1` | Windows 下的训练、全算法批处理和 Webots 自动评估入口 |

## 3. 世界与场景配置

Webots 使用 ENU 坐标：`x` 为东西方向，`y` 为南北方向，`z` 为高度；所有业务路径都使用地面二维坐标 `(x, y)`。世界基本步长为 16 ms。

核心地点如下：

- 工位：WS1～WS3 位于北侧接近点 `y=3.5`，WS4～WS6 位于南侧 `y=-3.5`。
- 存储点：S1～S4 位于货架北侧 `y=1.5`，S5～S8 位于南侧 `y=-1.5`。
- 充电站：CS1 `(-8.5, 0)`，CS2 `(8.5, 0)`。
- `WAYPOINTS` 和 `GRAPH_EDGES` 定义离散走廊图；每个业务地点映射到专用叶子节点 `DOCK_*`，避免其他机器人把装卸点当作普通通道穿越。
- `OBSTACLE_BOXES` 保存按机器人半径膨胀的货架和工位包围盒；当前 `CROSSING_WAYPOINTS` 为空，表示不允许机器人从货架间窄缝穿越。

三个实验场景共用同一个世界模板：

| 场景 | 机器人 | 平均任务间隔 | 负载含义 |
|---|---:|---:|---|
| A | 3 | 30 s | 小车队、低到达率 |
| B | 5 | 15 s | 中等车队和负载 |
| C | 8 | 8 s | 完整车队、高到达率 |

启动 Scenario A/B 时，Supervisor 会从场景树中真实删除多余的 `ROBOT_n` 节点，而不是仅隐藏或停用它们，随后再次计数并在数量不一致时直接报错。

## 4. 启动过程

1. Webots 加载世界，8 个机器人节点使用 `robot_controller`，中央节点使用 `factory_supervisor`；世界默认向 Supervisor 传入场景 `C`。
2. `FactorySupervisor.__init__()` 读取场景、调度器、随机种子和模型路径，创建任务生成器、运动协调器、调度器、指标收集器及机器人镜像状态。
3. 机器人控制器初始化电机、GPS、Compass、LiDAR、IMU、Emitter 和 Receiver，并运行传感器诊断。
4. 每台机器人发送 `ROBOT_READY`，包括机器人 ID、名称、成功标志和可选错误。
5. `StartupGate` 验证 ID 是否属于当前场景、是否重复、名称是否等于 `robot_<id>`。只有预期机器人全部成功后才置 `system_ready=True`。
6. 30 秒启动超时只是失败保护，不会把“部分机器人已就绪”当作可运行状态；当前配置禁止部分启动。

因此，任务可以在系统准备期间生成，但 `_assign_tasks()` 必须等系统就绪后才会真正派发。

## 5. 主循环的真实执行顺序

`FactorySupervisor.run()` 每 16 ms 执行一次，顺序是：

1. 推进 Webots；批处理模式达到 `SIM_DURATION` 时结束，GUI 直接打开时默认无限运行。
2. 通过 Supervisor 场景节点读取每台机器人的真实位姿和朝向。
3. 每一步向所有机器人广播其他机器人的位置、速度、采样时间、序列号和路径版本。
4. 接收机器人 READY、状态、到达、紧急制动和重规划请求。
5. 派发因时空冲突而延迟的路径。
6. 更新速度、里程、空闲/活动时间和电池状态。
7. 每约 1.5 s 推进一次持续规划器的离散时钟。
8. 周期性执行空闲机器人迁移、主动轨迹冲突扫描、停滞重规划与物理后退恢复；非物理节点迁移仅在显式诊断开关下启用。
9. 可选执行遗留互锁恢复和运行时 RHCR/CBS；二者默认关闭。
10. 由 `TaskGenerator` 生成新任务。
11. 调度并提交任务。
12. 每 5 s 执行图层死锁检测；每 `LOG_INTERVAL=100` 步记录一次指标。

批处理只有正常达到仿真时长才保存结果。若 Webots 被关闭、重置或超时导致 `step()` 提前返回 `-1`，本次短运行不会被写成有效实验结果。

## 6. 任务生成与生命周期

`TaskGenerator` 使用指数分布采样任务间隔，即泊松到达过程。场景配置允许在 `t=0` 立即生成首个任务。任务类型概率为：存储点到工位 40%，工位到存储点 35%，工位到工位 25%；起点与终点不会相同。优先级从 `1.0～1.5` 采样。

任务对象 `TransportTask` 保存：ID、起止地点及坐标、到达/分配/取货/完成时间、优先级、已分配机器人和状态，并派生等待时间、执行时间和总完成时间。

主状态链为：

```text
Task:  PENDING -> ASSIGNED -> IN_PROGRESS -> COMPLETED
Robot: IDLE -> EN_ROUTE_PICKUP -> CARRYING/EN_ROUTE_DELIVERY -> IDLE
```

详细提交逻辑：

1. 调度器只在 `PENDING` 任务和满足条件的空闲机器人之间输出 Assignment。
2. Supervisor 先刷新真实位置，再规划去取货点的路径。
3. 只有路径存在时才临时提交任务、机器人状态和航点。
4. 只有导航消息成功发送后，提交才成为最终结果；发送失败会完整回滚任务和机器人状态，并对该“机器人—任务”组合设置 5 秒失败 TTL。
5. 到达取货点后，任务进入 `IN_PROGRESS`，立即规划送货段并派发。
6. 到达送货点后，记录完成指标，释放该机器人全部动态路径预留，将机器人置为空闲。
7. 空闲机器人可被新任务立即链式认领；若没有新任务，则稍后移动到最近的空闲 `REST_NODE`，避免堵住装卸口。

机器人上报“到达目标”时，Supervisor 还会用真实位置做距离复核，过滤远离目标的误报。

## 7. 调度层

### 7.1 统一契约

所有调度器继承 `BaseScheduler`，新接口 `assign()` 返回 `SchedulerResult`：Assignment 列表、目标值、计算耗时、可行性、算法名和诊断信息。旧接口 `assign_task()` 仍保留以兼容单次分配。

`SchedulingContext` 可提供当前时间、拥堵图、禁用的机器人—任务对和真实路径成本函数。统一校验会拒绝：不存在或非空闲机器人、非待处理任务、低电量机器人、重复机器人/任务以及被临时禁用的组合。

Supervisor 一次调度调用可循环提交多个任务，但每次只取结果中的第一个 Assignment，提交后重新构建状态，避免批量结果在前一项提交后失效。主调度器异常、超时、无效或不可行时，会依次尝试 Hungarian、Greedy、NearestNeighbour 安全调度器并记录回退次数。

### 7.2 可用算法

| 名称 | 核心逻辑 |
|---|---|
| FCFS | 最早到达任务优先，再选择首个可用机器人 |
| NearestNeighbour | 选择机器人到取货点距离最短的组合 |
| RoundRobin | 轮转机器人，保持简单公平性 |
| Greedy | 在统一成本矩阵中反复选当前最低成本可行对 |
| Random | 在合法候选中按固定随机种子选择 |
| Hungarian | 用线性指派求全局一对一最小成本匹配 |
| Auction | 带价格迭代、时间预算和迭代上限的拍卖匹配 |
| GA | 对任务排列进行限时遗传搜索 |
| SA | 对任务排列做限时模拟退火搜索 |
| PPO_RL | NumPy 策略/价值网络推理，要求兼容 `.npz` 模型 |
| DQN | 动作掩码下的 Q 网络推理，要求兼容 `.pkl` 模型 |
| SARSA | 离散化状态上的表格策略，要求兼容 `.json` 模型 |

成本综合机器人到取货点、取货到送货距离、任务等待/优先级、电量和拥堵等上下文。RL 模型不是“缺模型也随机运行”：模型缺失或校验失败时，根据创建参数回退到确定性安全调度器；DQN/SARSA 运行期还有 20 ms 推理阈值和连续失败保护，并回退到 Hungarian。

### 7.3 RL 状态与动作

DQN/SARSA 使用 `SchedulingEnvironment`：固定机器人槽和任务槽，把机器人位置、状态、电量、任务候选特征等编码成定长观测。动作表示 `(robot_slot, task_slot)`，另有 no-op 动作；动作掩码保证只有空闲且电量足够的机器人与待处理任务组合可选。

奖励包含有效分配、任务完成和优先级正奖励，以及无效动作、无意义 no-op、等待、距离和碰撞惩罚。死锁本身当前权重为 0，因为它被视为可恢复运行事件，真正碰撞才是严重安全失败。

PPO 使用 `schedulers.py` 中的 NumPy 前馈策略/价值网络和 `train_ppo.py` 的 GAE、裁剪目标、熵项及价值损失训练流程。

## 8. 全局路径与多机器人协调

系统同时保留“走廊图”和“自由空间栅格”两套表示：

- 走廊图 A*：适合节点级最短路、位置映射、CBS 和死锁依赖分析。
- 栅格 A*：当前实际派发优先使用，分辨率 0.25 m，范围约 `x=[-9,9]`、`y=[-5.5,5.5]`，允许八方向移动。

### 8.1 单机器人栅格路径

`OccupancyGrid` 把物理障碍和不活动机器人写入栅格，并按机器人半径与安全裕量膨胀。`GridAStar` 将起终点吸附到自由单元，使用 octile 启发式搜索，禁止对角穿角；得到的路径可做视线平滑。`subsample_path()` 降低航点密度，但保留必要转折和无碰撞视线。

### 8.2 协同栅格规划

`plan_grid_lifelong()` 是正常业务路径的优先入口：

1. 释放机器人旧栅格预留。
2. 收集其他机器人已预留的路径单元并膨胀 2 格。
3. 首先把同伴路径临时设为硬障碍，尝试空间绕行。
4. 若走廊被封死，改成高代价区域重试，倾向更长但分离的路线。
5. 若仍失败，允许普通最短路径，但用发车延迟串行化共享通道。
6. 按 0.5 s 间隔在 0～20 s 内搜索可接受延迟，并检查节点占用和反向边交换冲突。
7. 若仅靠延迟不能解决，运行有时间和展开数预算的空间—时间栅格绕行。
8. 保存坐标路径和栅格预留，再施加车道分离、冲突延迟和航点压缩。

若该入口返回 `None`，Supervisor 回退到 `plan_path_for_robot()` 的较传统路径方案。规划失败不会让任务进入已分配状态。

### 8.3 CBS 与持续预留

`cbs_planner.py` 包含：

- `SpaceTimeAStar`：在顶点约束和边约束下搜索 `(node,time)`。
- `CBSPlanner`：检测最早的顶点冲突或边交换冲突，分裂约束树并只重规划受影响机器人。
- `LifelongPlanner`：为在线任务维护绝对时间预留，支持单机器人增量规划、释放、静态占位、时钟推进和查询。

运行时全局 RHCR 每 5 秒生成多机器人 CBS 候选，但默认关闭，因为当前候选没有事务性地下发，且 8 机器人时同步搜索可能阻塞 Webots 控制循环。它更接近实验功能，不是默认安全链路。

### 8.4 多级冲突与死锁恢复

默认安全链路由多个层次叠加：

1. 规划期：障碍膨胀、同伴路径绕行、时空预留和延迟派发。
2. 运行期主动扫描：预测未来轨迹，必要时保持、重规划或改变优先级。
3. Coordinator 死锁监视：根据长期无进展机器人形成阻塞关系，低优先级机器人让行并重规划。
4. 进度监视：持续约 3 秒无有效进展时触发重规划。
5. 长停滞恢复：达到 `STALL_RELOCATION_TIMEOUT=5s` 后选择低通行权机器人，以经过栅格规划、预约和版本化派发的后退路径释放通道。直接移动 Webots 节点默认关闭，仅可通过 `SMART_FACTORY_ENABLE_NONPHYSICAL_RECOVERY=1` 用于诊断。
6. 本地控制器：DWA、侧移、预测性减速和紧急停车是最后执行层保护。

遗留的 1.5 秒互锁恢复默认关闭，避免把“等待目标到达数据包”的机器人误判为死锁。

## 9. 单机器人控制器

`RobotController` 负责设备和循环，`WaypointNavigator` 负责运动决策。

每个机器人初始化：左右轮速度控制、GPS、Compass、LiDAR、IMU、通信设备。每步读取位置/朝向/LiDAR，接收命令，计算线速度和角速度，再转换为左右轮角速度并限幅。

支持的 Supervisor 下行命令主要包括：

| 命令 | 行为 |
|---|---|
| `navigate` | 接收完整航点列表和 `path_version`，开始/替换导航 |
| `hold` | 保留路径但在指定仿真时间前停车 |
| `stop` | 停止并清除当前执行意图 |
| `charge` | 兼容的单目标充电站导航命令 |
| `battery_swap` | 同步 Supervisor 完成换电后的电量并停车 |
| `peer_positions` | 更新其他机器人位置、速度和样本版本 |

机器人上行消息包含 `ROBOT_READY` 和周期状态：位置、朝向、电量、是否到达航点/最终目标、是否紧急制动、是否请求重规划等。`path_version` 用于降低旧命令或旧状态污染新路径的风险。

### 9.1 航点控制与 DWA

正常情况下先对准目标方向，再按距离和朝向误差生成速度。接近障碍或处于局部规避状态时，DWA 在允许的线速度/角速度窗口内采样，向前预测 1.5 s，并按以下因素评分：

- 终点朝向；
- 与障碍物的最小距离；
- 前进速度；
- 侧移模式中的横向偏置。

候选轨迹若进入危险距离直接淘汰。LiDAR 按前、左、右扇区扫描，近距离障碍会触发减速或停车。

### 9.2 同伴机器人避碰

Supervisor 每个仿真步广播同伴状态。机器人根据样本新鲜度、相对速度、前向投影和路径冲突计算速度因子：约 1.0 m 开始减速，约 0.55 m 停车，0.45 m 为紧急停车保护。系统还检查前方路径段而不只检查当前中心距，以便更早发现对向和交叉冲突。

对向相遇时可进入侧移模式：在左右空间中选更安全一侧，产生约 0.75 m 横向偏置；前方恢复到约 1.8 m 或达到 4 s 超时后退出。码头/货架附近有专门区域约束，避免侧移把机器人带入货架。

### 9.3 电池职责

机器人运动时按 `100% / 1800s` 线性耗电，并把电量回报 Supervisor。Supervisor 负责高层阈值和充电：

```text
IDLE/任务中低电量 -> RETURNING_TO_CHARGE -> CHARGING -> IDLE
```

- 低于 25%：空闲机器人送往最近充电站，且通常不再接新任务。
- 任务执行中低于 15%：任务可被重新排队，机器人立即转去充电。
- 前往充电站仍耗电；只有到站进入 `CHARGING` 才补电。
- 到站后模拟约 5 秒快速换电/补能，电量达到 95%～100% 后释放充电位置并回到空闲状态。

充电点也参与路径和静态预留，避免多车同时占据同一站位。

## 10. 通信一致性与容错

通信使用 Webots channel 1 的 JSON 消息。所有下行包都有 `target_robot`，机器人只执行发给自己的命令。关键一致性措施包括：

- READY 启动屏障，避免机器人设备尚未就绪就派发任务；
- 导航路径版本递增，延迟路径和 hold 绑定版本；
- 任务采用“先规划、再临时提交、发送成功后确认”的事务式顺序；
- 到达事件检查布尔值本身，而不是只检查字段存在；
- 到达最终目标进行真实距离复核；
- 充电状态下拒绝机器人侧下降电量覆盖 Supervisor 的上升电量；
- 规划失败组合短期熔断，防止每一帧重复选择同一失败组合。

## 11. 指标与输出

`MetricsCollector` 保存周期快照、任务完成明细、冲突/死锁和安全事件，并生成 `results/experiment_<场景>_<调度器>_<时间>.json`。主要指标包括：

- 任务生成数、完成数、完成率和每分钟吞吐量；
- 平均等待、执行、完成时间；
- 每台机器人完成量、里程、利用率和末端电量；
- 冲突、死锁、重规划；
- 最小机器人间距、间距违规和碰撞类安全事件；
- 调度平均/P95 延迟、无效输出和安全回退次数。

`run_all_algorithms_headless.ps1` 会训练 RL 模型，按 A/B/C 场景运行各调度器，将新产生的 JSON 聚合为 CSV，并按“安全违规少、完成率高、吞吐高、等待低”的顺序选出每个场景的最佳算法。

## 12. 运行方式

安装 Python 依赖：

```powershell
python -m pip install -r requirements.txt
```

交互运行：用 Webots 打开 `worlds/smart_factory.wbt`。默认场景 C、FCFS，且不会因 1800 s 配置自动关闭。

单组批处理：

```powershell
python scripts/run_experiments.py --scenario A --scheduler Hungarian --seeds 1 --webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
```

环境变量/参数入口：

| 变量 | 含义 |
|---|---|
| `SCENARIO` | A、B 或 C；批处理可覆盖世界中的默认 `controllerArgs` |
| `SCHEDULER` | 调度器名称 |
| `SEED` | 随机种子 |
| `MODEL_PATH` | PPO/DQN/SARSA 模型路径 |
| `SMART_FACTORY_SIM_DURATION` | 批处理仿真时长，默认 1800 s |
| `SMART_FACTORY_AUTO_STOP` | 是否达到时长后自动结束并保存 |
| `SMART_FACTORY_ENABLE_RHCR` | 是否启用实验性运行时 RHCR |
| `SMART_FACTORY_ENABLE_LEGACY_INTERLOCK` | 是否启用遗留互锁恢复 |
| `SMART_FACTORY_ENABLE_NONPHYSICAL_RECOVERY` | 是否允许诊断用非物理节点迁移；正式实验应保持关闭 |

训练和评估示例：

```powershell
python scripts/train_scheduler.py --algorithm dqn --episodes 200 --checkpoint-dir results/models/dqn
python scripts/evaluate_scheduler.py --algorithm dqn --checkpoint results/models/dqn/best_validation.pkl --split test
python scripts/train_ppo.py --episodes 500 --save-dir results/models/ppo
```

如果找不到 Webots，`run_experiments.py` 会进入独立简化仿真。该模式复现任务、调度、路径长度、移动和充电状态机，适合算法冒烟测试，但没有 Webots 刚体动力学、真实 LiDAR/DWA 或实际机器人间接触，因此不能替代安全性和运动学验证。

## 13. 扩展指南

### 新增调度器

1. 继承 `BaseScheduler` 并实现 `assign()`；输出统一 `SchedulerResult`。
2. 使用 `SchedulingContext` 和统一可行性规则，不绕过电量、状态及失败组合检查。
3. 在 `create_scheduler()` 注册名称。
4. 添加到批处理脚本，并分别测试空任务、无空闲机器人、低电量、不可达路径、超时和无效输出。

### 修改布局

必须同步检查 `.wbt` 实体、`WORKSTATIONS`/`STORAGE_AREAS`/`CHARGING_STATIONS`、`WAYPOINTS`、`GRAPH_EDGES`、`LOCATION_TO_NODE`、障碍包围盒和栅格边界。仅移动视觉模型或仅改任务坐标都会造成“目标看起来正确但路径碰撞/不可达”。改后至少运行布局、全路径、间隙、首跳和横向净空验证脚本。

### 修改避碰阈值

机器人半径、栅格安全裕量、同伴停车距离、路径预留膨胀和 DWA 障碍阈值是耦合参数。单独增大某一阈值可能让窄走廊彻底不可达；单独减小则可能造成规划可达但实体擦碰。应同时验证静态净空、双车对向、交叉口和停靠点。

## 14. 当前实现中需要特别注意的事实

- `RobotState.WAITING` 已进入充电主链路：低电量机器人在两个充电站都暂时不可达或充电导航命令发送失败时进入该状态，约 1 秒后重试；它不是普通任务排队状态，也不会被调度器选中。
- 任务提交具有事务边界：只有取货路径存在且导航命令成功发送后，`ASSIGNED` 状态才最终成立；发送失败会回滚任务、机器人、航点和延迟派发状态，并为失败的“机器人—任务”组合设置临时 TTL。
- 调度器一次可以返回多个分配，但 Supervisor 每轮只提交第一项，随后基于最新状态重新构造候选集和成本，防止前一项提交使剩余批量结果失效。
- 运行时 RHCR 默认关闭且候选不会事务式下发，不应在论文或报告中描述为默认在线控制器；默认在线安全链路是栅格预约、延迟发车、主动冲突扫描、进度重规划和死锁恢复。
- 充电状态以物理位置为准：空路径只有在机器人确实位于充电站附近时才代表“已经到达”，普通规划失败不会触发远程充电；`RETURNING_TO_CHARGE` 途中仍消耗电量。
- 栅格重规划采用候选预约事务：失败或派发失败恢复旧预约；只有版本化导航命令成功发送后才提交候选。延迟 hold 保留回滚点，实际导航派发后才最终提交。
- 空间预约由最终下发折线重新栅格化，并包含机器人足迹相邻单元；时间预约使用绝对仿真秒、实际直线/对角距离和转弯裕量，并按真实进度每 0.5 秒刷新。
- `pending_waypoints`、Supervisor hold、控制器 pause 和紧急制动均属于合法静止，不累计普通停滞时间；主动轨迹预测会将这些阶段模拟为原地等待。
- Supervisor 配置的最大线速度为 0.26 m/s，而机器人本地控制器实际限制为 0.22 m/s；时空预计和实验解释应采用执行端 0.22 m/s，或统一两处参数。
- 当前仓库没有独立的 `tests/` 自动化测试套件，也没有旧文档曾提到的 `scripts/verify_*.py`；现有验证主要依赖训练/评估脚本、批处理 PowerShell 入口以及 Webots 实际运行结果。
- `README.md` 目前只有项目标题，项目架构、运行约束和扩展边界应以本文及其他 `docs/` 专题文档为准。

## 15. 一句话理解各层边界

调度器决定“派谁做什么”，运动协调器决定“从全局怎样走且何时出发”，机器人控制器决定“这一帧左右轮怎么转”，Supervisor 状态机负责把取货、送货、空闲、充电、恢复和指标串成一条可复现实验链路。
## 预防性联合运动规划（2026-08-15 更新）

Supervisor 每 0.25 秒预测未来 10 秒轨迹，并将两两预测冲突构造成冲突图。两机器人冲突采用确定性的轻量优先级让行；三台及以上的冲突连通分量进入联合速度搜索。联合候选会重新生成组内全部轨迹，并与组内、组外所有活动机器人复验，只有安全候选才能提交。

联合调速最低保持 0.85 倍额定速度，避免用近似停车代替规划；方案至少稳定 3 秒、最多持续 8 秒。发送中途失败会回滚已经改变的成员，机器人状态遥测回报实际速度比例。无法用速度解决且已经迫近的冲突继续使用经过最终几何、空间预约和时间预约校验的路径重规划。紧急后退和停滞恢复仍保留为最后安全兜底，不属于预规划成功。

详细审查、失败实验与最终 Webots 指标见 `docs/JOINT_PROACTIVE_PLANNING_WORKFLOW_CN.md`。
