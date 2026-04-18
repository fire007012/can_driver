# can_driver 对齐 Eyou_Canopen_Master 的重构报告

## 1. 目的

本文档用于回答两个问题：

1. `can_driver` 是否还需要“再做一套生命周期状态机”？
2. 如果目标是向 `Eyou_Canopen_Master` 靠拢，`can_driver` 应该具体重构什么？

结论先行：

- `can_driver` 已经有一套系统级生命周期主链，不能推倒重来。
- 这次重构的重点不是再发明一套顶层状态机，而是在现有生命周期主链下面补齐“单总线运行时 + 共享状态面 + 轴级闭环状态逻辑”。
- `Eyou_Canopen_Master` 真正值得复用的，不只是 service 语义，而是“协调器只管 intent，轴逻辑根据真实反馈闭环推进”的架构。

换句话说，`can_driver` 当前缺的不是 `Configured/Standby/Armed/Running/Faulted` 这些名字，而是这些系统态下面缺少一层真正盯着电机反馈、总线新鲜度、模式回读、使能回读和故障位的运行时闭环。

## 2. 当前已有能力，应保留

对照当前 `can_driver` 代码与已有文档，以下能力已经存在，而且方向是对的：

- `OperationalCoordinator`
  - 已经负责系统级状态迁移。
  - 已具备 `init/enable/disable/halt/resume/recover/shutdown` 主链入口。
- `LifecycleServiceGateway`
  - 已经把生命周期 service 从 `CanDriverHW` 中拆出来。
  - 这和 `Eyou_Canopen_Master` 的 `ServiceGateway` 方向一致。
- `CommandGate`
  - 已经承接 hold、fresh-command latch、设备恢复后一拍保护等职责。
  - 这部分不该被删，而应该继续下沉为运行时约束。
- 统一生命周期语义
  - 当前文档已经在对齐 `Eyou_Canopen_Master` 的系统级生命周期语义。
  - 两个电机包在顶层生命周期概念上已经基本一致。

因此，这次架构调整应明确为：

- 保留现有 `OperationalCoordinator`
- 保留现有 `LifecycleServiceGateway`
- 保留现有 `CommandGate`
- 保留现有 lifecycle service 入口和总体状态集合
- 在其下补一层 `Eyou_Canopen_Master` 风格的闭环运行时

## 3. 当前真正的问题不在“没有生命周期”，而在“生命周期下面没有闭环”

### 3.1 系统级状态存在，但轴级状态判断仍然很弱

当前 `can_driver` 虽然有系统级 lifecycle mode，但其是否真的可以认为某轴“可运行”，还缺少基于真实反馈的闭环判定。至少以下条件目前没有被统一建模：

- 最近一次有效反馈时间是否超时
- 最近一次模式回读是否与目标模式一致
- 最近一次使能回读是否与目标使能一致
- fault 位是否真实清除
- 设备是否处于连续超时/降级状态
- 该轴是否只是“链路恢复”，还是“已经恢复到可进入运行态”

结果就是：

- 顶层 coordinator 可以切到某个系统态
- 但轴实际是否 ready，没有一个统一、可复用、可验证的闭环状态机去接住
- `CanDriverHW`、协议类、维护接口、定时刷新线程都可能各自做一部分判断

这会导致系统级状态“看上去合理”，但底层行为并不稳定。

### 3.2 `CanDriverHW` 仍然过重

当前 `CanDriverHW` 仍承接过多职责：

- `RobotHW` 适配
- direct topic 入口
- 协议写入
- 读反馈发布
- 设备健康判断
- 生命周期联动
- 安全保持

这导致两个直接问题：

- 一个对象里同时混着“控制接口层”和“运行时调度层”
- 后续任何可靠性修复都容易继续长回 `CanDriverHW`

### 3.3 总线写入缺乏单点所有权

和 `Eyou_Canopen_Master` 相比，`can_driver` 当前最危险的问题之一，是单个 CAN 设备上的发送职责没有被严格收口到一个运行时对象。

实际后果包括：

- 多个调用路径共享同一个 socket
- 查询、刷新、控制写入之间缺少统一调度
- 超时重试和周期刷新容易形成瞬时突发
- 平均流量不高，但瞬时排队深度会很高

这也是“平均占用不高，仍然爆缓存区”的一个核心解释。

平均总线占用低，不代表：

- 同一时刻写请求不扎堆
- PF_CAN 发送队列不会被短时打满
- 协议层不会在超时情况下触发额外 refresh/read storm

所以问题往往不是“总线带宽绝对不够”，而是“发送调度没有被串行化和节流，导致局部突发把内核 TX queue 塞满”。

### 3.4 协议对象承担了过多运行时行为

当前协议相关对象不只是“协议编解码器”，还承担了：

- transport 回调入口
- 内部线程或周期刷新
- 缓存
- 部分状态判断
- 部分恢复行为

这会把“协议格式”和“运行时控制策略”耦合在一起。协议类一旦自己开节拍、自己超时重试、自己维护部分状态，外层 coordinator 很难真正成为唯一控制面。

### 3.5 为什么这里会爆，而 Lely 不容易爆

`Eyou_Canopen_Master` 之所以稳，不是因为它“碰巧流量更低”，而是因为它在架构上天然抑制了写入风暴：

- 总线写入由事件线程/总线运行时串行拥有
- 轴逻辑不直接乱发命令，而是根据反馈与 intent 计算下一拍应输出什么
- `SharedState` 统一承接跨线程数据，而不是多个模块直接抢总线
- 协调器只改 intent，不直接把 protocol 命令散落到各处

因此即使双从站 200Hz，Lely 风格架构仍可能保持很低占用并稳定运行，因为它本质上减少了重复写、错位写和超时后的无序追发。

## 4. candump 看不到主站发帧的架构性含义

从现象上看，“`candump` 看不到主站发出的帧”和“TX queue saturated”同时出现，通常说明问题更像是：

- 帧在用户态到内核发送路径上就开始排队或被拒绝
- 或者帧尚未形成稳定可见的总线输出，就已经在本地发送路径上被堵住

在当前架构里，这和以下问题是同向的：

- 没有单一 TX owner
- 缺少 per-device 的节拍器和待发送队列
- 缺少对“无反馈/超时/未 ready 设备”的主动降载
- 缺少把“读超时”升级为“停止继续追发该轴某些请求”的闭环策略

因此，这不是单纯把 socket 改成 `O_NONBLOCK` 就彻底结束的问题。非阻塞发送是必要的止血手段，但如果发送源头仍然分散，长期仍会遇到：

- 某些帧长期被丢
- 某些查询持续追发
- 某些轴明明不健康却还在被周期轰炸

## 5. 对标 Eyou_Canopen_Master 后，can_driver 缺失的关键层

对照 `Eyou_Canopen_Master`，`can_driver` 还缺四个关键部件。

### 5.1 SharedState 或等价共享数据面

需要一个清晰的数据面对象，统一承接：

- 每轴反馈快照
- 每轴命令快照
- 每轴 intent
- 全局 fault 标志
- 命令同步序号
- 反馈新鲜度时间戳
- 设备 ready/degraded/offline 标志

这样 coordinator、轴逻辑、ROS 适配层、诊断层才不会继续通过零散成员变量互相耦合。

### 5.2 AxisRuntime / AxisLogic

需要给每个电机补一层轴级闭环逻辑，职责类似 `Eyou_Canopen_Master` 的 `AxisLogic`：

- 接收目标 intent
- 读取该轴最近反馈
- 判断当前轴状态
- 计算下一拍该发什么，而不是“想发什么就直接发”
- 在 fault、heartbeat timeout、mode mismatch、enable mismatch 时自动退回安全输出

建议至少引入以下轴级运行态：

- `Offline`
- `Seen`
- `Standby`
- `Armed`
- `Running`
- `Faulted`
- `Recovering`

这些轴级状态不是为了替代系统级 lifecycle，而是为了支撑它。

系统级 `Running` 不能只看“服务调过了没有”，而应来自“所有要求参与运行的轴都满足运行条件”的闭环结论。

### 5.3 DeviceRuntime：每个 CAN 设备只有一个 TX owner

这是本轮最关键的结构变更。

每个 CAN 设备都应该有一个唯一的 `DeviceRuntime`，统一负责：

- 持有 socket
- 统一排队和发送
- 控制发送节拍
- 分类处理周期写、查询写、恢复写
- 统计丢包、队列饱和、发送失败
- 统一接收反馈并更新时间戳

其他任何模块都不应再直接碰 socket。

所有写请求都应转换为“向 `DeviceRuntime` 提交发送意图”，再由它按固定策略发出。

### 5.4 Feedback Freshness Model

需要把“有没有反馈”从零散 if 判断升级为正式模型。

每轴至少要记录：

- `last_rx_time`
- `last_valid_state_time`
- `last_mode_match_time`
- `last_enable_match_time`
- `consecutive_timeout_count`
- `last_fault_time`

有了这套模型后，才能稳定回答：

- 这个轴是 offline、degraded，还是 ready
- 这个轴是否允许进入 `Armed`
- 这个轴是否必须降级回 `Standby` 或 `Faulted`
- 某种周期查询是否应该降频甚至暂停

## 6. 建议的目标架构

建议将 `can_driver` 调整为如下分层：

`LifecycleServiceGateway`
-> `OperationalCoordinator`
-> `SharedDriverState`
-> `AxisRuntime[]`
-> `DeviceRuntime[]`
-> `SocketCanController`

同时：

- `CanDriverHW` 只做 ROS / `RobotHW` 适配
- coordinator 只写系统 intent，不直接发协议帧
- `AxisRuntime` 根据 feedback + intent 生成动作请求
- `DeviceRuntime` 决定何时、以什么顺序真正下发 CAN 帧

更具体地说：

### 6.1 保留现有生命周期主链，但改变它的职责边界

现有生命周期对象应继续存在，但其职责改成：

- `LifecycleServiceGateway`
  - service 入口
  - 参数检查
  - 响应映射
- `OperationalCoordinator`
  - 系统级状态迁移
  - 系统 intent 生成
  - 故障聚合
  - 对轴运行条件进行总括判断
- `CommandGate`
  - 作为运行态保护器继续保留
  - 负责 fresh-command 与 hold 约束

也就是说，lifecycle 保留，但不再直接等同于“底层轴已经准备好了”。

### 6.2 新增 SharedDriverState

建议新增一个共享状态对象，包含：

- `AxisFeedbackState[]`
- `AxisCommandState[]`
- `AxisIntent[]`
- `DeviceHealthState[]`
- 全局 fault / degraded 标志
- command epoch / sync sequence

它是线程之间唯一允许共享的主数据面。

### 6.3 新增 AxisRuntime

每个轴维护一个轻量闭环运行时，负责：

- 从 `SharedDriverState` 取 feedback
- 根据系统 intent 和轴实际状态决定动作
- 把“想 enable / 想 run / 想 recover”翻译成可执行的轴级请求
- 输出给 `DeviceRuntime` 的是“动作请求”，不是直接 socket 写

### 6.4 新增 DeviceRuntime

每个 CAN 设备单独运行一个 runtime，负责：

- socket 的唯一所有权
- 发送队列管理
- 不同优先级请求的调度
- 非阻塞发送与丢弃计数
- 查询节流与退避
- 接收反馈后回写 `SharedDriverState`

建议把发送请求分成至少三类：

- `Control`
  - 运行必须的控制写
- `Recover`
  - fault reset、重新使能、恢复路径请求
- `Query`
  - 诊断和刷新读取

调度原则建议为：

- `Control` 高于 `Recover`
- `Recover` 高于 `Query`
- 队列饱和时最先丢 `Query`
- 明显 offline 的轴降低 `Query` 频率

这样可以直接缓解“低平均流量却爆队列”的现象。

## 7. 为什么这套方案更适合当前问题

### 7.1 它解释了“平均负载不高，但还是爆”

引入 `DeviceRuntime` 后，可以把问题从“总线平均占用”转成“每拍到底有多少帧被请求发送”。

只要能统计下面这些量，就能看清真实问题：

- 每周期生成的控制帧数
- 每周期生成的查询帧数
- 被丢弃的查询帧数
- 发送队列最大深度
- 单轴连续 timeout 次数
- 单轴降频前后的发送频率

这比只看 busload 更接近根因。

### 7.2 它解释了“为什么 Lely 没那么容易出事”

Lely 风格架构本质上做了三件事：

- 把“状态判断”建立在反馈上
- 把“动作生成”建立在 intent 上
- 把“实际发送”收口到唯一总线所有者

`can_driver` 目前最缺的正是第三点，而第一点和第二点也还不够闭环。

### 7.3 它允许先止血，再渐进重构

这套方案允许分阶段推进：

- 第一阶段先加统计和退避，先止住 TX queue 爆满
- 第二阶段收口单 TX owner
- 第三阶段再把轴级状态闭环接上

不用一次推翻现有代码。

## 8. 建议迁移路线

### Phase 1：观测与保护先行

目标：先把问题看清楚，并降低继续爆队列的概率。

本阶段建议：

- 给每个 CAN 设备加 TX/RX 统计
- 记录：
  - `tx_ok`
  - `tx_eagain`
  - `tx_dropped_query`
  - `tx_dropped_control`
  - `rx_timeout`
  - `last_rx_time`
- 给查询类请求加基础节流
- 明确“队列满时先丢查询，不丢关键控制帧”

说明：

这一步不改变总体架构，但为后续重构提供证据。

### Phase 2：引入 DeviceRuntime，收口单一 TX owner

目标：任何写 socket 的路径都只能经过 `DeviceRuntime`。

本阶段建议：

- `SocketCanController` 只暴露给 `DeviceRuntime`
- 协议对象不再直接调用发送
- `CanDriverHW` 不再直接发送帧
- 把读写请求统一改成“提交发送请求”

完成后，至少可以消除：

- 多源争抢 socket
- 局部突发无序写
- 不同路径各自 retry 的混乱

### Phase 3：引入 SharedDriverState

目标：把跨线程共享状态从零散成员变量收敛到统一数据面。

本阶段建议：

- 新建每轴 feedback/command/intent 结构
- 新建 per-device 健康结构
- 引入命令同步序号和反馈时间戳
- coordinator、`CanDriverHW`、轴运行时都只通过它交换状态

### Phase 4：引入 AxisRuntime / AxisLogic

目标：让轴状态真正闭环，而不是只看 service 是否调用成功。

本阶段建议：

- 每轴根据真实反馈推进状态
- `Running` 的前提变为：
  - 反馈新鲜
  - 模式回读匹配
  - 使能回读匹配
  - fault 清除
  - 设备未超时降级
- 恢复路径必须先回到 `Standby`，再重新进入 `Armed/Running`

### Phase 5：瘦身 CanDriverHW

目标：把 `CanDriverHW` 收口成真正的适配层。

完成后它应只负责：

- ROS 参数和 joint 配置
- `read()/write()` 与 `controller_manager` 对接
- 状态发布
- direct topic/兼容维护接口

不再负责：

- 总线调度
- 轴状态闭环
- 恢复策略细节
- 查询节拍决策

### Phase 6：协议类去运行时化

目标：协议类只负责协议，不再偷偷承担调度。

本阶段建议：

- 去掉协议对象内部分散的 refresh 线程或隐式周期行为
- 协议层只保留编解码与动作定义
- 由 `AxisRuntime`/`DeviceRuntime` 决定什么时候发

## 9. 推荐的状态职责划分

为避免再次做成“两套互相打架的状态机”，建议明确区分三层状态。

### 9.1 系统级生命周期

保留现有：

- `Inactive`
- `Configured`
- `Standby`
- `Armed`
- `Running`
- `Faulted`
- `Recovering`
- `ShuttingDown`

这一层只回答：

- 系统是否已初始化
- 系统是否允许进入运行
- 系统是否处于全局故障恢复流程

### 9.2 轴级运行状态

新增或正式化：

- `Offline`
- `Seen`
- `Standby`
- `Armed`
- `Running`
- `Faulted`
- `Recovering`

这一层回答：

- 某轴是否真的在线
- 某轴是否已经 ready
- 某轴是否具备进入运行态的真实条件

### 9.3 设备级链路健康状态

建议显式建模：

- `Healthy`
- `Degraded`
- `Offline`
- `Recovering`

这一层直接服务于总线调度与查询退避。

## 10. 验收标准

若按本文路线完成重构，建议以以下现象作为验收标准：

- 单从站或双从站在线时，长时间运行不再出现持续 `TX queue saturated`
- 从站异常、超时或短暂掉线时，系统进入降级，但主循环不被阻塞
- 队列饱和时优先丢查询帧，而不是把控制线程拖死
- `candump` 可稳定观察到主站关键输出帧
- lifecycle mode 与轴真实运行状态不再脱节
- 故障恢复后必须重新 fresh command，对旧命令天然免疫

## 11. 最终建议

建议将 `can_driver` 的后续重构定义为：

“保留当前生命周期主链，补齐 `Eyou_Canopen_Master` 风格的闭环运行时架构。”

更具体地说，下一轮工作的优先级应当是：

1. 先补统计、节流、退避，继续验证当前 TX 爆满根因。
2. 再收口单总线 `DeviceRuntime`，建立唯一 TX owner。
3. 然后补 `SharedDriverState` 与 `AxisRuntime`，把轴状态闭环接上。
4. 最后再把 `CanDriverHW` 瘦身成纯适配层。

这样做的好处是：

- 尊重 `can_driver` 已经完成的生命周期主链工作
- 同时真正吸收 `Eyou_Canopen_Master` 最有价值的部分
- 不会把“状态机重构”误做成“只改 service 名字和状态名”

本质上，`can_driver` 下一步最该学的不是“再建一个 lifecycle”，而是“像 `Eyou_Canopen_Master` 一样，让生命周期下面真的有一个可靠的闭环运行时”。

## 12. 按 Commit 实施计划

为避免本轮重构再次失控，建议严格按 commit 切分推进，每个 commit 都只解决一个层次的问题。

### Commit 1 ✅ `27b5c2c`

`feat(can_driver): add per-device tx/rx stats and query backoff`

目标：

- 先把当前爆队列问题看清楚
- 先止血，不大改架构

内容：

- 为每个 CAN 设备增加 TX/RX 统计
- 记录 `tx_ok`、`tx_eagain`、`tx_link_down`、`tx_drop_query`、`rx_ok`、`rx_timeout`
- 记录 `last_rx_time`
- 给查询类请求增加基础退避与降频
- 队列饱和时优先丢查询帧

验收：

- 运行中可看到更明确的设备级统计日志
- 出现读超时时，不再继续无节制追发查询帧

### Commit 2 ✅ `b0911e0`

`refactor(can_driver): funnel all socketcan writes through a single tx entry`

目标：

- 先统一发送入口
- 为后续 `DeviceRuntime` 做结构准备

内容：

- 引入 `CanTxDispatcher` 抽象接口和 `DirectCanTxDispatcher` 直通实现
- 收口所有 socket 发送路径
- 协议类（`EyouCan`、`MtCan`）不再直接调用 `canController->send()`，统一走 `submitTx()`
- 每个发送点按语义标注 `Category`（Control / Recover / Config / Query）

验收：

- 仓内所有发送路径都可以追到同一个 TX 入口

### Commit 3 ✅ `028fbbf`

`refactor(can_driver): introduce device runtime with prioritized nonblocking tx`

目标：

- 建立每设备唯一 TX owner

内容：

- 引入 `DeviceRuntime`，实现 `CanTxDispatcher` 接口
- 按 `Control` > `Recover` > `Config` > `Query` 四级优先级调度
- 单 worker 线程串行发送，消灭多路径争抢 socket
- 队列满时按类别丢弃并计数
- `DeviceManager` 中把 `DirectCanTxDispatcher` 全部替换为 `DeviceRuntime`
- 修正 shutdown 顺序：先停 `DeviceRuntime`，再关 transport

验收：

- 不同业务路径不再直接争抢同一个 socket
- `TX queue saturated` 不再由零散路径分别触发

已知短板（在 Commit 4 中解决）：

- `transport_->send()` 返回值未被检查，EAGAIN 时 `sentCount_` 仍然 +1
- 队列满时一律丢新帧，控制帧应考虑"丢旧保新"
- 无 per-axis 发送限流

### Commit 4

`fix(can_driver): handle transport send failures and control frame eviction in DeviceRuntime`

目标：

- 补齐 `DeviceRuntime` 的发送失败处理和队列淘汰策略
- 让统计数据真实可信

内容：

- `CanTransport::send()` 改为返回 `bool`（或 enum），表示发送成功/EAGAIN/链路异常
- `DeviceRuntime::workerLoop()` 根据返回值区分：
  - 成功 → `sentCount_++`
  - EAGAIN → 帧回插队头或丢弃，`eagainCount_++`，短暂退避后重试
  - 链路异常 → `linkDownCount_++`，触发降级
- Control 队列满时改用"丢最旧、保最新"淘汰策略（同一电机旧位置命令已过时）
- Recover / Config / Query 队列保持现有"丢新帧"策略
- 清理 `submit()` 中日志路径的多余 `snapshotStats()` 加锁

验收：

- 统计中 `sentCount_` 与 `eagainCount_` + `linkDownCount_` 之和等于实际 `send()` 调用次数
- Control 队列饱和时，新命令不会被丢弃，过时命令被淘汰
- EAGAIN 后 worker 不会陷入无退避的忙循环

### Commit 5

`refactor(can_driver): introduce shared driver state for feedback freshness and intents`

目标：

- 收敛跨线程数据面

内容：

- 增加统一的 feedback/command/intent 共享状态
- 建立 `last_rx_time`、`consecutive_timeout_count` 等正式字段
- 让 lifecycle、axis runtime、ROS 适配层共享同一份状态

验收：

- 反馈新鲜度和设备健康状态不再散落在多个对象里

### Commit 6

`refactor(can_driver): add axis runtime closed-loop states for pp path`

目标：

- 给现有主路径先补上轴级闭环状态

内容：

- 引入 `Offline/Seen/Standby/Armed/Running/Faulted/Recovering` 轴级状态
- 基于真实反馈判断 ready，而不是只看 service 成功
- 让恢复、使能、运行都依赖真实回读
- `AxisRuntime` 负责 per-axis 发送限流：offline 轴自动降低查询频率，连续超时轴暂停非关键请求

验收：

- lifecycle mode 与轴真实状态不再脱节
- 某轴异常不会拖累其它轴的发送资源

### Commit 7

`refactor(can_driver): slim CanDriverHW and remove protocol-owned refresh loops`

目标：

- 让 `CanDriverHW` 回到适配层角色

内容：

- 迁出总线调度与刷新策略
- 协议层只保留协议语义
- `CanDriverHW` 只保留 `RobotHW` / ROS 接线职责
- 删除 `DirectCanTxDispatcher`（已被 `DeviceRuntime` 全面替代，仅测试中保留或替换为 mock）

验收：

- `CanDriverHW` 不再继续膨胀成运行时和策略中心

### 实施顺序说明

Commit 1–3 已完成。下一步应立即开始 Commit 4，原因：

- 它是对已有 `DeviceRuntime` 的加固，不引入新抽象，改动范围小
- 不处理 `send()` 失败，后续上真机调试时几乎必然会碰到 EAGAIN 导致统计失真、退避缺失的问题
- Control 帧”丢新保旧”在高负载场景下会导致电机收到过时命令，越早修正越安全
- 完成后 `DeviceRuntime` 就是一个真正可靠的发送层，后续 Commit 5–7 可以放心往上堆

## 13. 迁移完整性审计（2026-04-18）

本节用于回答“删除 `can_driver-master`（或备份目录）后，`can_driver` 是否仍可完整运行”。

### 13.1 审计范围

- 代码与构建：`src/`、`include/`、`CMakeLists.txt`、`package.xml`
- 运行配置：`config/`、`launch/`、`scripts/`
- 文档：`docs/`
- 第三方 SDK：`lib/innfos-cpp-sdk`

### 13.2 审计结果

- 构建层已去除对 `can_driver-master` 路径的依赖。
- `lib/innfos-cpp-sdk` 已位于根包 `can_driver/lib/`，可独立提供 ECB 所需头文件与动态库。
- 与 `can_driver-master.__bak__` 对比后，根包缺失文件仅有历史临时文件：
  - `.codex`
  - `message.log`
  这两项均非运行、构建、测试、文档所需内容。
- `docs/` 已完成迁移：备份目录中的所有文档文件在根包 `docs/` 均可找到。

### 13.3 删除备份目录后的结论

在当前状态下，即使删除 `can_driver-master` / `can_driver-master.__bak__` 目录：

- `can_driver` 仍可完成构建并产出可执行节点；
- ECB/UDP/MT/PP 相关能力仍由根包内代码、配置、脚本与 SDK 提供；
- 文档体系仍完整保留在根包 `docs/` 中。

结论：迁移目标已满足“根包自洽、可独立运行”的要求。

## 14. MT/ECB 功能重点验证记录（2026-04-18）

本节聚焦验证：

- MT 电机控制链路是否正常
- ECB 控制链路是否正常
- 位置模式/速度模式是否可运行
- 角度与速度换算是否正确

### 14.1 已执行自动化验证

1. MT 协议单测：`test_mt_can_protocol`

- 结论：通过
- 覆盖点：
  - 速度模式下发帧编码（`setVelocity`）
  - 位置模式下发帧编码（`setPosition`）
  - 多圈角度读取保持（大数位置值）
  - 查询超时退避与刷新行为

2. CSP/位置模式相关单测：`test_eyou_can_csp`

- 结论：通过
- 覆盖点：
  - CSP 下位置快写流程
  - 模式切换后预配置速度与位置帧顺序
  - 位置模式与 CSP 模式切换后的连续动作行为

3. 角度/速度换算与方向符号单测：`test_joint_config_parser`、`test_limits_enforcement`

- 结论：通过
- 覆盖点：
  - PPR 到弧度比例换算（`2*pi/PPR`）
  - `direction_sign` 正负方向语义
  - `scaleAndClamp` 缩放与溢出钳位

4. 系统级模式与控制回归：`test_can_driver_hw_smoke`

- 结论：通过（45 tests, 0 failures）
- 覆盖点：
  - 位置/速度模式切换服务行为
  - CSP 位置指令换算与限位约束
  - 负方向关节对读写的修正

5. ECB 配置与设备管理补充单测（本次补回）

- `test_joint_config_parser` 新增：
  - ECB 固定 IP/发现策略/刷新周期解析
  - `ecb://<ip>` 回退解析
- `test_device_manager` 新增：
  - 无 SocketCAN 设备时 ECB 协议创建路径

### 14.2 验证结论（当前可确认范围）

- MT 路径：位置/速度控制与编码行为在单测层面可确认正常。
- 位置/速度模式切换：在系统 smoke 用例中可确认正常。
- 角度换算：PPR 缩放、方向修正、数值钳位在单测中可确认正确。
- ECB 路径：配置解析、协议装配与设备管理路径可确认正常。

### 14.3 仍需真机验证的部分

受限于当前环境无 ECB 实机网络，以下项需在真机执行脚本验证：

- ECB 电机实际位置闭环精度（目标角度与实测角度误差）
- ECB 速度模式实测稳态与动态响应
- 多电机联动时 ECB 刷新周期对控制稳定性的影响

建议使用根包已迁移脚本进行现场验证：

- `scripts/test_ecb_motor_motion.sh`
- `scripts/test_ecb_group_motion.sh`
- `scripts/test_ecb_four_motors.sh`
- `scripts/discover_ecb_and_enable_yaml.sh`
