# can_driver 单主站分层架构说明

日期：2026-04-01

## 1. 定位

`can_driver` 的正确定位不是“发 CAN 帧的底层驱动”，而是：

- 一个意优私有 CAN 协议电机主站运行时
- 一个单主站、单生命周期的电机管理系统
- 一个支持多协议后端的统一控制平面

这意味着：

- 系统层只有一个主站
- 生命周期只有一套权威状态机
- `PP` 和 `MT` 只是主站下的不同协议适配器

不应该变成：

- `PP` 一套状态机
- `MT` 另一套状态机
- 上层再包一层系统状态机

## 2. 总体分层

建议按下面 6 层理解 `can_driver`：

```text
ROS / controller_manager / service / topic / action
                        |
                        v
1. 接入层 CanDriverHW
                        |
                        v
2. 生命周期编排层 OperationalCoordinator / LifecycleDriverOps
                        |
                        v
3. 状态与判据层 SharedDriverState / AxisReadinessEvaluator
                        |
                        v
4. Device 运行时层 DeviceManager / DeviceRuntime / RefreshScheduler
                        |
                        v
5. 协议适配层 EyouCan / MtCan
                        |
                        v
6. 传输层 SocketCanController / CanTxDispatcher / CanTransport
```

这 6 层里：

- 上三层表达“管理语义”
- 中间一层负责“运行时调度”
- 下两层负责“协议与传输”

## 3. 各层职责

### 3.1 接入层

当前主要文件：

- `src/CanDriverHW.cpp`
- `include/can_driver/CanDriverHW.h`
- `src/can_driver_node.cpp`

职责：

- 对接 `ros_control`
- 提供 topic / service / action 接口
- 解析参数和 joint 配置
- 把上层意图翻译成系统动作或目标命令

不应该做的事：

- 直接理解协议子命令
- 直接维护协议刷新逻辑
- 直接解释设备级故障恢复细节

### 3.2 生命周期编排层

当前主要文件：

- `src/operational_coordinator.cpp`
- `include/can_driver/operational_coordinator.hpp`
- `src/lifecycle_driver_ops.cpp`
- `include/can_driver/lifecycle_driver_ops.hpp`
- `src/lifecycle_service_gateway.cpp`
- `include/can_driver/lifecycle_service_gateway.hpp`
- `src/command_gate.cpp`
- `include/can_driver/command_gate.hpp`

职责：

- 维护唯一系统生命周期
- 编排 `init / enable / halt / resume / disable / recover / shutdown`
- 决定系统什么时候允许运动、什么时候必须冻结

唯一有状态名解释权的对象应该是：

- `OperationalCoordinator`

这一层可以说：

- `Configured`
- `Standby`
- `Armed`
- `Running`
- `Faulted`
- `Recovering`
- `ShuttingDown`

但协议层和轴级层不应再自造另一套同名状态机。

### 3.3 状态与判据层

当前主要文件：

- `include/can_driver/SharedDriverState.h`
- `include/can_driver/AxisReadinessEvaluator.h`

职责：

- 保存共享事实
- 保存 intent
- 保存设备健康与轴反馈
- 输出轴级准入判据

这层应只回答：

- 反馈是不是新鲜
- 模式是不是匹配
- 使能是不是到位
- 故障是不是清掉
- 现在能不能 run

不应保存：

- 调度器内部状态
- 第二套生命周期状态名

### 3.4 Device 运行时层

当前主要文件：

- `src/DeviceManager.cpp`
- `include/can_driver/DeviceManager.h`
- `src/DeviceRuntime.cpp`
- `include/can_driver/DeviceRuntime.h`
- `include/can_driver/RefreshScheduler.h`
- `include/can_driver/DeviceRefreshWorker.h`

职责：

- 一台设备一个 runtime
- 统一该设备的发包所有权
- 处理 deadline 调度、背压、budget、refresh worker
- 在协议层之上做 runtime 级治理

这一层的本质是：

- 决定“什么时候查什么”
- 决定“现在总线压力下本轮发多少”

而不是：

- 决定系统生命周期
- 直接解释电机是否逻辑上处于 `Running`

### 3.5 协议适配层

当前主要文件：

- `src/EyouCan.cpp`
- `include/can_driver/EyouCan.h`
- `src/MtCan.cpp`
- `include/can_driver/MtCan.h`
- `include/can_driver/CanProtocol.h`

职责：

- 协议编解码
- 读写请求登记
- ACK / 回包解析
- 协议事实回填到共享状态

这里是 `PP` 和 `MT` 真正应该分开的地方。

原因不是它们属于不同系统，而是：

- 它们的命令模型不同
- 它们的反馈模型不同
- 它们的 refresh 字段不同

#### `PP / EyouCan`

更像“寄存器式伺服协议”：

- 有 `mode`
- 有 `enable`
- 有 `fault`
- 有 `position / velocity / current`
- 支持 `CSP`

#### `MT / MtCan`

更像“多圈执行器协议”：

- 以 `state / error / multiTurnAngle` 为核心
- 不具备与 `PP` 完全等价的 mode/enable 语义
- `quickSetPosition()` 也不是同一类能力

所以：

- `MT` 应独立成协议后端
- 但不能独立成第二个主站

### 3.6 传输层

当前主要文件：

- `src/SocketCanController.cpp`
- `include/can_driver/SocketCanController.h`
- `include/can_driver/CanTransport.h`
- `include/can_driver/CanTxDispatcher.h`

职责：

- 打开 SocketCAN
- 发送和接收 CAN 帧
- 处理非阻塞发送
- 统计背压和链路健康

这层只应该理解：

- 帧
- 队列
- errno
- backpressure

不应该理解：

- 生命周期
- 轴状态
- 模式使能故障

## 4. 两条关键流

### 4.1 命令下行流

```text
ROS接口 / controller_manager
  -> CanDriverHW
    -> OperationalCoordinator / LifecycleDriverOps
      -> DeviceManager / DeviceRuntime
        -> EyouCan 或 MtCan
          -> SocketCanController
```

含义：

- 上层说“初始化、恢复、开始运行、目标位置”
- 中间层决定“该不该放行、该怎么调度”
- 协议层决定“发哪种命令帧”
- 传输层只负责送出去

### 4.2 状态回流

```text
CAN回包
  -> SocketCanController
    -> EyouCan / MtCan 解析
      -> SharedDriverState 写入事实
        -> AxisReadinessEvaluator 输出判据
          -> OperationalCoordinator 更新系统决策
```

含义：

- 协议层回填事实
- 判据层不创造状态，只消费事实
- 生命周期层根据判据决定系统行为

## 5. 四条必须守住的边界

### 5.1 接入层不碰协议细节

`CanDriverHW` 不应直接依赖：

- `0x03 / 0x04 / 0x05`
- 某个协议的具体寄存器地址

### 5.2 生命周期层不碰协议帧

`OperationalCoordinator` / `LifecycleDriverOps` 不应关心：

- `PP` 用什么子命令
- `MT` 用什么读状态命令

它只关心：

- init 是否完成
- 轴是否 ready
- 是否允许进入 `Running`

### 5.3 SharedState 不收调度器内部状态

`SharedDriverState` 应只保存共享事实，例如：

- feedback
- command
- intent
- device health

不应保存：

- `next_due`
- `last_issue`
- `budget_cursor`
- `backoff`

这些属于 `DeviceRuntime / ProtocolRuntime` 内部实现状态。

### 5.4 协议层不拥有系统生命周期

`EyouCan` 和 `MtCan` 可以维护：

- 协议缓存
- 发送状态
- pending request
- refresh freshness

但不能维护：

- `Armed`
- `Running`
- `Recovering`

这些语义只能由系统生命周期层定义。

## 6. 为什么 MT 要独立成协议层，而不是独立成系统

`MT` 确实应该独立出来，但独立的是“协议适配器”，不是“主站系统”。

原因：

- `MT` 与 `PP` 的控制模型不同
- `MT` 与 `PP` 的反馈字段不同
- `MT` 与 `PP` 的 enable/mode/fault 语义并不完全等价

如果不独立成协议后端，就会出现两种坏情况：

- 为了共用逻辑，硬把 `MT` 往 `PP` 的设备模型里塞
- 或者把上层生命周期写成大量 `if (PP) ... else if (MT) ...`

所以正确做法是：

- 一个主站运行时
- 一套生命周期主链
- 多个协议适配器后端

也就是：

- 顶层统一
- 协议分治

## 7. 这套分层对标 Eyou_Canopen_Master 的意义

如果未来要和 `Eyou_Canopen_Master` 用胶水层统一，最重要的是：

- 对外系统生命周期语义一致
- 对内允许协议不同

因此：

- `Eyou_Canopen_Master` 下层是 CiA402 协议与状态机
- `can_driver` 下层是私有协议适配器

但两者上层都应该是：

- 单一主站
- 单一生命周期
- 统一的 enable / resume / halt / recover / shutdown 语义

## 8. 当前文件归层建议

### 接入层

- `src/CanDriverHW.cpp`
- `include/can_driver/CanDriverHW.h`
- `src/can_driver_node.cpp`
- `src/JointConfigParser.cpp`
- `include/can_driver/JointConfigParser.h`

### 生命周期编排层

- `src/operational_coordinator.cpp`
- `include/can_driver/operational_coordinator.hpp`
- `src/lifecycle_driver_ops.cpp`
- `include/can_driver/lifecycle_driver_ops.hpp`
- `src/lifecycle_service_gateway.cpp`
- `include/can_driver/lifecycle_service_gateway.hpp`
- `src/command_gate.cpp`
- `include/can_driver/command_gate.hpp`
- `src/motor_action_executor.cpp`
- `include/can_driver/motor_action_executor.hpp`
- `src/SafeCommand.cpp`
- `include/can_driver/SafeCommand.h`

### 状态与判据层

- `include/can_driver/SharedDriverState.h`
- `include/can_driver/AxisReadinessEvaluator.h`
- `include/can_driver/AxisRuntime.h`

### Device 运行时层

- `src/DeviceManager.cpp`
- `include/can_driver/DeviceManager.h`
- `src/DeviceRuntime.cpp`
- `include/can_driver/DeviceRuntime.h`
- `include/can_driver/DeviceRefreshWorker.h`
- `include/can_driver/RefreshScheduler.h`

### 协议适配层

- `src/EyouCan.cpp`
- `include/can_driver/EyouCan.h`
- `src/MtCan.cpp`
- `include/can_driver/MtCan.h`
- `include/can_driver/CanProtocol.h`
- `include/can_driver/CanType.h`
- `include/can_driver/MotorID.h`

### 传输层

- `src/SocketCanController.cpp`
- `include/can_driver/SocketCanController.h`
- `include/can_driver/CanTransport.h`
- `include/can_driver/CanTxDispatcher.h`

### 工具层

- `scripts/realtime_motor_ui.py`

## 9. 最终判断

以后理解 `can_driver`，建议固定成一句话：

`can_driver = 意优私有 CAN 电机主站运行时`

不是“纯驱动”，也不是“两套协议各自为政的小系统”，而是：

- 一个主站
- 一套系统生命周期
- 多个协议适配器
- 一个统一的设备运行时

这就是后续所有重构应当共同服从的分层基线。
