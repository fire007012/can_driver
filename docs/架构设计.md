# can_driver 架构文档

日期：2026-03-03  
适用版本：当前 `main` 分支

## 1. 目标与边界

`can_driver` 是 ROS1 `ros_control` 的硬件抽象实现，负责：

- 接收 `controller_manager` 的控制命令（`read/write` 循环）
- 管理 CAN 设备与协议对象（MT / PP）
- 提供服务接口（初始化、关闭、恢复、单电机命令）
- 提供 direct topic（调试旁路命令）与状态发布

不负责：

- 轨迹规划（MoveIt）
- 底盘导航（Navi）
- 上层控制器策略

## 2. 模块分层

```text
controller_manager / ros_control controllers
                |
                v
         +--------------+
         |  CanDriverHW |
         +--------------+
          |         |
          |         +-- SafeCommand (数值安全: scale/clamp)
          |
          +-- DeviceManager (transport/protocol/mutex 生命周期)
          |
          +-- JointConfigParser (参数解析与校验)
                |
                v
      SocketCanController + MtCan / EyouCan
```

### 2.1 CanDriverHW

职责：

- 生命周期编排（`init/reset/shutdown`）
- `read/write` 控制循环
- ROS 服务与 topic 接入
- direct 命令超时仲裁与限位统一处理

### 2.2 DeviceManager

职责：

- `can_device` 对应 transport 的创建/复用/重置
- MT/PP 协议对象创建与访问
- 设备级互斥锁管理
- 刷新线程启动与设备统一关闭

### 2.3 JointConfigParser

职责：

- `joints` 参数解析与校验
- `motor_id`（int/hex string）解析
- `protocol/control_mode/scale` 等字段验证

### 2.4 SafeCommand

职责：

- 命令缩放与整数范围钳制
- 非法输入（NaN/Inf/非法 scale）拒绝

## 3. 关键数据结构

### 3.1 JointConfig（CanDriverHW 内部）

每个 joint 保存：

- 静态配置：`name/motorId/protocol/canDevice/controlMode/scale`
- 状态缓存：`pos/vel/eff`
- ros_control 命令缓存：`posCmd/velCmd`
- direct 命令缓存：`directPosCmd/directVelCmd + hasDirect* + lastDirect*Time`
- 限位快照：`limits/hasLimits`

### 3.2 预计算分组与命令缓冲

- `jointGroups_`：按 `(canDevice, protocol)` 的 joint 分组，初始化时构建，`read/write` 复用
- `rawCommandBuffer_`：每周期目标原始指令缓存（`int32_t`）
- `commandValidBuffer_`：每周期有效标记缓存（`uint8_t`）

设计目标：避免实时路径重复构造 map/vector 与字符串拷贝。

## 4. 控制循环流程

### 4.1 read()

1. 检查 `active_`
2. 遍历 `jointGroups_`，按设备锁串行读取协议缓存
3. 写入局部快照
4. 在 `jointStateMutex_` 下回写 `joints_` 状态字段

### 4.2 write()

1. 检查 `active_`
2. 执行 `pos/vel` 限位接口（`enforceLimits(period)`）
3. 在 `jointStateMutex_` 下为每个 joint 决策命令源：
   - direct 命令且未超时 -> 用 direct
   - 否则用 controller 命令
4. 对最终命令执行限位钳制与 `scaleAndClampToInt32`
5. 遍历 `jointGroups_`，按设备锁发送到协议层

direct 命令安全规则：

- 不绕过限位
- 受 `direct_cmd_timeout_sec` 超时控制，超时后自动回退到 ros_control 输出

## 5. ROS 接口与调用链

### 5.1 Service

- `~init`：设备（重）初始化
- `~shutdown`：停止驱动并关闭全部设备
- `~recover`：按 motor_id 或通配恢复
- `~motor_command`：Enable/Disable/Stop/SetMode

`onMotorCommand` 使用表驱动执行常规命令，并通过 `executeOnMotor` 统一错误路径。

### 5.2 Topic

- `~motor/<joint>/cmd_velocity`
- `~motor/<joint>/cmd_position`
- `~motor_states`

`~motor_states` 语义：发布编码器原始计数（`pos/vel` 会按 scale 反变换）。

## 6. 并发模型与锁策略

### 6.1 共享状态

- `active_`：生命周期原子标志
- `jointStateMutex_`：保护 `joints_` 的状态/命令缓存字段
- 设备锁（由 `DeviceManager` 管理）：串行化同设备协议调用

### 6.2 锁顺序

推荐顺序：

1. `jointStateMutex_`（仅在解析命令或写回状态时短持有）
2. 设备锁（协议调用前）

当前实现避免在持有设备锁时再申请 `jointStateMutex_`，降低反向锁序风险。

## 7. 生命周期

### 7.1 init()

1. `resetInternalState()`
2. `loadRuntimeParams()`
3. `parseAndSetupJoints()`
4. `registerJointInterfaces()`
5. `loadJointLimits()`
6. `startMotorRefreshThreads()`
7. `setupRosComm()`
8. `active_=true`

### 7.2 reset/shutdown/destructor

- `resetInternalState()`：停 timer、停订阅、清 joint 缓存、关闭设备管理器
- `onShutdown()`：`active_=false` + `deviceManager_.shutdownAll()` + 清 direct 标记
- 析构函数复用 `resetInternalState()` 后收口 service 句柄

## 8. 配置参数

- `joints`：关节映射与协议配置
- `direct_cmd_timeout_sec`：direct 命令超时秒数（默认 0.5）
- `motor_state_period_sec`：状态发布周期秒数（默认 0.1）
- `direct_cmd_queue_size`：direct 订阅队列长度（默认 1）

参数非法时采用安全回退默认值并告警。

## 9. 扩展指南

### 9.1 新增协议类型

1. 扩展 `CanType`
2. 在 `DeviceManager` 增加协议容器与创建逻辑
3. 在 `JointConfigParser` 放开协议字段校验
4. 在 `CanDriverHW` 分组与发送路径接入新类型

### 9.2 新增 motor command

1. 扩展 srv 常量
2. 在 `onMotorCommand` 表驱动增加条目
3. 若需要额外参数校验，单独分支处理

## 10. 测试策略

现有：

- `test_safe_command`
- `test_joint_config_parser`
- `test_socketcan_controller`

建议补充：

- `DeviceManager` mock 测试（设备不可用/协议缺失/异常路径）
- `CanDriverHW` 级别集成测试（shutdown 与并发 service 组合）
- direct 命令超时回退与限位一致性测试
