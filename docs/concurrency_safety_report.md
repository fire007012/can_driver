# can_driver 并发安全修复报告

## 1. 范围
本次修复覆盖 `CanDriverHW` 的并发访问与生命周期安全，目标是消除以下高风险问题：
- `onShutdown()` 与 `read/write/service/subscriber` 并发导致容器竞争与悬空指针
- `joints_` 状态字段 (`pos/vel/eff`) 的并发读写数据竞争
- direct topic 回调线程直接打总线，和 `write()` 并发下发命令

涉及文件：
- `can_driver/include/can_driver/CanDriverHW.h`
- `can_driver/src/CanDriverHW.cpp`

## 2. 已落地修复

### 2.1 生命周期与容器并发保护
- 新增 `active_` 原子标志，`read/write/service` 在驱动 inactive 时直接返回错误或跳过。
- 新增 `protocolMutex_`（`std::shared_mutex`）保护：
  - `transports_`
  - `mtProtocols_`
  - `eyouProtocols_`
  - `deviceCmdMutexes_`
- `onShutdown()` 改为：先 `active_=false`，再在写锁下清理协议与 transport。

效果：避免 `clear()` 与并发读取 map 造成未定义行为。

### 2.2 协议对象生命周期安全
- `getProtocol()` 返回类型由裸指针改为 `std::shared_ptr<CanProtocol>`。
- 调用方持有 `shared_ptr` 后即便 map 被清空，对象也不会立刻悬空。

效果：降低 `use-after-free` 风险。

### 2.3 按 can_device 串行化协议调用
- 新增 `deviceCmdMutexes_`（每个 `can_device` 一把锁）。
- `read()/write()/onRecover()/onMotorCommand()` 在调用协议接口前，按设备加锁。

效果：同一总线上的协议调用被串行化，减少并发写总线/并发改协议状态的竞争面。

### 2.4 关节状态并发保护
- 新增 `jointStateMutex_`。
- `read()` 先采样到局部快照，再在锁内回写 `pos/vel/eff`。
- `publishMotorStates()` 在锁内拷贝状态到消息数组，锁外发布。

效果：消除 `read()` 与 `publishMotorStates()` 对 `pos/vel/eff` 的 data race。

### 2.5 消除 subscriber 旁路下发
- direct topic 回调不再直接 `proto->setVelocity/setPosition`。
- 回调仅更新 `JointConfig` 里的 direct 命令缓冲：
  - `directVelCmd/hasDirectVelCmd`
  - `directPosCmd/hasDirectPosCmd`
- `write()` 成为唯一下发路径，统一执行命令下发。

效果：避免回调线程与控制循环线程同时打总线。

### 2.6 命令输入加强
- `CMD_SET_MODE` 仅允许 `value` 为 `0` 或 `1`，其余返回错误。

## 3. 验证结果
执行：
- `catkin_make --pkg can_driver`

结果：
- 编译通过，`can_driver_transport` 与 `can_driver_node` 均成功链接。

## 4. 与原问题对照
- 问题 1（`onShutdown()` 竞争）：**已修复（主要路径）**
- 问题 2（`joints_` 状态 data race）：**已修复**
- 问题 3（协议对象并发调用）：**部分修复**（通过 per-device 锁显著收敛）
- 问题 4（subscriber 与 write 冲突）：**已修复**（统一 write 下发）
- 问题 5（`initDevice()` 运行时修改 map）：**已修复（加写锁）**

## 5. 残余风险与后续建议
- 当前 direct 命令缓冲为“持续覆盖”策略（写入后一直生效，直到新的 direct 命令或 Stop/Disable 清除）。若希望和 controller 严格仲裁，建议增加显式优先级与超时失效策略。
- `read()/write()` 对 `joints_` 元数据假定初始化后不再改动；若后续支持运行时热插拔 joint，需要把 `joints_` 结构本身纳入读写锁保护。
- 建议补 TSAN 压测与故障注入用例，覆盖：并发 `shutdown + motor_command + read/write`。
