# P6 修复过程报告：协议命令返回值语义升级

## 1. 背景
P6 问题是协议命令接口无返回值，`CanDriverHW` 无法判定命令是否被协议层接受，导致 service 常出现“无条件 OK”。

本次目标：把命令接口从 `void` 升级为 `bool`，让上层能基于返回值反馈失败。

## 2. 变更范围
- `include/can_driver/CanProtocol.h`
- `include/can_driver/MtCan.h`
- `include/can_driver/EyouCan.h`
- `src/MtCan.cpp`
- `src/EyouCan.cpp`
- `src/CanDriverHW.cpp`

## 3. 修复步骤

### 3.1 协议抽象接口改造
将以下方法统一改为 `bool` 返回：
- `setMode`
- `setVelocity`
- `setAcceleration`
- `setDeceleration`
- `setPosition`
- `Enable`
- `Disable`
- `Stop`

约定：
- `true`：协议层接受/执行了该命令
- `false`：命令被拒绝或前置条件不满足（如控制器不可用）

### 3.2 MtCan 实现对齐
- 所有对应方法签名改为 `bool`。
- 在缺少 `canController` 时返回 `false`。
- 成功走到下发路径后返回 `true`。
- `Disable()` 改为返回 `Stop()` 的结果。

### 3.3 EyouCan 实现对齐
- 同步将所有对应方法改为 `bool`。
- 若 `canController` 不存在返回 `false`。
- 可执行路径返回 `true`。

### 3.4 CanDriverHW 调用链对齐
- `write()`：
  - 调用 `setVelocity/setPosition` 后检查返回值。
  - 返回 `false` 时输出节流告警，不再默默吞掉。
- `onRecover()`：
  - 调用 `Enable` 后检查返回值。
  - `false` 时跳过该电机，不计为恢复成功。
- `onMotorCommand()`：
  - 对 `Enable/Disable/Stop/setMode` 逐条检查返回值。
  - 返回 `false` 时直接设置 `res.success=false` 并返回明确 message。

## 4. 验证
执行：
- `catkin_make --pkg can_driver`

结果：
- 编译通过，`can_driver_transport` 与 `can_driver_node` 成功构建。

## 5. 修复效果
- P6 从“仅靠日志感知失败”升级为“接口层可感知失败”。
- 上层 service 调用可得到真实失败反馈，避免“设备不就绪仍返回 OK”。

## 6. 已知限制
- 目前 `CanTransport::send` 仍为 `void`，协议层无法获得总线发送结果的硬确认。
- 因此 `bool` 语义是“命令被协议层接受并尝试下发”，不是“电机已执行成功”。
- 若要进一步提升真实性，需要后续扩展传输层确认机制（ACK/错误码回传）。
