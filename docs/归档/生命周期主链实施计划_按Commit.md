# can_driver 生命周期主链实施计划（按 Commit）

## Summary

本文档是 `can_driver/docs/生命周期主链重构计划.md` 的执行版，目标是把
`can_driver` 当前已经长进 `CanDriverHW` 的生命周期、安全保持、恢复编排逻辑，
按可落地的 commit 顺序拆分出去。

计划基于当前代码现状制定：

- `OperationalCoordinator` 已存在，但仍偏轻量
- 生命周期服务已经加入 `can_driver`
- `recover -> Enable` 已拆开，`ResetFault()` 已加入协议抽象
- `fresh command latch` 已接入，但仍长在 `CanDriverHW`
- `CanDriverHW` 仍然同时承担：
  - `RobotHW` 桥接
  - 生命周期 service 入口
  - 生命周期动作编排
  - 命令保持与 fresh-command 同步
  - 设备掉线恢复策略
  - 故障检测降级

本计划的结束状态是：

- `LifecycleServiceGateway` 只负责生命周期服务入口
- `OperationalCoordinator` 负责生命周期状态迁移与动作编排
- `CommandGate` 负责命令保持、fresh-command、掉线恢复后的闸门
- `CanDriverHW` 回到 `RobotHW` 适配层角色

## 最终目标

重构完成后，`can_driver` 必须满足以下最终行为：

- `recover` 只表示 `Faulted -> Standby`
- `recover` 不得隐式 `enable` 或 `resume`
- `Running` 之外不得下发正常运动命令
- `enable` 只进入 `Armed`
- `resume` 才进入 `Running`
- `halt` 从 `Running` 回到 `Armed`
- `enable`、`resume`、`recover`、设备掉线恢复后，都必须要求 fresh command
- 生命周期 service 返回值必须反映真实执行结果，不允许假成功
- 自动故障降级统一收口到 coordinator，不再在多个位置各自切状态

## Commit Plan

### Commit 1

`refactor(can_driver): introduce lifecycle runtime and service gateway skeleton`

目标：

- 先把生命周期入口从 `CanDriverHW` 身上摘下来
- 但暂时不改业务逻辑，只做结构搬运和依赖反转

新增或改动的核心对象：

- 新增 `LifecycleServiceGateway`
- 新增 `LifecycleRuntime` 或等价的共享上下文对象
- `CanDriverHW` 暴露最小必要接口给 gateway/coordinator 使用

建议修改文件：

- `include/can_driver/lifecycle_service_gateway.hpp`
- `src/lifecycle_service_gateway.cpp`
- `include/can_driver/CanDriverHW.h`
- `src/CanDriverHW.cpp`
- `src/can_driver_node.cpp`
- `CMakeLists.txt`

实现要求：

- `LifecycleServiceGateway` 注册以下服务：
  - `init`
  - `shutdown`
  - `recover`
  - `enable`
  - `disable`
  - `halt`
  - `resume`
- `CanDriverHW` 不再直接 `advertiseService()` 上述生命周期服务
- gateway 先只做转发，具体逻辑仍可暂时调用 `CanDriverHW` 暴露的现有入口
- `motor_command` 和 `set_zero_limit` 保持在 `CanDriverHW`
- 对外服务名与当前保持一致，避免第一刀就引入接口变动

完成标志：

- `CanDriverHW` 中删除生命周期 service server 成员
- `CanDriverHW` 中删除生命周期 service 注册逻辑
- 节点启动后生命周期服务仍然可见、可调用

验收：

- `catkin_make --pkg can_driver`
- 冒烟验证服务存在：
  - `rosservice list | grep can_driver_node`

不做的事：

- 不在这个 commit 里重写 `OperationalCoordinator`
- 不在这个 commit 里动 `freshCommandRequired_` 相关逻辑

### Commit 2

`refactor(can_driver): extract motor action executor and unify protocol operations`

目标：

- 先把协议访问、设备锁、状态码映射从 `CanDriverHW` 抽出来
- 给后续 coordinator 和维护服务共用

新增或改动的核心对象：

- 新增 `MotorActionExecutor`
- 统一封装当前 `executeOnMotor()` 的语义
- 支持批量动作和失败聚合

建议修改文件：

- `include/can_driver/motor_action_executor.hpp`
- `src/motor_action_executor.cpp`
- `include/can_driver/CanDriverHW.h`
- `src/CanDriverHW.cpp`
- 如有必要，补充 `IDeviceManager` 相关暴露

实现要求：

- `MotorActionExecutor` 统一处理：
  - `DeviceNotReady`
  - `ProtocolUnavailable`
  - `Rejected`
  - `Exception`
- 支持至少以下动作：
  - `Enable`
  - `Disable`
  - `Stop`
  - `ResetFault`
- 支持按 joint 执行和按全 joint 批量执行
- 提供“成功集/失败首项/是否部分成功”的结果结构
- `CanDriverHW::onMotorCommand()`、现有 lifecycle handler 临时改用该执行器

完成标志：

- `CanDriverHW` 不再自己维护 `MotorOpStatus + executeOnMotor()` 那套协议访问实现
- 生命周期主链和维护服务共用同一个动作执行器

验收：

- `catkin_make --pkg can_driver`
- `catkin_make run_tests_can_driver`

不做的事：

- 不在这个 commit 里变动生命周期状态机规则
- 不在这个 commit 里调整 recover 语义

### Commit 3

`refactor(can_driver): promote operational coordinator to own lifecycle actions`

目标：

- 让 `OperationalCoordinator` 成为真正的生命周期协调器
- 生命周期动作编排不再留在 `CanDriverHW`

建议修改文件：

- `include/can_driver/operational_coordinator.hpp`
- `src/operational_coordinator.cpp`
- `include/can_driver/lifecycle_runtime.hpp` 或等价上下文头
- `src/lifecycle_service_gateway.cpp`
- `src/CanDriverHW.cpp`

实现要求：

- 为 `OperationalCoordinator` 注入 `DriverOps`
- `DriverOps` 至少提供：
  - `init_device`
  - `enable_all`
  - `disable_all`
  - `halt_all`
  - `reset_all_faults`
  - `shutdown_all`
  - `all_motion_healthy`
  - `all_faults_cleared`
  - `request_hold_and_fresh_command`
- 以下动作全部迁入 coordinator：
  - `RequestInit`
  - `RequestEnable`
  - `RequestDisable`
  - `RequestRelease`
  - `RequestHalt`
  - `RequestRecover`
  - `RequestShutdown`

必须一次性修掉的语义问题：

- `enable` 部分成功部分失败时，必须回滚已成功 enable 的电机
- `disable` / `halt` 失败时不能返回成功
- `recover` 只允许全局恢复，不再支持 per-motor 业务语义
- `recover` 成功前必须确认 fault 真正清除
- `recover` 失败时保持 `Faulted`

兼容策略：

- `~recover` 继续使用当前 `Recover.srv`
- 但只接受 `motor_id == 0xFFFF`
- 传入具体 `motor_id` 直接失败，报 legacy 提示

完成标志：

- `LifecycleServiceGateway` 只负责调 coordinator
- `CanDriverHW` 中删除 `onInit/onShutdown/onRecover/onEnable/onDisable/onHalt/onResume`

验收：

- `catkin_make --pkg can_driver`
- 新增并通过 `test_operational_coordinator`

### Commit 4

`refactor(can_driver): extract command gate and unify hold semantics`

目标：

- 把命令保持、fresh-command、掉线恢复保持逻辑从 `CanDriverHW` 抽出去
- 消除现在散落在多个位置的重复命令清理逻辑

新增或改动的核心对象：

- 新增 `CommandGate`

建议修改文件：

- `include/can_driver/command_gate.hpp`
- `src/command_gate.cpp`
- `include/can_driver/CanDriverHW.h`
- `src/CanDriverHW.cpp`
- `src/operational_coordinator.cpp`

实现要求：

- `CommandGate` 统一负责：
  - `holdCommandsForLifecycleTransition()`
  - `armFreshCommandLatch()`
  - `consumeFreshCommandLatchIfSatisfied()`
  - device not ready 的命令清理
  - device recover 后的一拍 hold
- `CanDriverHW` 不再持有以下状态：
  - `commandLatchBaselines_`
  - `freshCommandRequired_`
  - `lastDeviceReadyState_`
- `CanDriverHW::write()` 改为：
  - 读取 lifecycle mode
  - 若非 `Running`，经 gate 返回“阻断”
  - 命令采样后交 gate 判断是否满足 fresh command
  - 仅在 gate 放行时写总线
- startup 对齐、lifecycle 切换、recover、掉线恢复，都统一走 gate

完成标志：

- `CanDriverHW` 中删除上述 latch/ready-state 字段和对应 helper
- `write()` 长度明显下降，职责集中到“采样/限幅/下发”

验收：

- `catkin_make --pkg can_driver`
- `test_can_driver_hw_smoke` 新增以下覆盖：
  - `halt -> resume` 后旧命令不会直接透传
  - `recover` 后未收到 fresh command 不下发运动
  - 掉线恢复后必须 fresh command 才重新放行

### Commit 5

`refactor(can_driver): route health updates through coordinator`

目标：

- 把自动故障降级的入口从 `CanDriverHW` 的多个位置收口到 coordinator
- 避免 `CanDriverHW` 再维护第二套状态机

建议修改文件：

- `include/can_driver/operational_coordinator.hpp`
- `src/operational_coordinator.cpp`
- `src/CanDriverHW.cpp`

实现要求：

- coordinator 提供统一的健康更新入口，例如：
  - `UpdateFromFeedback()`
  - 或 `ReportHealthSnapshot()`
- `publishMotorStates()` 只负责收集健康快照并上报 coordinator
- `write()` 里发现 fault 时，不再直接 `SetFaulted()`，改为上报或触发统一降级入口
- 自动故障降级只允许 coordinator 改 lifecycle mode

完成标志：

- `CanDriverHW` 中删除直接 `SetFaulted()` 的运行期散点
- lifecycle mode 的主动切换只剩 reset/config/coordinator 主路径

验收：

- `catkin_make --pkg can_driver`
- `test_operational_coordinator` 覆盖：
  - `Armed/Running` 下故障自动降级到 `Faulted`

### Commit 6

`refactor(can_driver): shrink CanDriverHW to robot-hw adapter role`

目标：

- 完成主链收口
- 清理 `CanDriverHW` 中已无必要的 lifecycle、安全策略残留

建议修改文件：

- `include/can_driver/CanDriverHW.h`
- `src/CanDriverHW.cpp`
- 新增/调整的 runtime、gateway、executor、gate 相关头源文件

实现要求：

- `CanDriverHW` 只保留：
  - 参数读取和 joint 初始化
  - `read()` 反馈同步
  - `write()` 命令采样、限幅、缩放、协议写入
  - direct topic
  - `motor_states`
  - `motor_command`
  - `set_zero_limit`
- 清理与 lifecycle 强耦合的遗留 helper、状态字段、重复 reset 逻辑
- 头文件中与生命周期主链相关的成员只保留对 runtime/gate/coordinator 的依赖注入引用

完成标志：

- `CanDriverHW.cpp` 不再承担 service gateway、动作编排、命令同步策略三类职责
- 文件规模和职责边界都明显收敛

验收：

- `catkin_make --pkg can_driver`
- `catkin_make run_tests_can_driver`

### Commit 7

`docs(can_driver): align lifecycle docs and scripts with global recover semantics`

目标：

- 文档与脚本统一到新语义
- 不再让旧 `recover` 语义继续污染后续使用方式

建议修改文件：

- `docs/使用指南.md`
- `docs/单电机控制说明.md`
- `docs/无硬件集成测试方案.md`
- `docs/架构设计.md`
- 相关 shell/python 脚本

实现要求：

- 所有文档统一写明：
  - `recover` 是全局生命周期恢复
  - `recover` 后不会自动 enable
  - `recover` 后必须 fresh command
- 所有示例调用改为：
  - `motor_id: 65535`
- 删除或标记 legacy 的 per-motor recover 说明
- 保留 `motor_command` 的维护入口说明，但明确其不是正式生命周期语义

完成标志：

- 包内文档不再存在“recover = 重新使能”或“recover 可单轴恢复”的正式表述

验收：

- 人工检查 `rg -n "recover" docs scripts README.md`

## 测试矩阵

必须新增或补齐以下测试：

### 单测

- `test_operational_coordinator.cpp`
  - 状态迁移矩阵
  - `enable` 回滚
  - `disable/halt` 失败上报
  - `recover` 成功与失败
  - 自动 fault downgrade

- `test_command_gate.cpp`
  - fresh-command latch
  - 非 `Running` 阻断
  - device recover hold
  - hold 后旧 direct/ros_control 命令不透传

### 集成/冒烟

- 扩展 `tests/test_can_driver_hw_smoke.cpp`
  - 生命周期服务仍可调用
  - `enable -> resume` 后 fresh command 才放行
  - `halt -> resume` 后旧命令不透传
  - `recover` 后阻断直到 fresh command
  - 指定单个 `motor_id` 的 `recover` 被拒绝

## 执行顺序约束

执行时必须遵守以下顺序，不能跳：

1. 先拆 gateway，再拆 coordinator
2. 先抽 protocol action executor，再让 coordinator 编排动作
3. 先让 coordinator 接管生命周期主链，再抽 command gate
4. 先把状态切换入口收口，再做 `CanDriverHW` 清瘦
5. 文档和脚本收尾必须放在最后

原因：

- 这样可以保证每个 commit 都处于可编译、可验证状态
- 避免一开始同时拆 service、state machine、write gate，导致 review 和回归都不可控

## Assumptions

- 本计划基于当前 `feature/can-driver-lifecycle-refactor` 分支继续推进
- 本轮不进入统一外观层节点实现，只完成 `can_driver` 包内主链重构
- `Recover.srv` 本轮先不做 breaking change
- `.vscode` 本地变更不纳入任何 commit
