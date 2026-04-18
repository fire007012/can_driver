# can_driver 代码结构与可维护性重构报告

日期：2026-03-03
范围：`can_driver` 包（重点：`CanDriverHW`）

## 1. 背景

在完成并发与安全修复后，`CanDriverHW.cpp` 仍存在明显的结构性问题：

- 单文件职责过多（生命周期、参数解析、设备管理、控制循环、ROS 通信、服务处理混杂）
- `init()` 体量过大，流程依赖隐式
- service 路径存在重复模板代码（获取资源、检查、加锁、异常处理）
- 数值缩放/钳制逻辑分散，难复用与测试
- 若干运行参数使用魔法数字，不利于部署调优

本轮目标是先做“低风险、可连续交付”的结构化改造，保持行为稳定，同时降低后续大模块拆分难度。

## 2. 执行策略

采用分轮改造，每轮均独立编译并提交：

1. 先做函数级拆分（不改行为）
2. 抽取重复执行模板
3. 抽取数值安全工具模块
4. 参数化魔法数字

## 3. 本次落地改动

### 3.1 `init()` 流程重构（函数级拆分）

提交：`8bbc2f8`（本次补充见最新提交）

将原始 `init()` 的巨型流程拆为可读步骤函数：

- `resetInternalState()`
- `loadRuntimeParams(...)`
- `parseAndSetupJoints(...)`
- `rebuildJointGroups(...)`
- `registerJointInterfaces()`
- `loadJointLimits(...)`
- `startMotorRefreshThreads()`
- `setupRosComm(...)`

效果：

- `init()` 变为流程编排入口，启动步骤清晰
- 降低定位初始化故障时的认知负担
- 为后续独立模块文件拆分做铺垫
- `parseAndSetupJoints(...)` 失败路径增加统一 `resetInternalState()` 回滚

### 3.2 统一电机操作执行模板

提交：`e036381`

新增：

- `executeOnMotor(...)`
- `MotorOpStatus` 状态枚举

当前 `MotorOpStatus` 取值：

- `Ok`
- `DeviceNotReady`
- `ProtocolUnavailable`
- `Rejected`
- `Exception`

并替换 `onRecover` 与 `onMotorCommand` 中重复逻辑。

效果：

- 消除多处重复的“获取资源->检查->加锁->执行->异常处理”模板
- 服务路径错误处理口径更一致
- 后续迁移到独立 `DeviceManager` 时改动面更小

### 3.3 抽取 `SafeCommand` 数值安全模块

提交：`56f9e9f`

新增文件：

- `include/can_driver/SafeCommand.h`
- `src/SafeCommand.cpp`

提供工具函数：

- `clampToInt32(...)`
- `clampToInt16(...)`
- `scaleAndClampToInt32(...)`

并在 `write()` 与 `publishMotorStates()` 中接入。

效果：

- 数值边界处理逻辑集中、可复用
- 降低控制主循环函数复杂度
- 为后续单元测试（纯函数边界）提供明确入口

### 3.4 参数化魔法数字

提交：`09f8625`

新增可配置参数（含非法值回退）：

- `direct_cmd_timeout_sec`（已存在，继续保留）
- `motor_state_period_sec`（默认 `0.1`）
- `direct_cmd_queue_size`（默认 `1`）

效果：

- 部署时无需改代码即可调节发布周期与直接命令队列长度
- 参数校验与默认回退提升运行鲁棒性

### 3.5 实时路径堆分配优化（热路径）

提交：`a4a678a`

改动：

- `read()/write()` 不再每周期构建 `std::map` 分组，改为初始化阶段构建 `jointGroups_` 并复用
- `write()` 不再每周期构造 `std::vector<JointCommand>` 与字符串拷贝
- 命令缓冲改为成员复用：
  - `rawCommandBuffer_`
  - `commandValidBuffer_`

效果：

- 消除 `write()` 热路径中的字符串拷贝与容器临时分配
- 降低 500Hz/1kHz 控制循环中的堆分配抖动风险

## 4. 文件影响清单

- `include/can_driver/CanDriverHW.h`
- `src/CanDriverHW.cpp`
- `include/can_driver/SafeCommand.h`（新增）
- `src/SafeCommand.cpp`（新增）
- `CMakeLists.txt`

## 5. 编译验证

每一轮均执行：

```bash
catkin_make --pkg can_driver
```

结果：均编译通过（`can_driver_transport`、`can_driver_node` 成功链接）。

## 6. 与初始审查项对照

已完成：

- `init()` 巨型函数拆分（函数级）
- service 重复模板抽取
- 命令安全处理抽取为独立工具模块
- 魔法数字参数化（部分）
- 热路径动态分配优化（`read/write` 分组与命令缓冲复用）

尚未完成（下一阶段）：

- `DeviceManager` 独立成单独类与文件
- `JointConfig` 解析独立成单独类与文件
- `CanDriverHW_services.cpp` 文件级拆分
- 配套单元测试补齐（重点 `SafeCommand` / 参数解析边界）

## 7. 风险与兼容性说明

- 本轮以“结构调整优先、行为保持”为原则，没有改变对外 service/topic 名称。
- 新增参数均有默认值与非法值回退，不配置时可保持旧行为。
- 由于仍在同一主类内进行部分组织，最终模块化边界尚未完全收敛；建议按下一阶段计划继续推进。
- `init()` 当前仅在 `parseAndSetupJoints(...)` 失败时做显式回滚；后续若让更多步骤返回错误码，应统一接入同一回滚策略。

## 8. 与安全修复项关系

| 安全项 | 状态 | 说明 | 提交 |
|--------|------|------|------|
| 直接命令绕过限位 | ✅ 已修复 | direct 命令在 `write()` 中同样执行限位钳制 | `19a23cf` |
| 直接命令标志不自动清除 | ✅ 已修复 | 引入 `direct_cmd_timeout_sec`，超时后回退到 ros_control 命令 | `19a23cf` |

## 9. SafeCommand 测试计划（细化）

| 函数 | 用例 | 预期 |
|------|------|------|
| `scaleAndClampToInt32` | `cmd=100, scale=1.0` | 返回 `true`，输出 `100` |
| `scaleAndClampToInt32` | `cmd=1e18, scale=1.0` | 返回 `true`，输出 `INT32_MAX` |
| `scaleAndClampToInt32` | `cmd=-1e18, scale=1.0` | 返回 `true`，输出 `INT32_MIN` |
| `scaleAndClampToInt32` | `cmd=NaN, scale=1.0` | 返回 `false` |
| `scaleAndClampToInt32` | `cmd=Inf, scale=1.0` | 返回 `false` |
| `scaleAndClampToInt32` | `cmd=1.0, scale=0.0` | 返回 `false` |
| `scaleAndClampToInt32` | `cmd=1.0, scale=-1.0` | 返回 `false` |
| `scaleAndClampToInt32` | `cmd=0.0, scale=1e-300` | 返回 `true`，输出 `0` |
| `clampToInt16` | `40000.0` | `32767` |
| `clampToInt16` | `-40000.0` | `-32768` |

## 10. 后续建议

1. 在下一轮将 `executeOnMotor` 与 protocol/transport 生命周期迁移到 `DeviceManager`。
2. 提取 `JointConfig` 解析模块，降低 `CanDriverHW` 与 `XmlRpc` 的耦合。
3. 按“测试计划（第 9 节）”补齐 `SafeCommand` GTest 用例并接入 CI。
4. 完成文件级拆分后，再评估实时循环中的日志与字符串构造成本。

## 11. 本次补充重构（增量）

提交：见本轮实际 git 提交记录（`git log --oneline`）

- 已新增 `DeviceManager`：
  - `include/can_driver/DeviceManager.h`
  - `src/DeviceManager.cpp`
  - 负责 transport/protocol/mutex 生命周期与查询
- 已新增 `JointConfigParser`：
  - `include/can_driver/JointConfigParser.h`
  - `src/JointConfigParser.cpp`
  - 负责 joint 配置解析与 `motor_id` 解析
- `CanDriverHW` 已改为调用 `DeviceManager` 与 `JointConfigParser`，降低耦合
- `onMotorCommand` 改为表驱动执行
- subscriber direct 命令回调改为工厂 lambda 去重
- 析构逻辑复用 `resetInternalState()`
- 单测补充：
  - `tests/test_safe_command.cpp`
  - `tests/test_joint_config_parser.cpp`
