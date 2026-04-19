# can_driver 体积与职责拆分报告

日期：2026-04-01

## 1. 结论摘要

`can_driver` 现在已经不是一个“单纯的 CAN 驱动包”。

如果只统计代码文件，并排除 `tests/`，当前大约有：

- 代码文件数：`41`
- 物理行数：`10498`
- 近似有效代码行数（去掉空行和纯注释）：`8768`

这个体量本身不算离谱，但它已经明显超过“一个单协议驱动包”的自然规模。  
真正的问题不是“行数大”，而是**职责叠加太多**：

- ROS hardware 接口在这里
- 生命周期主链在这里
- device 级调度与背压处理在这里
- SocketCAN 传输在这里
- 两套协议实现也都在这里
- SharedState / freshness / readiness 判据还在这里
- 调试 UI 也在这里

所以它看起来像一个驱动包，实际上已经长成了一个“小型运行时系统”。

## 2. 当前体积分布

本次统计口径：

- 包路径：`can_driver`
- 统计目录：`include/`、`src/`、`scripts/`
- 统计后缀：`.h`、`.hpp`、`.cpp`、`.py`
- 排除：`tests/`、`docs/`、`config/`、`launch/`

### 2.1 按目录

- `src/`：`6406` 行
- `include/`：`3245` 行
- `scripts/`：`847` 行

### 2.2 按职责分组

- 协议与传输：`3538` 行
- ROS 硬件入口与主流程：`2135` 行
- 生命周期与动作编排：`1684` 行
- Device 层调度与运行时：`1650` 行
- 调试 UI：`847` 行
- 共享状态与判据：`644` 行

这说明当前最大头并不是某一个“巨无霸文件”，而是**多条主链叠加在同一个包里**。

## 3. 大文件观察

当前最大的几个文件如下：

- `src/CanDriverHW.cpp`：`1242` 行
- `src/EyouCan.cpp`：`1105` 行
- `src/MtCan.cpp`：`1078` 行
- `scripts/realtime_motor_ui.py`：`847` 行
- `src/DeviceManager.cpp`：`710` 行
- `src/lifecycle_driver_ops.cpp`：`559` 行
- `include/can_driver/CanDriverIoRuntime.h`：`434` 行
- `include/can_driver/SharedDriverState.h`：`427` 行

这些文件变大的原因并不完全一样：

- `CanDriverHW.cpp` 大，是因为它同时承担了 ROS 硬件层、参数加载、joint 语义转换、生命周期入口协作、初始化和写入路径。
- `EyouCan.cpp` / `MtCan.cpp` 大，是因为协议层不只做编解码，还做了 refresh、timeout、retry、backoff、shared state 同步。
- `DeviceManager.cpp` 大，是因为它同时负责 protocol 装配、refresh worker、设备健康同步、调度状态维护。
- `realtime_motor_ui.py` 大，是因为调试 UI 直接承担了参数回退、服务探测、实时控制面板、联动逻辑等功能。

所以这里不是“某个文件写得有点长”这么简单，而是**层间边界不够硬**。

## 4. 为什么会长到一万行

### 4.1 包里塞了两套协议

当前同时包含：

- `EyouCan` 私有 PP 协议
- `MtCan` 多圈协议

这意味着协议层天然翻倍，而且每套协议都不是纯编解码，而是带运行时逻辑。

### 4.2 生命周期链路已经成型

当前已经存在：

- `OperationalCoordinator`
- `LifecycleServiceGateway`
- `LifecycleDriverOps`
- `command_gate`
- `motor_action_executor`

这套东西本质上已经接近一个“控制平面”。

### 4.3 Device 运行时单独长出一层

当前又有：

- `DeviceManager`
- `DeviceRuntime`
- `DeviceRefreshWorker`
- `RefreshScheduler`

这已经不是传统“小驱动类直接发包”的结构，而是设备级运行时。

### 4.4 共享状态和判据也在包内

当前还把：

- `SharedDriverState`
- `AxisReadinessEvaluator`
- 若干 freshness / intent / mode / enabled 判据

一起放在包内部，导致“协议事实”和“生命周期判据”互相靠得很近。

### 4.5 UI 没有外置

`realtime_motor_ui.py` 自己就接近 `850` 行。  
如果把它视为工具链的一部分而不是驱动核心的一部分，当前体积的感受会立刻轻很多。

## 5. 现在的体积是否合理

要分两层看。

### 5.1 如果把它当“单个 CAN 驱动包”

那就偏大，而且偏复杂。

原因是：

- 协议
- 调度
- 生命周期
- 硬件抽象
- 调试工具

都揉在一起了。

### 5.2 如果把它当“意优私有协议控制运行时”

那现在这个量级是能解释的。

也就是说，**问题不是它不该有这些代码，而是这些代码目前都堆在一个包名下面**。  
这会导致两个后果：

- 新人一眼看不出主链在哪
- 修改任何一层时，都容易误伤相邻层

## 6. 哪些部分是核心必需，哪些部分适合拆

### 6.1 核心必需

这些东西无论如何都要有：

- `CanDriverHW`
- `DeviceManager / DeviceRuntime`
- `SocketCanController`
- `EyouCan`
- `SharedDriverState`
- `OperationalCoordinator` 主链

这些构成了真正的运行时主干。

### 6.2 第一优先适合拆出去的

#### 调试 UI

`scripts/realtime_motor_ui.py`

建议：

- 继续留在仓库里
- 但从“驱动核心”认知中剥离
- 文档上明确它是工具，不是 runtime 核心

这是最低风险、收益最高的认知拆分。

#### MT 协议

当前 `MtCan` 自成体系，和 PP 的运行方式差异也很大。

建议：

- 先逻辑分层
- 后续再决定是否单独拆包或单独成库

这样做能明显减轻 `can_driver` 主线心智负担。

### 6.3 第二优先适合拆层的

#### 生命周期控制平面

当前这部分已经很像一个独立层：

- service gateway
- coordinator
- driver ops
- command gate

建议：

- 先保持同包
- 但目录和命名上明确收成 `lifecycle/` 或独立静态库

这样后面继续对齐 `Eyou_Canopen_Master` 会更顺。

#### 判据与共享状态

`SharedDriverState` 和 `AxisReadinessEvaluator` 应该更明确地定位为：

- 事实缓存层
- 判据评估层

而不是继续向“第二套轴状态机”生长。

## 7. 我建议的拆分路线

### 阶段 1：先做“认知拆分”，不改外部接口

目标：

- 不改 ROS 接口
- 不改 YAML 语义
- 不改 launch 使用方式
- 只是把内部目录与模块边界收硬

建议结果：

- `runtime/`：`DeviceManager`、`DeviceRuntime`、`RefreshScheduler`
- `protocol/pp/`：`EyouCan`
- `protocol/mt/`：`MtCan`
- `transport/`：`SocketCanController`、`CanTxDispatcher`
- `lifecycle/`：`OperationalCoordinator`、`LifecycleDriverOps`、`command_gate`
- `state/`：`SharedDriverState`、`AxisReadinessEvaluator`
- `tools/`：`realtime_motor_ui.py`

这一步做完，行数不会变少很多，但理解难度会明显下降。

### 阶段 2：再做“编译单元拆分”

目标：

- 把大 cpp 文件拆小
- 降低修改时的耦合面

优先拆分对象：

- `CanDriverHW.cpp`
- `EyouCan.cpp`
- `MtCan.cpp`
- `DeviceManager.cpp`

其中最值得先拆的是：

- `CanDriverHW.cpp`
- `EyouCan.cpp`

因为它们既大，又处在最常改的热路径上。

### 阶段 3：最后考虑“包级拆分”

这一步要谨慎，因为会影响 catkin 依赖和外部使用方式。

如果未来真的要拆包，我更倾向于：

- `can_driver_runtime`
- `can_driver_protocol_pp`
- `can_driver_protocol_mt`
- `can_driver_tools`

但这不是当前的第一优先级。  
当前更重要的是先把**单生命周期主链**和**协议运行时边界**稳定下来。

## 8. 体积收缩的现实预期

如果只是做“认知拆分”和“大文件拆小”，总行数不会骤降。  
但从维护体验上会有很大改善。

如果未来把 UI 和 MT 协议从主认知里剥离，`can_driver` 的“核心主线体感”大致会从：

- 当前约 `1.05` 万行

收缩成：

- 面向 PP 单生命周期主链的核心代码约 `6000` 到 `7000` 行量级

这就会更接近一个可控的驱动运行时核心。

## 9. 最终判断

我的判断是：

- 这个包现在“代码量偏大”是真的
- 但它并不是无意义地膨胀
- 真正的问题是**多条职责主链共居一个包，边界表达还不够强**

换句话说：

- 它不是“烂到不可收拾”
- 也不是“一个普通小驱动包”
- 它更像一个已经成型、但还没彻底整理好的专用运行时

所以后续正确方向不是简单删代码，而是：

1. 保住单生命周期主链
2. 收紧协议层与运行时边界
3. 把 UI、MT、生命周期控制平面从认知上剥开
4. 再按热路径拆大文件

这样做，后面才会越改越顺，而不是继续越补越胖。
