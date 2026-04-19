# can_driver 重构后项目形态总览

日期：2026-04-01

## 1. 现在它是什么

这一轮重构之后，`can_driver` 的定位已经收敛为：

- 一个意优私有 CAN 电机主站运行时
- 一个单主站、单生命周期的电机管理系统
- 一个支持多协议后端的统一控制平面

它不再适合被理解成：

- “只负责发包收包的底层驱动”
- 或“PP 一套状态机、MT 一套状态机、上面再套一层系统状态机”

## 2. 当前主链

当前主链可以用下面这条链理解：

`ROS/service/topic -> CanDriverHW -> OperationalCoordinator -> LifecycleDriverOps -> DeviceManager -> DeviceRuntime/RefreshScheduler -> EyouCan/MtCan -> SharedDriverState`

其中：

- `CanDriverHW`
  - ROS 接入层
  - 负责参数、topic、service、ros_control 对接
- `OperationalCoordinator`
  - 唯一系统生命周期状态机
  - 负责 `Inactive/Configured/Armed/Running/Faulted` 等系统状态
- `LifecycleDriverOps`
  - 把系统动作翻译成设备动作
  - 聚合轴级准入判据
- `DeviceManager`
  - 管理 device、transport、protocol、refresh runtime
- `DeviceRuntime`
  - 统一一个 device 的发包所有权与背压处理
- `RefreshScheduler`
  - 负责 PP/MT 的字段级刷新计划
- `EyouCan/MtCan`
  - 各自负责协议编解码与共享事实回填
- `SharedDriverState`
  - 只保存共享事实，不再混入系统生命周期

## 3. 当前最重要的收敛结果

### 3.1 只剩一套系统生命周期

现在真正有状态名解释权的对象只有：

- `OperationalCoordinator`

轴级层不再继续向“第二套状态机”扩张。

### 3.2 SharedDriverState 已收回到“事实缓存”

`SharedDriverState` 当前主要只保存：

- 轴反馈事实
- 轴命令事实
- 轴 intent
- 设备健康

已经移除了无实际消费者、或容易污染边界的派生/全局状态。

### 3.3 Recover 的跨帧确认回到生命周期层

`AxisReadinessEvaluator` 现在只负责“判据计算”。

真正跨帧累计 `Recover` 健康样本的是：

- `LifecycleDriverOps`

这让“判据层”和“生命周期编排层”的职责重新分开。

### 3.4 PP 刷新已经是 deadline 模型

PP 的刷新已不是旧的 `cycle % N` 轮转逻辑，而是：

- 每轴每字段独立维护 deadline
- 分离 due / issued / response 三个阶段
- 压力模式只限制单轮预算，不再饿死字段
- stale warning 改为 missed windows 语义

### 3.5 startup probe 与 steady refresh 已分层

当前刷新语义是：

- `motor_query_hz`
  - 全局 steady 默认频率
- `startup_probe_query_hz`
  - 初始化阶段的 startup probe 频率
- 每个 device
  - 可以临时覆盖 refresh rate
  - 初始化结束后回退到全局 steady 默认

这意味着 startup 不会再粗暴地切全局轮询频率。

## 4. 当前测试形态

测试现在分成两类：

- 普通 `gtest`
  - 纯逻辑、纯协议、纯状态模型测试
- `rostest`
  - 依赖 ROS graph 的 smoke / integration test

`hw smoke` 已经迁到 `rostest`。

正确运行命令：

```bash
source ~/Robot24_catkin_ws/devel/setup.bash
rostest can_driver test_can_driver_hw_smoke.test
```

## 5. 当前项目还不是什么

即便这一轮已经收敛很多，它现在仍然不是：

- 完全重写后的最终版主站
- 与 `Eyou_Canopen_Master` 完全同构的所有细节都已经一比一对齐
- 已经把所有 MT/PP 差异都抽象到最优边界

更准确地说，它现在已经达到的是：

- 主体架构方向正确
- 主链边界清楚
- 关键运行时语义已从混乱态收敛到可维护态

## 6. 用一句话总结当前形态

现在的 `can_driver`，已经从“能跑但层次容易串味的混合驱动”，收敛成了：

- 一套单生命周期主站
- 一层共享事实缓存
- 一层轴级判据评估
- 一套按 device 组织的刷新运行时
- 两个协议后端 `PP/MT`

也就是：

- 主站是一个
- 生命周期是一套
- 协议后端可以有多个
