# 达妙履带电机接入实施计划（按 Commit）

## 目标

把原 `control` 里的达妙履带电机控制链拆成两部分：

1. `can_driver` 内的达妙驱动后端与生命周期接入。
2. 独立的底盘控制节点，只保留 `cmd_vel -> wheel joint` 的底盘运动学与业务逻辑。

当前阶段只处理履带电机，控制模式限定为 `velocity`，并把达妙设备纳入 `can_driver` 的生命周期管理、共享状态和刷新调度。

## 范围边界

- 本仓库负责：
  - 达妙协议后端
  - 生命周期接入
  - 共享状态/刷新调度
  - 配置解析
  - 单元测试与烟测
- 本仓库暂不负责：
  - 底盘控制节点拆分后的新包实现
  - URDF / transmission / controller YAML 调整
  - 底盘运动学节点与上层导航联调

## 改动文件范围

- 驱动与协议接入
  - `include/can_driver/CanType.h`
  - `include/can_driver/DamiaoCan.h`
  - `include/can_driver/DeviceManager.h`
  - `include/can_driver/RefreshScheduler.h`
  - `include/can_driver/SharedDriverState.h`
  - `src/DamiaoCan.cpp`
  - `src/DeviceManager.cpp`
  - `src/JointConfigParser.cpp`
  - `src/lifecycle_driver_ops.cpp`
  - `src/CanDriverHW.cpp`
  - `CMakeLists.txt`
- 测试
  - `tests/test_joint_config_parser.cpp`
  - `tests/test_device_manager.cpp`
  - `tests/test_refresh_scheduler.cpp`
  - `tests/test_damiao_can_protocol.cpp`
  - `tests/test_can_driver_hw_smoke.cpp`

## Commit 拆分

### Commit 1

`docs(can_driver): add Damiao track integration plan`

- 新增本文档。
- 固定本轮改造边界、文件范围和提交拆分方式。

### Commit 2

`feat(can_driver): port Damiao velocity backend into lifecycle driver`

- 在 `can_driver` 内新增 `DamiaoCan` 协议后端。
- 复用旧 `control` 里速度模式的主流程：
  - 写控制模式寄存器
  - 等 ACK
  - 使能
  - 发送零速
  - 等状态反馈
- 把达妙电机纳入：
  - `DeviceManager`
  - 生命周期操作
  - 共享状态
  - 刷新调度
  - 启动同步
- 配置解析限制为：
  - `protocol: DM`
  - `control_mode: velocity`
  - `motor_id` 低字节在 `[0, 15]`

### Commit 3

`test(can_driver): cover Damiao parser protocol and startup path`

- 增加 DM 配置解析测试。
- 增加 DM 刷新调度测试。
- 增加 DM 协议编码/解码测试。
- 增加 `DeviceManager` 创建 DM 协议测试。
- 增加 `CanDriverHW` 启动同步烟测：
  - 速度语义关节在没有 `positionValid` 时，只要有有效速度/状态反馈也能完成启动。

## 实施说明

- 迁移策略优先“搬旧代码语义”，不重新设计达妙协议流程。
- 旧 `control` 中底盘节点保留在原仓库，后续只抽出底盘运动学和 ROS topic/action 接口。
- 等底盘控制节点拆出后，再做 URDF 与 wheel joint 强依赖的联调修改。
