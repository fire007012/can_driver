# can_driver 使用指南

本文档面向 `can_driver` 包本身，包含：安装/启动、参数配置、单电机控制、控制器加载、MoveIt/Navi 接入与排障。

## 1. 包定位

`can_driver` 是 ROS1 `ros_control` 硬件层节点，负责把上层控制（MoveIt / Navi / 手动命令）转换成 CAN 电机协议命令。

当前实现支持：
- MT 协议电机（典型用于底盘轮）
- PP 协议电机（典型用于机械臂轴）
- 按 joint 配置混合使用两个协议

---

## 2. 关键文件

- `can_driver/launch/can_driver.launch`
- `can_driver/config/can_driver.yaml`
- `can_driver/config/ros_controllers.yaml`
- `can_driver/src/can_driver_node.cpp`
- `can_driver/src/CanDriverHW.cpp`

---

## 3. 编译与启动

```bash
cd /home/dianhua/catkin_ws
catkin_make --pkg can_driver
source devel/setup.bash
roslaunch can_driver can_driver.launch
```

---

## 4. 配置说明

## 4.1 `config/can_driver.yaml`

用途：定义关节到电机映射与协议信息。

每个 joint 至少需要：
- `name`
- `motor_id`
- `protocol` (`MT` / `PP`)
- `can_device` (`can0` / `can1` / `vcan0` ...)
- `control_mode` (`velocity` / `position`)

可选：
- `position_scale`
- `velocity_scale`

注意：
- `name` 必须和 URDF joint 名一致。
- `control_mode` 决定该 joint 对应的硬件接口类型。

## 4.2 `config/ros_controllers.yaml`

用途：定义可用控制器。

当前定义了：
- `joint_state_controller`
- `arm_controller` (`JointTrajectoryController`)
- `wheel_controller` (`DiffDriveController`)

## 4.3 `launch/can_driver.launch`

用途：加载参数、启动 `can_driver_node`、通过 spawner 启动控制器。

当前默认行为：
- 仅启动 `joint_state_controller`
- `arm_controller` 和 `wheel_controller` 在注释块里，需要你手动启用

---

## 5. 对外接口

## 5.1 Services

- `/can_driver_node/init`
- `/can_driver_node/shutdown`
- `/can_driver_node/recover`
- `/can_driver_node/motor_command`

`/can_driver_node/motor_command`（`can_driver/MotorCommand`）命令常量：
- `CMD_ENABLE=0`
- `CMD_DISABLE=1`
- `CMD_STOP=2`
- `CMD_SET_MODE=3` (`value: 0/1`)

## 5.2 Topics

- `/can_driver_node/motor_states` (`can_driver/MotorState`)
  - 发布频率：10 Hz
  - 已实现字段：`motor_id`, `name`, `position`, `velocity`, `current`, `mode`
  - 未实现字段：`enabled`, `fault`（预留，当前版本未填充）
- `/can_driver_node/motor/<joint>/cmd_velocity` (`std_msgs/Float64`)
- `/can_driver_node/motor/<joint>/cmd_position` (`std_msgs/Float64`)
- `/joint_states`（由 `joint_state_controller` 发布，频率见 `ros_controllers.yaml` 中的 `publish_rate: 50`）

---

## 6. 单电机控制（最小闭环）

以 `left_wheel (motor_id=321)` 为例：

1. 使能
```bash
rosservice call /can_driver_node/motor_command "{motor_id: 321, command: 0, value: 0.0}"
```

2. 下发速度
```bash
rostopic pub -1 /can_driver_node/motor/left_wheel/cmd_velocity std_msgs/Float64 "data: 20.0"
```

3. 查看状态
```bash
rostopic echo /can_driver_node/motor_states
```

4. 停止
```bash
rosservice call /can_driver_node/motor_command "{motor_id: 321, command: 2, value: 0.0}"
```

5. 失能
```bash
rosservice call /can_driver_node/motor_command "{motor_id: 321, command: 1, value: 0.0}"
```

---

## 7. MoveIt / Navi 怎么接入

是否“能执行轨迹/导航”，由三层共同决定：

1. `can_driver.yaml`：joint 接口类型（position/velocity）
2. `ros_controllers.yaml`：控制器定义
3. `can_driver.launch` 的 spawner `args`：实际启动了哪些控制器

结论：
- 你只启动 `joint_state_controller` 时，MoveIt/Navi 一般无法驱动执行。
- 启动 `arm_controller` 才能承接机械臂轨迹。
- 启动 `wheel_controller` 才能承接 `/cmd_vel` 底盘控制。

检查命令：
```bash
rosservice call /controller_manager/list_controllers
```

---

## 8. 与 ros_canopen 体感对齐度（当前）

抛开协议差异，目前是：
- 架构接近（`hardware_interface + controller_manager`）
- 单轴 direct 控制可用
- 但还没完全达到“无差别体感”

主要差距：
- 默认 controller 启动不完整
- MoveIt 控制器映射还需要按工程补齐
- 状态机/恢复/命令仲裁仍在可靠性补强阶段

---

## 9. 开发者选项

### 9.1 软件回环模式

用途：无需真实 CAN 硬件即可测试 ros_control 控制器逻辑。

启用方法：
1. 在 `CanDriverHW.cpp` 中定义宏 `#define SOFTWARE_LOOPBACK_MODE 1`
2. 重新编译 `catkin_make --pkg can_driver`

行为：
- `write()` 不发送 CAN 帧，命令值直接写入 `posCmd/velCmd`
- `read()` 直接读取 `posCmd/velCmd` 作为反馈（完美跟随）
- 适用于测试 MoveIt 轨迹规划、controller_manager 加载等上层逻辑

注意：
- 此模式下电机不会真实运动
- 仅用于开发调试，生产环境必须关闭（`#define SOFTWARE_LOOPBACK_MODE 0`）

---

## 10. 常见排障

1. 启动即失败
- 检查 `can_device` 是否存在且 up
- 检查 `motor_id/protocol/control_mode` 是否拼写正确

2. 有 `/joint_states` 但电机不动
- 只说明状态链路可用，不代表执行链路已加载
- 用 `list_controllers` 确认 `arm_controller` / `wheel_controller` 是否 running

3. 命令发了没反应
- 先 Enable
- 检查是否有其它节点同时控制同一轴
- 检查 `candump canX` 是否有帧

4. 接口找不到
- 先 `rosservice list` / `rostopic list`，以运行时名称为准

---

## 10. 相关文档

- `can_driver/docs/can_driver_single_motor_usage.md`：偏 UI/单轴实操
- `can_driver/docs/can_driver_plan_reliability.md`：可靠性路线图
- `can_driver/docs/concurrency_safety_report.md`：并发安全修复报告
