# can_driver

`can_driver` 是基于 SocketCAN 的 ROS2 电机驱动包，负责把私有 CAN 协议设备接入 `ros2_control`，并提供生命周期服务、直连调试接口、协议适配和测试工具。当前同时覆盖 `MT`、`PP`、`DM` 和 `ECB` 后端。

## 包结构

```text
can_driver/
|-- action/
|   |-- HomeMotor.action
|   `-- MoveMotor.action
|-- config/
|   |-- can_driver_arm_only.yaml
|   |-- can_driver.yaml
|   |-- mt_control_profiles.yaml
|   |-- can_driver_tracks_only.yaml
|   |-- ros2_controllers.yaml
|   `-- ros2_controllers_tracks_only.yaml
|-- docs/
|   |-- README.md
|   `-- 归档/
|-- launch/
|   |-- can_driver.launch.py
|   `-- tracks_only.launch.py
|-- msg/
|   `-- MotorState.msg
|-- scripts/
|   |-- realtime_motor_ui.py
|   |-- mt_motor_interface.py
|   `-- test_*.sh
|-- src/
|   |-- can_driver_node.cpp
|   |-- CanDriverHW.cpp
|   |-- CanDriverRuntime.cpp
|   |-- driver_ros_endpoints.cpp
|   |-- lifecycle_service_gateway.cpp
|   `-- *_protocol.cpp
|-- srv/
|   |-- Init.srv
|   |-- Recover.srv
|   |-- Shutdown.srv
|   |-- MotorCommand.srv
|   `-- SetZero*.srv / ApplyLimits.srv
`-- tests/
    `-- test_*.cpp
```

## 快速开始

编译：

```bash
cd /home/fire/ros2_ws_car
source /opt/ros/humble/setup.bash
colcon build --packages-select can_driver
source install/setup.bash
```

默认启动是 standalone 驱动节点，不自动进入 ros2_control 控制链：

```bash
ros2 launch can_driver can_driver.launch.py
```

仅启动履带：

```bash
ros2 launch can_driver tracks_only.launch.py
```

如果要一并加载控制器：

```bash
ros2 launch can_driver can_driver.launch.py \
  use_ros2_control:=true \
  start_controllers:="joint_state_broadcaster arm_controller wheel_controller"
```

## 常用命令

查看生命周期和状态：

```bash
ros2 service list | grep can_driver_node
ros2 topic echo /can_driver_node/motor_states
ros2 topic echo /can_driver_node/lifecycle_state
```

标准生命周期：

```bash
ros2 service call /can_driver_node/init can_driver/srv/Init "{device: canable0, loopback: false}"
ros2 service call /can_driver_node/enable std_srvs/srv/Trigger "{}"
ros2 service call /can_driver_node/disable std_srvs/srv/Trigger "{}"
ros2 service call /can_driver_node/halt std_srvs/srv/Trigger "{}"
ros2 service call /can_driver_node/resume std_srvs/srv/Trigger "{}"
ros2 service call /can_driver_node/recover can_driver/srv/Recover "{motor_id: 65535}"
ros2 service call /can_driver_node/shutdown can_driver/srv/Shutdown "{force: false}"
```

直连单电机调试：

```bash
ros2 topic pub --once /can_driver_node/motor/left_wheel/cmd_velocity std_msgs/msg/Float64 '{data: 30.0}'
ros2 topic pub --once /can_driver_node/motor/arm_joint_1/cmd_position std_msgs/msg/Float64 '{data: 0.2}'
```

维护服务：

```bash
ros2 service call /can_driver_node/motor_command can_driver/srv/MotorCommand "{motor_id: 1, command: 3, value: 2}"
ros2 service call /can_driver_node/set_zero can_driver/srv/SetZero "{motor_id: 1, zero_offset_rad: 0.0, use_current_position_as_zero: false, apply_to_motor: false}"
ros2 service call /can_driver_node/set_zero_limit can_driver/srv/SetZeroLimit "{motor_id: 1, zero_offset_rad: 0.0, use_current_position_as_zero: false, min_position_rad: -1.0, max_position_rad: 1.0, use_urdf_limits: false, apply_to_motor: false}"
```

## 接口速查

节点：

- `can_driver_node`
  - 入口：`src/can_driver_node.cpp`
  - 主体：`CanDriverSystem + controller_manager + can_driver_node + runtime/services/endpoints`

默认订阅：

- `~motor/<joint>/cmd_velocity` `std_msgs/msg/Float64`
- `~motor/<joint>/cmd_position` `std_msgs/msg/Float64`

默认发布：

- `~motor_states` `can_driver/msg/MotorState`
- `~lifecycle_state` `std_msgs/msg/String`

默认服务：

- `~init`
- `~enable`
- `~disable`
- `~halt`
- `~resume`
- `~recover`
- `~shutdown`
- `~motor_command`
- `~set_zero`
- `~apply_limits`
- `~set_zero_limit`

关键消息 / 服务：

- `msg/MotorState.msg`
- `srv/MotorCommand.srv`
- `srv/Init.srv`
- `srv/Recover.srv`
- `srv/Shutdown.srv`

关键参数：

- `config/can_driver.yaml`
  - 默认全量 can_driver 电机配置
- `config/can_driver_tracks_only.yaml`
  - 履带专用硬件配置
- `config/can_driver_arm_only.yaml`
  - 机械臂后半段专用硬件配置
- `config/ros2_controllers.yaml`
  - 默认控制器定义
- `config/ros2_controllers_tracks_only.yaml`
  - 履带专用控制器定义
- `launch/can_driver.launch.py`
  - 是否加载控制器、启动哪些控制器
- `launch/tracks_only.launch.py`
  - 履带专用启动入口

## 使用边界

- 默认 launch 以 standalone 模式启动，不自动进入 ros2_control 控制链。
- `direct motor` 话题适合单电机排障，不应替代正式 ros2_control 控制链。
- `MoveMotor.action` / `HomeMotor.action` 目前只保留为消息合同，不是默认 launch 的主要操作入口。
- ROS1 旧 launch 和 `CanDriverHW` 源码只作为迁移对照，当前入口以 ROS2 launch 和 `CanDriverSystem` 为准。

## 文档入口

- [`docs/README.md`](docs/README.md)
  - 当前文档导航与归档入口
- 推荐先读：
  - `docs/配置文件字段详解与从零配置指南.md`
  - `docs/使用指南.md`
  - `docs/电机配置变更指南.md`
  - `docs/达妙履带接入说明.md`
