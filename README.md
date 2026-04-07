# can_driver

ROS1 `hardware_interface::RobotHW` implementation for SocketCAN motor control.

Supported protocols:
- `MT`
- `PP` (Eyou)

Supported control modes:
- `position` - Position control (PP mode)
- `velocity` - Velocity control (PV mode)
- `csp` - Cyclic Synchronous Position mode (high-frequency position control for multi-axis synchronization)

## Build

```bash
cd /home/dianhua/catkin_ws
catkin_make --pkg can_driver
```

## Run

```bash
source /home/dianhua/catkin_ws/devel/setup.bash
roslaunch can_driver can_driver.launch
```

## Key Runtime Params

- `direct_cmd_timeout_sec` (default: `0.5`)
- `motor_state_period_sec` (default: `0.1`)
- `motor_query_hz` (default: `0.0`, protocol auto strategy)
- `direct_cmd_queue_size` (default: `1`)
- `debug_bypass_ros_control` (recommended: `false`, debugging only; check your loaded YAML because local configs may override it)

## Interfaces

- Topic sub: `~motor/<joint>/cmd_velocity` (`std_msgs/Float64`)
- Topic sub: `~motor/<joint>/cmd_position` (`std_msgs/Float64`)
- Topic pub: `~motor_states` (`can_driver/MotorState`, feedback-only semantics with `*_valid` and `feedback_fresh`)
- Service: `~init`
- Service: `~shutdown`
- Service: `~recover`
- Service: `~enable`
- Service: `~disable`
- Service: `~halt`
- Service: `~resume`
- Service: `~motor_command` (`CMD_SET_MODE=3`, value `0=position 1=velocity 2=csp`)

## More Docs

- `docs/文档导航.md`
- `docs/架构设计.md`
- `docs/使用指南.md`
- `docs/CAN帧测试方案.md`
- `docs/测试计划.md`
- `docs/单电机控制说明.md`
- `docs/归档/代码结构重构报告_2026-03-03.md`
