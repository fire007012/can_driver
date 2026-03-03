# can_driver

ROS1 `hardware_interface::RobotHW` implementation for SocketCAN motor control.

Supported protocols:
- `MT`
- `PP` (Eyou)

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
- `direct_cmd_queue_size` (default: `1`)

## Interfaces

- Topic sub: `~motor/<joint>/cmd_velocity` (`std_msgs/Float64`)
- Topic sub: `~motor/<joint>/cmd_position` (`std_msgs/Float64`)
- Topic pub: `~motor_states` (`can_driver/MotorState`)
- Service: `~init`
- Service: `~shutdown`
- Service: `~recover`
- Service: `~motor_command`

## More Docs

- `docs/USAGE.md`
- `docs/code_structure_refactor_report_2026-03-03.md`
