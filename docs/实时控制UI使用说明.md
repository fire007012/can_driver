# 实时控制 UI 使用说明

该 UI 用于拖动滑条实时发送单电机位置/速度命令，适合联调与示教。

## 功能

- 选择关节
- 切换位置/速度模式（调用 `motor_command` 的 `CMD_SET_MODE`）
- 滑条实时发送命令（拖动即发 + 可选 20Hz 连续发送）
- 使能 / 停止 / 失能按钮

## 启动前提

1. 已启动 `can_driver_node`
2. 可访问以下接口：
   - `/can_driver_node/motor_command`
   - `/can_driver_node/motor/<joint>/cmd_position`
   - `/can_driver_node/motor/<joint>/cmd_velocity`

## 启动命令

```bash
source /home/fire/catkin_ws/devel/setup.bash
rosrun can_driver realtime_motor_ui.py
```

## 使用建议

- 首次联调先小范围：
  - 位置范围如 `[-0.2, 0.2]`
  - 速度范围如 `[-0.5, 0.5]`
- 先点击“使能”，再拖动滑条
- 若关节突然不动，先点“停止”再“使能”

## 安全注意

- UI 是直接命令控制，不做轨迹规划
- 请确保限位已配置（URDF 或 rosparam joint_limits）
- 快写模式（PP `CMD=0x05`）下建议保持较小步进与范围
- 快写模式下位置命令只写 `0x0A`，不会自动改写速度；需要时请先发送速度命令再进行位置拖动
