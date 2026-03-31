# 实时控制 UI 使用说明

该 UI 用于拖动滑条实时发送单电机位置/速度命令，适合联调与示教。

## 功能

- 生命周期管理：
  - `Init`
  - `Init + Release`
  - `Enable / Disable`
  - `Release/Resume / Halt`
  - `Recover / Shutdown`
- 选择关节
- 切换位置/速度模式（调用 `motor_command` 的 `CMD_SET_MODE`）
- 滑条实时发送命令（拖动即发 + 可配置连续发送频率）
- 使能 / 停止 / 失能按钮
- 反馈显示会按 `position_scale / velocity_scale` 自动换算到 `rad / rad/s`

## 启动前提

1. 已启动 `can_driver_node`
2. 可访问以下接口：
   - `/can_driver_node/motor_command`
   - `/can_driver_node/motor/<joint>/cmd_position`
   - `/can_driver_node/motor/<joint>/cmd_velocity`

## 启动命令

```bash
source /home/dianhua/Robot24_catkin_ws/devel/setup.bash
rosrun can_driver realtime_motor_ui.py
```

真机联调推荐显式指定：

```bash
source /home/dianhua/Robot24_catkin_ws/devel/setup.bash
rosrun can_driver realtime_motor_ui.py _init_loopback:=false _stream_hz:=100
```

参数说明：

- `_init_loopback`
  - `false`：真机推荐
  - `true`：仅 `vcan` / 本机回环测试建议使用
- `_stream_hz`
  - 控制“连续发送”勾选后的发布频率
  - 默认跟随 `/can_driver_node/control_frequency`

## 使用建议

- 首次联调先小范围：
  - 位置范围如 `[-0.2, 0.2]`
  - 速度范围如 `[-0.5, 0.5]`
- 真机推荐先 `Init`，确认日志进入 `Armed`，再按需点 `Init + Release` 或 `Release/Resume`
- 先点击“单电机使能”，再拖动滑条
- 若关节突然不动，先点“停止”再“使能”
- 若只是想点动，不建议勾选“连续发送”
- 若要联调 `CSP`，请把 `_stream_hz` 设到接近控制频率，例如 `100` 或 `250`

## 安全注意

- UI 是直接命令控制，不做轨迹规划
- 请确保限位已配置（URDF 或 rosparam joint_limits）
- 快写模式（PP `CMD=0x05`）下建议保持较小步进与范围
- 快写模式下位置命令只写 `0x0A`，不会自动改写速度；需要时请先发送速度命令再进行位置拖动
- `motor_states` 中的 `position / velocity` 原本是协议原始单位，UI 现在会按 joint 的
  `position_scale / velocity_scale` 转成 `rad / rad/s` 后再显示
