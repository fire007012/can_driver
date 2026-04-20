# ECB 联调测试方案

## 目标

本文用于记录当前 `can_driver` 下 ECB 电机联调的推荐流程，重点覆盖：

- ECB 网络接口配置和发现方式
- 单电机控制时如何兼容安全层
- 速度/位置模式测试流程
- 多电机联调时的启动、分组和风险点
- 常见故障和日志判断

## 网络接入

ECB 通过以太网通信。常见默认网段为 `192.168.1.x`，主机必须有一个真实有线网口处于同网段。

### 1. 确认网卡和链路

先看主机网卡：

```bash
ip -4 -brief addr
ip link
```

如果某个网口显示 `DOWN`，即使已经配置了 `192.168.1.100/24`，也不代表链路可用。常见原因是网线未插、接错网口、交换机/ECB 未上电，或该网口不是实际连接 ECB 的网口。

可以尝试拉起网口：

```bash
sudo ip link set eno1 up
ip -4 -brief addr show dev eno1
```

如果仍然是 `DOWN`，优先换实际有线口或检查物理链路。

### 2. 临时配置 ECB 网段地址

假设实际连接 ECB 的网口是 `eno1`：

```bash
sudo ip addr add 192.168.1.100/24 dev eno1
sudo ip link set eno1 up
```

测试结束后可清理：

```bash
sudo ip addr del 192.168.1.100/24 dev eno1
```

如果脚本提示推荐网口为 `eno1`，优先按提示配置。不要把地址加到未插线或不是 ECB 链路的网口上。

### 3. SDK logtool 权限

首次发现 ECB 前，建议设置 SDK 抓包权限：

```bash
cd ~/catkin_ws/src/can_driver
sudo sh lib/innfos-cpp-sdk/tools/logtool/linux_x86_64/setpath.sh
newgrp pcap
getcap lib/innfos-cpp-sdk/tools/logtool/linux_x86_64/ActuatorLogTool
```

期望包含：

```text
cap_net_admin,cap_net_raw+eip
```

### 4. 发现电机并回填配置

```bash
cd ~/catkin_ws/src/can_driver
bash scripts/discover_ecb_and_enable_yaml.sh 1
```

成功时会看到类似：

```text
[ECB-DISCOVER] discovered 2 192.168.1.30
[ECB-DISCOVER] discovered 3 192.168.1.30
[ECB-DISCOVER] discovered 4 192.168.1.30
[ECB-DISCOVER] discovered 5 192.168.1.30
```

如果出现 `Connected error code:803` 或 `No available device`，优先检查：

- 主机是否真的有网口在 `192.168.1.0/24`
- 实际连接 ECB 的网口是否 `UP`
- ECB 是否上电
- 是否有 VPN/虚拟网卡抢占默认抓包设备
- 是否已经执行 `setpath.sh` 和 `newgrp pcap`

## 网络配置模式

ECB joint 支持两种发现策略。

### 固定 IP 模式

推荐现场测试优先使用固定 IP：

```yaml
protocol: ECB
can_device: ecb://192.168.1.30
ecb_ip: 192.168.1.30
ecb_discovery: fixed
```

优点：

- 启动快
- 日志明确
- 多电机同一 ECB 控制器时更稳定

单电机 launch 默认就是固定 IP：

```bash
roslaunch /home/rera/catkin_ws/src/can_driver/launch/can_driver_ecb_single.launch \
  joint_name:=ecb_joint_03 \
  motor_id:=0x03 \
  can_device:=ecb://192.168.1.30 \
  ecb_ip:=192.168.1.30 \
  ecb_discovery:=fixed
```

### 自动发现模式

如果现场 IP 不确定，可使用：

```yaml
can_device: ecb://auto
ecb_discovery: auto
```

或启动时覆盖：

```bash
roslaunch /home/rera/catkin_ws/src/can_driver/launch/can_driver_ecb_single.launch \
  can_device:=ecb://auto \
  ecb_ip:= \
  ecb_discovery:=auto
```

自动发现依赖 SDK 扫描，受网卡、权限、广播链路影响更大。正式联调确认 IP 后，建议切回固定 IP。

## 启动驱动

### 单电机联调

单电机调试优先使用：

```bash
roslaunch /home/rera/catkin_ws/src/can_driver/launch/can_driver_ecb_single.launch \
  joint_name:=ecb_joint_03 \
  motor_id:=0x03 \
  can_device:=ecb://192.168.1.30 \
  ecb_ip:=192.168.1.30
```

单电机 launch 只加载一个 joint，可以避免未接入的其它电机把整机 lifecycle 拉到 `Faulted`。

### 多电机联调

多电机联调使用标准 launch 和 `config/can_driver.yaml` 中的 ECB joints：

```bash
roslaunch /home/rera/catkin_ws/src/can_driver/launch/can_driver.launch
```

确认 `config/can_driver.yaml` 中只保留现场真实接入的电机。不要加载未上电、未联网或 ID 不存在的 ECB 轴，否则 lifecycle 健康检查会失败。

## 安全层兼容原则

当前驱动不建议关闭安全层。ECB 联调应在以下开关保持开启的基础上完成：

```yaml
safety_stop_on_fault: true
safety_require_enabled_for_motion: true
lifecycle_require_enabled_for_running: true
safety_hold_after_device_recover: true
```

### lifecycle 状态

控制命令只有在 `/can_driver_node/lifecycle_state` 为 `Running` 时才会下发：

```bash
rostopic echo /can_driver_node/lifecycle_state
```

典型状态含义：

- `Standby`：已初始化但未 enable
- `Armed`：已 enable，可准备 release/resume
- `Running`：允许运动命令下发
- `Faulted`：存在故障或健康检查失败，运动命令会被阻断

如果进入 `Faulted`，使用正式 recover 流程：

```bash
rosservice call /can_driver_node/recover "{motor_id: 65535}"
rosservice call /can_driver_node/enable
rosservice call /can_driver_node/resume
```

不要通过关闭 `safety_*` 参数来绕过故障。

### 模式切换顺序

ECB 模式切换必须同时兼容两套约束：

- 安全层要求：切模式前先进入非 Running/可控状态，避免运动中直接切模式。
- ECB SDK 行为：实际运动前需要在 enable 后激活 profile mode。

推荐顺序：

1. `halt`，让 lifecycle 从 `Running` 回到 `Armed`
2. `Disable`
3. `Set mode`
4. `Enable`
5. 驱动在 `Enable()` 后重新激活 ECB profile mode
6. `resume`，回到 `Running`
7. 先发对齐命令，再发目标运动命令

`scripts/test_ecb_motor_motion.sh` 已经内置该顺序。

### 对齐命令

安全层在模式切换、recover、设备恢复后会要求 fresh command，避免旧命令在恢复瞬间误动作。

手工发 topic 时也要遵守：

- 速度模式：先发当前实际速度，通常接近 `0`
- 位置模式：先发当前位置
- 确认稳定后再发真正目标值

测试脚本会自动读取 `/can_driver_node/motor_states` 并完成对齐。

### fault 处理

当 ECB 上报 fault，驱动会：

1. 自动发送一次 `Stop`
2. 阻断后续运动命令
3. 将 lifecycle 拉到 `Faulted`

日志示例：

```text
[InnfosEcb] motor_id=3 fault error_code=0x00000200 (ERR_STEP_OVER).
[CanDriverHW] Joint 'ecb_joint_03' has fault, auto Stop sent and motion command blocked.
```

`ERR_STEP_OVER` 通常表示位置目标跳变过大或绝对多圈位置不合适。处理方式是减小位置步长、先回到当前位置对齐、必要时重新设零和限位。

## 单电机测试

启动单电机 launch 后运行：

```bash
cd ~/catkin_ws/src/can_driver
bash scripts/test_ecb_motor_motion.sh 3 1.0 2.0 1.0 2.0 /can_driver_node
```

参数含义：

- `3`：motor_id
- `1.0`：速度命令，单位 `rad/s`
- `2.0`：速度保持时间
- `1.0`：位置偏移，单位 `rad`
- `2.0`：位置保持时间
- `/can_driver_node`：驱动命名空间

确认方向和比例正确后，可以逐步增大：

```bash
bash scripts/test_ecb_motor_motion.sh 3 4.0 2.0 6.0 2.0 /can_driver_node
```

观察状态：

```bash
rostopic echo /can_driver_node/motor_states
```

重点看：

- `enabled`
- `fault`
- `mode`
- `position`
- `velocity`
- `current`
- `feedback_fresh`

## 多电机联调

### 基本原则

多电机联调前，先逐台完成单电机测试。确认每台电机都满足：

- SDK 可发现
- `motor_id` 与配置一致
- 单独启动时能进入 `Running`
- 速度模式能正反转
- 位置模式不会频繁触发 `ERR_STEP_OVER`
- `motor_states` 中反馈刷新稳定

### 分组测试

默认按两组测试：

```bash
bash scripts/test_ecb_group_motion.sh 1.0 2.0 1.0 2.0 /can_driver_node
```

默认组：

```text
2,3
4,5
```

指定分组：

```bash
export ECB_TEST_GROUPS="2,3"
bash scripts/test_ecb_group_motion.sh 1.0 2.0 1.0 2.0 /can_driver_node
```

如果要四台一起测试：

```bash
export ECB_TEST_GROUPS="2,3,4,5"
bash scripts/test_ecb_group_motion.sh 1.0 2.0 1.0 2.0 /can_driver_node
```

### 多电机注意事项

1. 所有加载进驱动的 ECB joint 都必须真实在线。

   多电机 lifecycle 是整体健康检查。任意一个轴无反馈、未 enabled、fault 或 feedback degraded，都可能让系统从 `Running` 自动进入 `Faulted`。

2. 固定 IP 优先。

   多电机如果挂在同一个 ECB 控制器，通常配置为同一个 `ecb_ip`，不同 `motor_id`。建议使用：

   ```yaml
   can_device: ecb://192.168.1.30
   ecb_ip: 192.168.1.30
   ecb_discovery: fixed
   ```

3. 先小幅度、低速度。

   多电机第一次联调建议使用：

   ```bash
   bash scripts/test_ecb_group_motion.sh 1.0 2.0 1.0 2.0 /can_driver_node
   ```

   确认方向和反馈后再增大命令。

4. 注意位置绝对值。

   ECB 位置接口是多圈位置，日志中的：

   ```text
   setPosition motor_id=3 raw=-549008 target=-54.9008 rev
   ```

   表示目标绝对位置为 `-54.9008` 圈。即使相对偏移不大，如果当前多圈零点长期漂移，也可能触发 `ERR_STEP_OVER` 或限位类问题。

5. 注意 profile 参数。

   单电机 launch 支持：

   ```xml
   ecb_profile_position_max_rpm
   ecb_profile_position_acceleration_rpm_s
   ecb_profile_position_deceleration_rpm_s
   ecb_profile_velocity_acceleration_rpm_s
   ecb_profile_velocity_deceleration_rpm_s
   ```

   多电机联调时应保证 profile 不过激，先用保守加速度和速度上限。

6. 观察电流。

   如果 `raw` 正常、`enabled=True`、`fault=False`、`mode` 正常，但 `velocity=0` 且 `current` 几乎不变，优先检查功率电源、抱闸、机械卡滞或电机侧限制。

## 推荐联调顺序

1. 配置有线网口到 ECB 网段。
2. 执行 `setpath.sh` 并确认 `ActuatorLogTool` capability。
3. 运行 `scripts/discover_ecb_and_enable_yaml.sh 1`。
4. 单电机 launch 启动目标轴。
5. 确认 `/can_driver_node/lifecycle_state == Running`。
6. 执行单电机小幅度测试。
7. 逐台完成所有电机单测。
8. 切换到多电机配置，只保留真实在线的 joints。
9. 先按 `2,3`、`4,5` 分组测试。
10. 最后再做全组同步测试。

## 常见问题

### 1. SDK 发现失败

现象：

```text
Connected error code:803
No available device.
```

处理：

```bash
ip -4 -brief addr
sudo ip addr add 192.168.1.100/24 dev eno1
sudo ip link set eno1 up
bash scripts/discover_ecb_and_enable_yaml.sh 1
```

如果网口仍是 `DOWN`，检查物理链路。

### 2. 启动后 `Startup feedback sync timed out`

说明驱动没有收到目标轴的启动反馈。检查：

- `ecb_ip` 是否正确
- `motor_id` 是否存在
- `ecb_discovery` 是否与现场一致
- 是否只有目标电机在线但配置加载了更多电机

### 3. 模式正常、raw 正常，但电机不转

先看是否有：

```text
[InnfosEcb] re-activate mode after enable motor_id=... mode=Mode_Profile_...
```

如果没有，说明当前运行的不是最新编译版本，重新：

```bash
cd ~/catkin_ws
catkin_make --pkg can_driver
source devel/setup.bash
```

如果有这行，再看 `motor_states` 中 `enabled/fault/mode/velocity/current`。无 fault 且电流不变化时，优先检查电机侧功率和机械条件。

### 4. `ERR_STEP_OVER`

说明 ECB 认为位置阶跃过大。处理：

- 减小位置测试幅度
- 先发当前位置对齐命令
- 避免直接从很大的绝对多圈位置跳到另一个大位置
- 必要时通过维护工具重新设零和限位

### 5. 多电机一启动就 `Faulted`

通常是配置中加载了未在线或未 enabled 的轴。处理：

- 单电机调试用 `can_driver_ecb_single.launch`
- 多电机调试前确认所有 `joints` 都真实在线
- 暂时只保留当前测试组的 ECB joints

