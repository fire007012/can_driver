# ECB 四电机联调指南

本文档说明如何在 `can_driver` 包内接入并联调 4 台 ECB 电机，当前目标电机 `motor_id` 固定为 `2/3/4/5`。

适用场景：

- 现场仅接 ECB 网络电机，不依赖 `can0/can1`
- 四台 ECB 已完成固定 IP 规划
- 希望在同一个驱动节点下统一启动、统一测试

## 1. 包内接入方式

当前仓库统一使用标准入口：

- 配置文件：`config/can_driver.yaml`
- 启动文件：`launch/can_driver.launch`
- 单电机测试：`scripts/test_ecb_motor_motion.sh`
- 四电机顺序测试：`scripts/test_ecb_four_motors.sh`
- 分组同时测试：`scripts/test_ecb_group_motion.sh`

是否启用 ECB 由 joint 的 `protocol: ECB` 决定；若当前机器只接 ECB，请在标准 `can_driver.yaml` 中保留 ECB joints，并移除/注释未接入的非 ECB joints。

## 2. 四电机配置约定

当前四台 ECB 电机已经按统一 joint 命名接入：

```yaml
can_driver_node:
  joints:
    - name: ecb_joint_02
      motor_id: 0x02
      protocol: ECB
      can_device: ecb://192.168.1.30
      ecb_ip: 192.168.1.30
      ecb_discovery: fixed
      control_mode: position

    - name: ecb_joint_03
      motor_id: 0x03
      protocol: ECB
      can_device: ecb://192.168.1.30
      ecb_ip: 192.168.1.30
      ecb_discovery: fixed
      control_mode: position

    - name: ecb_joint_04
      motor_id: 0x04
      protocol: ECB
      can_device: ecb://192.168.1.30
      ecb_ip: 192.168.1.30
      ecb_discovery: fixed
      control_mode: position

    - name: ecb_joint_05
      motor_id: 0x05
      protocol: ECB
      can_device: ecb://192.168.1.30
      ecb_ip: 192.168.1.30
      ecb_discovery: fixed
      control_mode: position
```

说明：

- 当前配置假设这 4 台 ECB 挂在同一个网络端点 `192.168.1.30` 下，通过不同 `motor_id` 区分。
- 若现场固定 IP 与此不同，只需要修改 `config/can_driver.yaml` 中对应 joint 的 `can_device/ecb_ip`。
- `position_scale` 与 `velocity_scale` 已按 ECB 原始单位换算为 SI 单位，测试脚本直接使用 `rad / rad/s`。

## 3. 启动流程

先确保主机与 ECB 处于同一网段。若 ECB 使用 `192.168.1.x`，而主机网卡没有该网段地址，可临时加一个：

```bash
sudo ip addr add 192.168.1.100/24 dev enp2s0
```

启动标准 can_driver：

```bash
roslaunch can_driver can_driver.launch
```

确认服务已就绪：

```bash
rosservice list | grep /can_driver_node/motor_command
```

## 4. 单电机联调

逐台测试时，可直接指定 `motor_id`：

```bash
bash scripts/test_ecb_motor_motion.sh 2 0.8 2.0 1.2 2.0 /can_driver_node
bash scripts/test_ecb_motor_motion.sh 3 0.8 2.0 1.2 2.0 /can_driver_node
bash scripts/test_ecb_motor_motion.sh 4 0.8 2.0 1.2 2.0 /can_driver_node
bash scripts/test_ecb_motor_motion.sh 5 0.8 2.0 1.2 2.0 /can_driver_node
```

说明：

- 脚本会依次执行 `Enable -> 速度模式 -> 正反速度 -> Stop -> 位置模式 -> 相对位置往返 -> Final stop`
- 位置测试已按“当前位置 ± 偏移量”执行，避免 ECB 绝对位置命令导致的“只第一次有效”假象

## 5. 四电机顺序联调

新增的一键脚本会按 `2 -> 3 -> 4 -> 5` 顺序逐台测试：

```bash
bash scripts/test_ecb_four_motors.sh 0.8 2.0 1.2 2.0 /can_driver_node
```

默认参数含义：

- `0.8`：速度命令，单位 `rad/s`
- `2.0`：速度保持时间，单位 `s`
- `1.2`：相对位置偏移量，单位 `rad`
- `2.0`：位置保持时间，单位 `s`
- `/can_driver_node`：驱动命名空间

可选环境变量：

```bash
ECB_TEST_MOTOR_IDS="2 3 4 5" ECB_TEST_PAUSE_SEC=1.5 \
bash scripts/test_ecb_four_motors.sh 0.6 1.5 0.8 1.5 /can_driver_node
```

适用场景：

- 想快速确认 4 台电机都能完成模式切换与运动
- 想做现场整包回归验证
- 想在修改 IP/线缆/交换机后做统一复测

## 6. 分组同时联调

新增脚本支持“组内同时转动、组间顺序测试”。默认分组如下：

- `2,3`
- `4,5`

直接执行：

```bash
bash scripts/test_ecb_group_motion.sh 8.0 2.0 12.0 2.0 /can_driver_node
```

脚本行为：

- 组内电机先全部 `Enable`
- 组内电机全部切到速度模式
- 组内电机同步正转、同步反转
- 组内电机全部 `Stop`
- 组内电机全部切到位置模式
- 组内电机以各自当前位置为基准，同步做相对位置往返

如果你想改成“4 台同时转”，可以覆盖默认分组：

```bash
ECB_TEST_GROUPS="2,3,4,5" \
bash scripts/test_ecb_group_motion.sh 8.0 2.0 12.0 2.0 /can_driver_node
```

如果你想按别的分组方式测试，也可以自定义：

```bash
ECB_TEST_GROUPS="2,4 3,5" ECB_TEST_GROUP_PAUSE_SEC=1.5 \
bash scripts/test_ecb_group_motion.sh 8.0 2.0 12.0 2.0 /can_driver_node
```

## 7. 包内控制建议

若目标是“方便一起控制”，建议遵循下面的包内组织方式：

- 统一使用 `config/can_driver.yaml` 管理 ECB 四电机配置
- 统一使用 `launch/can_driver.launch` 启动 ECB 驱动
- 单电机问题先用 `scripts/test_ecb_motor_motion.sh` 排查
- 整体回归再用 `scripts/test_ecb_four_motors.sh`
- 分组并发验证用 `scripts/test_ecb_group_motion.sh`

这样做的好处是：

- 配置结构统一，ECB/DM/PP/MT 都由 joint 的 `protocol` 决定后端
- 只要保持 `can_driver.yaml` 与当前机器真实接线一致，就不会被无关设备配置拖累
- 四台电机共享同一套 scale、刷新周期和安全策略
- 单机排障与多机回归的入口清晰

## 8. 常见问题

1. `motor_command` 服务不存在

- 先确认使用的是 `roslaunch can_driver can_driver.launch`
- 再看启动日志里是否出现设备初始化失败

2. 某一台能动，另一台不能动

- 先单独执行对应 `motor_id` 的 `test_ecb_motor_motion.sh`
- 再检查该 `motor_id` 是否已经在 `config/can_driver.yaml` 中配置

3. 四台都配置了，但只有一台响应

- 确认现场 ECB 端是否真的允许通过不同 `motor_id` 区分 2/3/4/5
- 确认固定 IP `192.168.1.30` 对应的是整组执行器，而不是单个执行器

4. 位置模式看起来只转一次

- 这是 ECB 绝对位置语义带来的常见误判
- 当前脚本已经改成相对偏移测试，优先使用仓库里的最新版脚本
