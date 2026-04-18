# ECB 网络电机接入与排障指南

本文档用于 1MINTASCA/Innfos ECB 网络电机在 can_driver 中的配置、验证与排障。

## 1. 兼容结论

当前 can_driver 已支持 ECB 协议，接入路径是“协议层接入”，不是 UDP-CAN 透传。

- 已兼容：Enable/Disable/Stop、位置模式、速度模式、状态读取（position/velocity/current）
- 已兼容：固定 IP + ID、自动扫描（lookup）
- 已兼容：与 MT/PP 混合配置（按 joint 的 protocol 分流）

注意：ECB 底层虽然是网络通信（SDK 内部包含以太网/UDP 实现），但上层应通过 SDK API 控制，不应复用 UdpCanTransport 的 CAN 帧语义。

## 2. 配置模板

在 `config/can_driver.yaml` 或 `config/can_driver_ecb.yaml` 的 `joints` 内添加 ECB 关节。生产/联调都建议直接使用固定 IP：

```yaml
- name: ecb_joint_02
  motor_id: 0x02
  protocol: ECB
  can_device: ecb://192.168.1.30
  ecb_ip: 192.168.1.30
  ecb_discovery: fixed
  ecb_refresh_ms: 20
  control_mode: position
  position_scale: 6.283185307179586e-4
  velocity_scale: 1.0471975511965978e-2
```

如果现场有多台电机，建议每台都写成独立固定 IP 条目。当前推荐的配置形态如下：

```yaml
- name: ecb_joint_02
  motor_id: 0x02
  protocol: ECB
  can_device: ecb://192.168.1.30
  ecb_ip: 192.168.1.30
  ecb_discovery: fixed
  ecb_refresh_ms: 20
  control_mode: position
  position_scale: 6.283185307179586e-4
  velocity_scale: 1.0471975511965978e-2

- name: ecb_joint_03
  motor_id: 0x03
  protocol: ECB
  can_device: ecb://<discover_ip_of_03>
  ecb_ip: <discover_ip_of_03>
  ecb_discovery: fixed
  ecb_refresh_ms: 20
  control_mode: position
  position_scale: 6.283185307179586e-4
  velocity_scale: 1.0471975511965978e-2

- name: ecb_joint_04
  motor_id: 0x04
  protocol: ECB
  can_device: ecb://<discover_ip_of_04>
  ecb_ip: <discover_ip_of_04>
  ecb_discovery: fixed
  ecb_refresh_ms: 20
  control_mode: position
  position_scale: 6.283185307179586e-4
  velocity_scale: 1.0471975511965978e-2

- name: ecb_joint_05
  motor_id: 0x05
  protocol: ECB
  can_device: ecb://<discover_ip_of_05>
  ecb_ip: <discover_ip_of_05>
  ecb_discovery: fixed
  ecb_refresh_ms: 20
  control_mode: position
  position_scale: 6.283185307179586e-4
  velocity_scale: 1.0471975511965978e-2
```

说明：

- 当前已确认固定 IP：`ID 2 -> 192.168.1.30`
- `ID 3/4/5` 的固定 IP 不要手填猜测值，请先通过发现脚本拿到现场实际结果再写入
- `scripts/discover_ecb_and_enable_yaml.sh` 现在会把所有扫到的 ECB 电机一起回填成固定 IP 条目，不再只写单台

## 3. 单位与角度问题（重点）

### 3.1 内部原始单位

ECB 协议实现采用固定点原始单位：

- 位置 raw：1 raw = 1e-4 rev
- 速度 raw：1 raw = 0.1 rpm
- 电流 raw：1 raw = 1 mA

### 3.2 推荐 scale（上层 SI 单位）

为了让上层接口继续用 rad / rad/s，请使用：

- position_scale = 2*pi/10000 = 6.283185307179586e-4
- velocity_scale = (2*pi/60)/10 = 1.0471975511965978e-2

含义：

- raw * position_scale = rad
- raw * velocity_scale = rad/s

### 3.3 常见角度问题

1. 小角度命令几乎不动
- 典型原因：position_scale 仍是 1.0 或误配为 2*pi（整圈粒度）。

2. 速度明显偏大/偏小
- 典型原因：velocity_scale 未按 0.1rpm 粒度配置。

3. 位置模式只第一次有效，后面像“失效”
- ECB 的 `cmd_position` 是绝对位置命令，不是相对增量。
- 若脚本每次都发同一个绝对目标，看起来就会像“只第一次能动”。
- 当前 `scripts/test_ecb_motor_motion.sh` 已改为：先读取当前位置，再按“当前位置 ± 偏移量”的方式做位置测试。

4. 模式切换后仍按旧模式表现
- 先确认 service `CMD_SET_MODE` 已成功，再检查是否有其它节点持续向相反 topic 下发命令。

## 4. 一键测试脚本

新增脚本：scripts/test_ecb_motor_motion.sh

在不知道电机 ID/IP 时，先执行自动发现并回填 YAML：

```bash
scripts/discover_ecb_and_enable_yaml.sh 1
```

说明：

- 参数 `1` 表示选择第 1 台被发现的电机；如果有多台，可改为 `2/3/...`
- 脚本会调用 SDK 的 `01_lookupActuators` 自动扫描，并把 `config/can_driver.yaml` 和 `config/can_driver_ecb.yaml` 中 `ECB_AUTO_PROBE` 段一起改为固定 IP + ID
- 若现场同时在线多台 ECB，会一次性回填多条 `ecb_joint_02/03/04/05...`
- 回填后优先使用 `roslaunch can_driver can_driver_ecb.launch` 重启驱动再进行运动测试

示例：

```bash
roslaunch can_driver can_driver_ecb.launch
bash scripts/test_ecb_motor_motion.sh 2 0.8 2.0 1.2 2.0 /can_driver_node
bash scripts/test_ecb_motor_motion.sh 3 0.8 2.0 1.2 2.0 /can_driver_node
bash scripts/test_ecb_motor_motion.sh 4 0.8 2.0 1.2 2.0 /can_driver_node
bash scripts/test_ecb_motor_motion.sh 5 0.8 2.0 1.2 2.0 /can_driver_node
```

参数：

- motor_id|auto：目标电机 ID 或自动挑选首个 ECB 关节
- vel_rad_s：速度命令（rad/s）
- vel_hold_s：速度保持时间（秒）
- pos_rad：相对位置偏移量（rad）。脚本会以当前位置为基准计算绝对目标值
- pos_hold_s：位置保持时间（秒）
- driver_ns：驱动命名空间（默认 /can_driver_node）

环境变量：

- ECB_TEST_STREAM_HZ：位置/速度流式下发频率（默认 20）
- ECB_TEST_SEND_DISABLE=1：结束时自动 Disable

## 5. 排障清单

1. 服务不可达
- 检查 `rosservice list` 是否存在 `/can_driver_node/motor_command`
- 若你当前只接 ECB，优先用 `roslaunch can_driver can_driver_ecb.launch`

2. 命令发出但无动作
- 先 Enable，再 CMD_SET_MODE，再发 topic 命令
- 检查 joint 配置 protocol 是否 ECB，motor_id 是否匹配
- 检查 ecb_discovery 与 ecb_ip 是否正确
- 若速度模式能动、位置模式像“只动一次”，先确认你发的是相对位移脚本版本，而不是反复下发同一个绝对位置

3. 自动扫描慢或失败
- 优先改为 fixed IP + ID
- 关注网络拓扑、交换机、防火墙
- 若发现脚本提示 `Connected error code:803`：表示 SDK 与 ECB 通信失败，优先检查网段、防火墙、网口连通与电机上电状态
- 若同时出现 `No available device`：通常是主机网卡与 ECB 默认网段不一致（常见 ECB 为 `192.168.1.x`，而主机在其它网段）
  - 临时加同网段地址后重试：`sudo ip addr add 192.168.1.100/24 dev <your_nic>`
  - 重试发现：`scripts/discover_ecb_and_enable_yaml.sh`
  - 测试后删除临时地址：`sudo ip addr del 192.168.1.100/24 dev <your_nic>`
- 若在虚拟机内调试（VMware/VirtualBox/KVM）：确认网卡为桥接模式（bridged），NAT 模式通常无法透传 ECB 发现依赖的二层广播
- 若脚本提示 `not in pcap group`：执行 `newgrp pcap` 或重新登录后再重试

4. 只发现/只控制到 ID 2，看不到 3/4/5
- 先确认 `scripts/discover_ecb_and_enable_yaml.sh` 的输出里是否真的扫到了 `3/4/5`
- 若没扫到，优先检查：
  - 3/4/5 电机是否已上电
  - 3/4/5 是否与主机处于同一二层网络
  - 3/4/5 是否仍保留默认网段或已被改过 IP
- 若已扫到但未写入配置，重新执行发现脚本；当前版本会把所有已发现 ECB 一起回填

5. 脚本报错 no ECB joint found in joints config
- 含义：脚本已读取到 joints 参数，但其中没有任何 `protocol: ECB` 条目（或读到了其他命名空间的 joints）。
- 检查：
  - `rosparam get /can_driver_node/joints | grep protocol`
  - 若你的节点不在 `/can_driver_node`，按实际命名空间替换。
- 修复：
  - 在 `config/can_driver.yaml` 的 `can_driver_node.joints` 下新增至少一个 ECB 条目；
  - 重新 `roslaunch can_driver can_driver.launch`；
  - 再执行脚本：`scripts/test_ecb_motor_motion.sh auto 0.8 2.0 1.2 2.0 /can_driver_node`

6. 状态刷新抖动
- 增大 ecb_refresh_ms（如 20 -> 50）
- 降低控制频率和测试流式频率

7. 故障位持续触发
- 观察 /can_driver_node/motor_states 与日志
- 根据 SDK 错误码排查（如 IP 冲突、设备离线）

## 6. 运行建议

- 生产环境优先 fixed IP + ID，避免每次启动扫描带来的不确定性。
- 仅接 ECB 时优先使用 `can_driver_ecb.launch`，避免被 `can0/can1` 的 MT/PP 配置连带启动失败。
- 首次联调建议先速度小幅正反，再做小角度位置闭环，再放大位移。
- 保持单节点独占控制，避免多个节点同时写同一 joint 的 direct topic。
