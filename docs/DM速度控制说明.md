# DM 速度控制边界与配置

DM 通过 `can_driver_node`（standalone）或 `can_driver/CanDriverSystem`（ros2_control）接入，协议实现为 `DamiaoCan`。支持 velocity；逻辑 position 现映射为 MIT，需显式配置 `dm_mit_kp`、`dm_mit_kd`（参见下文）。CSP及0x100+ID的位置速度模式未实现。

## 单位与反馈路由

速度输入为 rad/s。后端缓存位置、速度使用定点 SI：1 raw = 0.0001 rad / rad/s；不是电机编码器脉冲数。两个 ROS2 入口对 DM 默认采用此比例，并拒绝不匹配的显式 scale 及不受支持的模式。`direction_sign` 可以为 +1 或 −1。

示例只说明配置格式；ID、Master ID 和接口名必须匹配实际设备：

```yaml
can_driver_node:
  control_frequency: 100.0
  motor_query_hz: 20.0
  direct_cmd_timeout_sec: 0.2
  debug_bypass_ros_control: false
  joints:
    - name: dm_motor
      motor_id: 0x07
      master_id: 0x08
      protocol: DM
      can_device: can0
      control_mode: velocity
      position_scale: 0.0001
      velocity_scale: 0.0001
      direction_sign: 1.0
```

`master_id` 是电机反馈帧的 CAN ID；驱动按它接收，不会改写电机 Master ID。历史履带配置为 7→8、5→6，不能据此认定新接入设备使用同样参数。

## 初始化与反馈

速度模式初始化写控制模式寄存器 10=3，等待写应答后读取该寄存器，要求实际读回 3；MIT位置模式使用1，并先读取0x50精确位置。随后只读寄存器 21/22/23（PMAX/VMAX/TMAX），按各轴实际量程解码反馈；不再覆盖这些寄存器。配置应答期限为500 ms。模式确认后先发送中性帧并设置Hold intent，在失能态取得真实反馈。读回超时、非有限值、非正值或超过内部数值存储边界时初始化失败。没有执行保存或零点写入。

普通状态帧不携带模式寄存器。`mode_valid` 表示本次初始化已读回确认配置模式，并收到状态反馈；不代表持续轮询模式或可检测其他主站随后修改模式。量程未确认时位置、速度及力矩反馈不标记有效。standalone 状态话题使用共享反馈有效标志，最近状态接收超过 500 ms 时 `feedback_fresh=false`；有效性与新鲜度是两个不同字段。

DM 的 `current` 历史字段实际承载力矩（0.01 Nm/raw），不是电流安培；本次保持接口兼容。

## 停止与失败处理

速度帧为标准 ID `0x200 + motor_id`、4 字节小端 float32(rad/s)。周期 keepalive 会重复缓存速度，不是只读查询。轴处于运行意图时，驱动后台以不超过 20 ms 的周期重复发送缓存速度，避免电机配置了通信丢失保护时因控制器偶发漏周期而自动失能；使能/恢复阶段使用 50 ms 周期，失能和停止继续发送零速。

standalone 的 direct 命令超时后调用 Stop，把后端缓存速度清零，并发送零速；即使开启 debug bypass 也执行该期限。发送失败保留超时状态，后续控制周期重试。期限必须为有限正数。该期限只适用于 standalone direct 命令，不替代 ros2_control 的控制器安全策略。

DM 控制/恢复帧等待发送完成，最多 100 ms；队列丢弃、链路错误和超时返回失败，失败的速度命令把缓存清零。Stop 的发送成功不等于电机已经停转，必须检查真实速度。使能/失能还要求请求发出后的新状态反馈，不能用先前缓存状态判定成功。配置通过寄存器应答确认，keepalive 仍异步发送。

上述软件期限依赖进程和线程继续运行；没有验证进程退出、主机失效或总线断开时电机自动停止。CAN 的 BUS-OFF 恢复和实机运动效果需单独验证。


## MIT 位置控制

DM 的 `control_mode: position` 映射为 MIT 模式（寄存器10=1、帧ID=电机ID），不是内置位置速度模式。必须显式提供 `dm_mit_kp`（500/4095至500）和 `dm_mit_kd`（5/4095至5），保证量化后非零。示例5和0.5尚未在DM-J10010L-2EC上整定验证。目标单位rad，V_des、T_ff为0；需要上层渐变轨迹和实际速度监测，MIT自身并非硬速度限制。

初始化按V1.1说明书读取0x50精确位置；位置反馈与输出轴角度的关系需在实机确认。使能先发送零增益中性帧；Stop/超时把缓存改为零增益中性帧，释放驱动力，不代表制动或位置保持。后续必须检查实际速度并失能。节点低字节限0–15，Master ID限标准11位。详见 [型号说明书](DM-J10010L-2EC减速电机说明书V1.1.md)。


本次DM-J10010L-2EC实机采用MIT命令Kp=200、Kd=4完成约±90°往返（编码器实测+89.742°/−89.616°），不代表其他负载适用同样增益。实际量程为PMAX12.5、VMAX25、TMAX200；必须每轴读回，不能作为所有DM的默认参数。原电机TIMEOUT为0，本次没有修改或验证电机自主失联保护。
