# CSP 模式使用指南

## 什么是 CSP 模式

CSP（Cyclic Synchronous Position，周期同步位置模式）是一种高频位置控制模式：

- 上层控制器以固定周期（如 4ms）持续发送位置目标
- 使用快写命令（CMD=0x05），无需等待返回
- 多轴起步时间差 < 10ms（相比 PP 模式的 100~300ms）
- 轨迹控制权在上层控制器，而非电机内部

## 配置方法

在 `can_driver.yaml` 中设置：

```yaml
can_driver_node:
  # 建议使用 250Hz 控制频率（4ms 周期）
  control_frequency: 250.0

  joints:
    - name: rotary_table
      motor_id: 0x16
      protocol: PP          # 协议类型：PP（EyouCan）或 MT，与 control_mode 独立
      can_device: can0
      control_mode: csp     # 运动控制模式：position / velocity / csp
      position_scale: 9.587379924285257e-05
      velocity_scale: 9.587379924285257e-05

    - name: larger_arm
      motor_id: 0x05
      protocol: PP
      can_device: can0
      control_mode: csp
```

> **`protocol` 与 `control_mode` 的关系**：
> - `protocol` 指定底层通信协议（`PP` = EyouCan，`MT` = MtCan），决定 CAN 帧格式。
> - `control_mode` 指定运动控制策略（`position` / `velocity` / `csp`），决定 `write()` 调用哪个接口。
> - 两个字段独立配置。CSP 模式目前仅支持 `protocol: PP`（EyouCan 实现了 `quickSetPosition()`）。
>
> **启动行为**：驱动初始化时会自动对所有 `control_mode: csp` 的关节调用
> `setMode(MotorMode::CSP)`，无需手动发送模式切换命令。
> 若任意 CSP 关节的 `setMode` 失败，驱动将**中止初始化**并打印 ERROR 日志，
> 以防止电机在错误模式下运动。

## 控制周期要求

CSP 模式建议使用 250Hz（4ms）控制周期：

```xml
<param name="control_frequency" value="250" />
```

较低的频率（如 100Hz）也可以工作，但同步精度会降低。

## 适用场景

- 需要多轴精确同步的应用
- 需要上层完全控制轨迹的场景
- 对起步时间一致性要求高的任务
- 需要高频位置更新的应用

## 与 PP 模式的对比

| 特性 | PP 模式（position） | CSP 模式（csp） |
|------|-------------------|----------------|
| 轨迹生成 | 电机内部 | 上层控制器 |
| 命令类型 | CMD=0x01（写入命令，需返回） | CMD=0x05（快写命令，无需返回） |
| 每轴帧数 | 2~4 帧 | 1 帧 |
| 控制周期 | 不固定 | 固定高频（如 4ms） |
| 多轴同步精度 | 100~300ms | < 10ms |
| 总线效率 | 低 | 高 |

## 注意事项

1. **协议支持**：CSP 模式目前仅支持 PP（EyouCan）协议，MT 协议暂不支持
2. **控制频率**：建议使用 250Hz，确保平滑控制
3. **总线负载**：CSP 模式下总线占用率会增加，4 轴 @ 250Hz ≈ 1000 帧/秒
4. **初始化**：驱动启动时会自动对 CSP 关节下发模式切换命令（`setMode(CSP)`）；
   在此之前，仍需确保电机已使能（通过 `MotorCommand` 服务的 `CMD_ENABLE`），
   并根据需要通过 `setVelocity` 配置目标速度上限

## 示例

完整的 CSP 模式配置示例：

```yaml
can_driver_node:
  control_frequency: 250.0
  motor_query_hz: 100.0
  pp_fast_write_enabled: true

  joints:
    - name: joint1
      motor_id: 0x01
      protocol: PP
      can_device: can0
      control_mode: csp
      position_scale: 9.587379924285257e-05

    - name: joint2
      motor_id: 0x02
      protocol: PP
      can_device: can0
      control_mode: csp
      position_scale: 9.587379924285257e-05
```

## 故障排查

### 电机不响应 CSP 命令

1. 确认电机固件支持 CSP 模式（模式 5）
2. 驱动启动时会自动调用 `setMode(CSP)`，可在日志中确认：
   `[CanDriverHW] applyInitialModes: joint 'xxx' set to CSP mode.`
   若看到 WARN 说明 setMode 失败，需检查 CAN 连接和电机状态
3. 确认电机已使能：`Enable(motorId)`

### 多轴同步精度不理想

1. 提高控制频率到 250Hz
2. 检查总线负载，避免其他设备占用带宽
3. 确认所有轴都使用 CSP 模式

### 总线过载

1. 降低控制频率（如 100Hz）
2. 减少轮询频率（motor_query_hz）
3. 考虑使用多个 CAN 总线分散负载
