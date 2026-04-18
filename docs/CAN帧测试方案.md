# CAN 帧解析与集成测试方案

日期：2026-03-03

## 1. 测试层次结构

```
┌─────────────────────────────────────────┐
│  Level 4: 真实硬件测试                    │  ← 最终验证
│  (真实 CAN 设备 + 真实电机)                │
└─────────────────────────────────────────┘
              ↑
┌─────────────────────────────────────────┐
│  Level 3: CanDriverHW 集成测试           │  ← 端到端逻辑
│  (Mock Transport + 模拟 CAN 帧)          │
└─────────────────────────────────────────┘
              ↑
┌─────────────────────────────────────────┐
│  Level 2: 协议层测试                      │  ← 帧编码/解码
│  (MtCan/EyouCan + Mock Transport)       │
└─────────────────────────────────────────┘
              ↑
┌─────────────────────────────────────────┐
│  Level 1: 帧转换测试                      │  ← 已完成
│  (SocketCanController 单元测试)          │
└─────────────────────────────────────────┘
```

## 2. Level 2: 协议层测试（已创建）

### 2.1 测试文件
- `tests/test_mt_can_protocol.cpp`
- `tests/test_eyou_can_protocol.cpp`（待创建）

### 2.2 测试策略

**使用 Mock Transport**：
```cpp
class MockTransport : public CanTransport {
    // 捕获发送的帧
    std::vector<Frame> sentFrames;

    // 模拟接收帧
    void simulateReceive(const Frame &frame);
};
```

**测试内容**：
1. ✅ 命令编码正确性（位置、速度、使能等）
2. ✅ 反馈解码正确性（位置、速度、电流等）
3. ✅ 边界值处理（INT32_MAX/MIN）
4. ✅ 多电机并发
5. ✅ 未知电机 ID 忽略

**优点**：
- 不需要真实硬件
- 快速执行（<1ms）
- 可以精确控制输入
- 易于调试

**局限**：
- 无法测试真实 CAN 总线行为
- 无法测试时序问题

## 3. Level 3: CanDriverHW 集成测试

### 3.1 测试文件
`tests/test_can_driver_hw_integration.cpp`

### 3.2 测试策略

**方案 A: 使用 Mock Transport（推荐）**

```cpp
TEST(CanDriverHWIntegration, WriteCommandGeneratesCorrectCANFrame)
{
    // 1. 创建 CanDriverHW，注入 Mock Transport
    // 2. 配置一个 joint（position mode, scale=0.001）
    // 3. 设置 posCmd = 1.0 rad
    // 4. 调用 write()
    // 5. 验证 Mock Transport 收到的 CAN 帧：
    //    - 帧 ID 正确
    //    - 数据 = 1.0 / 0.001 = 1000
    //    - 数据被正确编码（小端序）
}

TEST(CanDriverHWIntegration, LimitsActuallyEnforced)
{
    // 1. 配置 joint，URDF 限位 max=1.0 rad
    // 2. 设置 posCmd = 5.0 rad（超出限位）
    // 3. 调用 write()
    // 4. 验证发送的 CAN 帧数据 = 1.0 / scale（被钳制）
}

TEST(CanDriverHWIntegration, DirectCommandTimeoutFallback)
{
    // 1. 发送 direct 命令 = 2.0 rad
    // 2. 设置 ros_control 命令 = 1.0 rad
    // 3. 立即调用 write()
    // 4. 验证使用 direct 命令（2.0）
    // 5. 等待超过 timeout
    // 6. 再次调用 write()
    // 7. 验证使用 ros_control 命令（1.0）
}

TEST(CanDriverHWIntegration, ReadDecodesCANFrameCorrectly)
{
    // 1. 创建 CanDriverHW
    // 2. Mock Transport 模拟接收位置反馈帧（raw=2000）
    // 3. 调用 read()
    // 4. 验证 joint.pos = 2000 * scale
}
```

**方案 B: 使用虚拟 CAN 设备（vcan0）**

```cpp
TEST(CanDriverHWIntegration, RealCANLoopback)
{
    // 前提：系统有 vcan0
    // 1. 创建 CanDriverHW，使用 vcan0 + loopback
    // 2. 发送命令
    // 3. 用 candump 或另一个 SocketCanController 监听
    // 4. 验证收到的帧
}
```

**优点**：
- 测试真实的 CAN 栈
- 可以用 candump 验证

**缺点**：
- 需要系统配置 vcan0
- 测试环境依赖

### 3.3 实现细节

**关键问题：如何注入 Mock Transport？**

当前 CanDriverHW 内部创建 SocketCanController，无法注入 mock。

**解决方案 1：依赖注入（推荐）**

修改 CanDriverHW 构造函数：
```cpp
class CanDriverHW {
public:
    // 默认构造函数（生产环境）
    CanDriverHW() = default;

    // 测试构造函数（注入 mock）
    explicit CanDriverHW(std::shared_ptr<CanTransport> mockTransport);
};
```

**解决方案 2：使用 vcan0（不修改代码）**

```bash
# 创建虚拟 CAN 设备
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

测试时使用 vcan0，配合 loopback 模式。

## 4. Level 4: 真实硬件测试

### 4.1 测试环境

**硬件需求**：
- CAN 适配器（USB-CAN 或 PCIe-CAN）
- 至少 1 个测试电机
- 急停按钮

**软件需求**：
- candump（监控工具）
- can-utils（调试工具）

### 4.2 测试流程

**阶段 1: CAN 通信验证（30 分钟）**

```bash
# 1. 启动 can_driver_node
roslaunch can_driver can_driver.launch

# 2. 在另一个终端监控 CAN 帧
candump can0

# 3. 发送极小命令
rostopic pub /can_driver/motor/joint1/cmd_position std_msgs/Float64 "data: 0.01"

# 4. 观察 candump 输出
# 验证：
# - 帧 ID 正确
# - 数据长度正确
# - 数据值合理（0.01 / scale）
# - 无异常大的数值
```

**阶段 2: 反馈解码验证（30 分钟）**

```bash
# 1. 手动发送 CAN 帧（模拟电机反馈）
cansend can0 241#0010000000000000  # 发送位置 = 0x1000

# 2. 查看 ROS topic
rostopic echo /can_driver/motor_states

# 3. 验证解码后的位置 = 0x1000 * scale
```

**阶段 3: 限位验证（1 小时）**

```bash
# 1. 检查日志，确认限位已加载
# 应该看到：
# [CanDriverHW] Joint 'joint1': URDF limits [-1.57, 1.57] rad

# 2. 发送超出限位的命令
rostopic pub /can_driver/motor/joint1/cmd_position std_msgs/Float64 "data: 5.0"

# 3. 用 candump 验证实际发送的值被钳制到 1.57
# 4. 观察电机是否停在限位位置
```

**阶段 4: 超时机制验证（30 分钟）**

```bash
# 1. 发送 direct 命令
rostopic pub /can_driver/motor/joint1/cmd_position std_msgs/Float64 "data: 0.5"

# 2. 立即观察 candump，应该看到命令 = 0.5
# 3. 等待 > direct_cmd_timeout_sec（默认 0.5s）
# 4. 观察 candump，应该看到命令回退到 ros_control 值（可能是 0）
```

**阶段 5: 长时间稳定性测试（4 小时）**

```bash
# 1. 运行正弦波轨迹
rostopic pub /joint_trajectory_controller/command trajectory_msgs/JointTrajectory ...

# 2. 持续监控：
# - CPU 使用率
# - 内存使用（检测泄漏）
# - CAN 总线负载
# - 错误日志

# 3. 每小时检查一次电机温度
```

### 4.3 验收标准

| 测试项 | 通过标准 |
|--------|---------|
| CAN 帧格式 | 帧 ID、DLC、数据格式符合协议规范 |
| 数值编码 | 发送值 = (命令值 / scale)，误差 < 1 count |
| 数值解码 | 反馈值 = (原始值 * scale)，误差 < 0.1% |
| 限位执行 | 超限命令被钳制，电机不超限 |
| 超时回退 | 超时后自动回退到 ros_control 命令 |
| 稳定性 | 4 小时无崩溃、无内存泄漏、无异常日志 |

## 5. 测试工具

### 5.1 CAN 帧分析脚本

```python
#!/usr/bin/env python3
# scripts/analyze_can_frames.py

import can
import struct

def decode_mt_position_frame(frame):
    """解码 MT 协议位置帧"""
    if frame.arbitration_id == 0x241:
        pos = struct.unpack('<i', frame.data[:4])[0]
        print(f"Motor position: {pos} counts")
        return pos
    return None

def main():
    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    print("Listening for CAN frames...")
    for msg in bus:
        decode_mt_position_frame(msg)

if __name__ == '__main__':
    main()
```

### 5.2 自动化测试脚本

```bash
#!/bin/bash
# scripts/run_hardware_tests.sh

set -e

echo "=== CAN Driver Hardware Test Suite ==="

# 1. 检查 CAN 设备
if ! ip link show can0 &>/dev/null; then
    echo "ERROR: can0 not found"
    exit 1
fi

# 2. 启动 can_driver_node
roslaunch can_driver can_driver.launch &
DRIVER_PID=$!
sleep 5

# 3. 检查限位日志
if ! grep -q "URDF limits" /tmp/can_driver.log; then
    echo "WARNING: No URDF limits found!"
fi

# 4. 发送测试命令
rostopic pub -1 /can_driver/motor/joint1/cmd_position std_msgs/Float64 "data: 0.01"
sleep 1

# 5. 验证 CAN 帧（使用 candump）
timeout 2 candump can0 | grep "241" || echo "No feedback received"

# 6. 清理
kill $DRIVER_PID

echo "=== Test Complete ==="
```

## 6. 测试优先级

### 高优先级（本周完成）
1. ✅ Level 2: 协议层测试（test_mt_can_protocol.cpp）
2. ⚠️ Level 3: CanDriverHW 集成测试（需要依赖注入）
3. ⚠️ Level 4: 真实硬件 CAN 通信验证

### 中优先级（下周完成）
1. Level 4: 限位验证
2. Level 4: 超时机制验证
3. Level 2: EyouCan 协议测试

### 低优先级（下月完成）
1. Level 4: 长时间稳定性测试
2. 性能测试（延迟、吞吐量）
3. 压力测试（高频命令）

## 7. 风险与缓解

| 风险 | 缓解措施 |
|------|---------|
| Mock Transport 与真实行为不一致 | Level 4 真实硬件测试验证 |
| 协议规范理解错误 | 与电机厂商确认协议文档 |
| 测试环境缺少 vcan0 | 提供自动化脚本创建 vcan0 |
| 真实硬件测试危险 | 渐进式测试 + 急停按钮 |

## 8. 总结

**当前状态**：
- Level 1: ✅ 完成
- Level 2: ⚠️ 部分完成（已创建 MT 协议测试框架）
- Level 3: ❌ 缺失（需要依赖注入支持）
- Level 4: ❌ 缺失（需要真实硬件）

**建议行动**：
1. 完善 test_mt_can_protocol.cpp（填充实际协议规范）
2. 创建 test_eyou_can_protocol.cpp
3. 修改 CanDriverHW 支持依赖注入
4. 创建 test_can_driver_hw_integration.cpp
5. 在测试台上进行 Level 4 测试

**预计时间**：
- Level 2 完成：2 天
- Level 3 完成：3 天
- Level 4 初步验证：1 天
- Level 4 完整测试：1 周

**上真机前最低要求**：
- Level 2 全部通过
- Level 3 核心场景通过（限位、超时）
- Level 4 CAN 通信验证通过
