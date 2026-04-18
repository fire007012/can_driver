# CSP 模式实施路线图（按 Commit）

**目标**: 在 can_driver 中实现 CSP（周期同步位置模式），将多轴起步时间差从 100~300ms 降低到 < 10ms

**实施方案**: 方案 A（混合方案）- 保留当前架构，自实现 CSP 模式

**预计工作量**: 2-3 天，7 个 commits

---

## Commit 1: feat(protocol): add CSP mode enum and interface

**目标**: 在协议抽象层增加 CSP 模式的枚举和接口定义

**修改文件**:
- `include/can_driver/CanProtocol.h`

**具体改动**:

```cpp
// 在 ControlMode 枚举中增加 CSP
enum class ControlMode {
    Position = 0,  // PP 模式（轮廓位置模式）
    Velocity = 1,  // PV 模式（轮廓速度模式）
    CSP = 5        // CSP 模式（周期同步位置模式）
};

// 在 CanProtocol 类中增加快写位置接口
class CanProtocol {
public:
    // ... 现有接口 ...

    /**
     * @brief 快写位置命令（用于 CSP 模式）
     * @param motorId 电机 ID
     * @param position 目标位置（角度，单位：度）
     * @return 成功返回 true，失败返回 false
     * @note 使用 CMD=0x05 快写命令，无需等待返回确认
     */
    virtual bool quickSetPosition(uint8_t motorId, double position) = 0;
};
```

**验证方式**:
```bash
cd can_driver
catkin build can_driver
```

**Commit Message**:
```
feat(protocol): add CSP mode enum and interface

Add CSP (Cyclic Synchronous Position) mode support to protocol layer:
- Add ControlMode::CSP enum value (5)
- Add quickSetPosition() virtual interface for fast position writes

This is the first step to implement CSP mode for multi-axis synchronization.

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

---

## Commit 2: feat(eyou): implement quick set position command

**目标**: 在 EyouCan 协议实现中增加快写位置命令

**修改文件**:
- `include/can_driver/EyouCan.h`
- `src/EyouCan.cpp`

**具体改动**:

1. 在 `include/can_driver/EyouCan.h` 中声明：
```cpp
class EyouCan : public CanProtocol {
public:
    // ... 现有接口 ...
    bool quickSetPosition(MotorID motorId, int32_t position) override;
};
```

2. 在 `src/EyouCan.cpp` 中实现：
```cpp
bool EyouCan::quickSetPosition(MotorID Id, int32_t position)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    // CSP 模式：使用快写命令（CMD=0x05），只发送位置帧，无需等待返回
    sendWriteCommand(motorId, 0x0A, static_cast<uint32_t>(position), 4, kFastWriteCommand);
    return true;
}
```

> **单位说明**：`position` 参数为协议原始整数（脉冲数），不在此处做单位换算。
> 角度 → 脉冲的转换由上层 `CanDriverHW::write()` 通过 `positionScale`（配置的
> `position_scale`）统一完成，与 PP 模式的 `setPosition()` 保持一致。

**验证方式**:
```bash
catkin build can_driver
```

**Commit Message**:
```
feat(eyou): implement quick set position command

Implement quickSetPosition() for CSP mode:
- Use CMD=0x05 (quick write) for fast position updates
- Convert angle (degrees) to pulses (65536 pulses = 360°)
- No need to wait for response, enabling high-frequency control

This enables CSP mode to send position commands at 250Hz.

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

---

## Commit 3: feat(eyou): support CSP mode setting

**目标**: 让 EyouCan 支持设置模式 5（CSP 模式）

**修改文件**:
- `src/EyouCan.cpp`

**具体改动**:

修改 `EyouCan::setMode()` 方法，增加 CSP 分支：
```cpp
bool EyouCan::setMode(uint8_t motorId, ControlMode mode) {
    uint8_t modeValue;
    switch (mode) {
        case ControlMode::Position:
            modeValue = 1;
            break;
        case ControlMode::Velocity:
            modeValue = 3;
            break;
        case ControlMode::CSP:
            modeValue = 5;  // 新增
            break;
        default:
            ROS_ERROR("Unknown control mode");
            return false;
    }
    // ... 发送模式设置命令 ...
}
```

**验证方式**:
```bash
catkin build can_driver
```

**Commit Message**:
```
feat(eyou): support CSP mode setting

Add CSP mode (mode 5) support to setMode():
- Map ControlMode::CSP to mode value 5
- Motors can now be switched to CSP mode

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

---

## Commit 4: feat(hw): support CSP mode in write loop

**目标**: 在 CanDriverHW 的控制循环中支持 CSP 模式

**修改文件**:
- `include/can_driver/CanDriverHW.h`
- `src/CanDriverHW.cpp`

**具体改动**:

在 `src/CanDriverHW.cpp` 的 `write()` 方法中增加 CSP 分支：
```cpp
void CanDriverHW::write(const ros::Time& time, const ros::Duration& period) {
    std::lock_guard<std::mutex> lock(deviceMutex_);

    for (const auto& group : jointGroups_) {
        auto proto = group.protocol;

        if (group.controlMode == ControlMode::CSP) {
            // CSP 模式：快速遍历所有电机
            for (size_t i = 0; i < group.jointIndices.size(); ++i) {
                size_t jointIdx = group.jointIndices[i];
                uint8_t motorId = jointMotorIds_[jointIdx];
                double targetPos = jointPositionCommand_[jointIdx];
                proto->quickSetPosition(motorId, targetPos);
            }
        } else {
            // PP/PV 模式：保持原有逻辑
            // ...
        }
    }
}
```

**验证方式**:
```bash
catkin build can_driver
```

**Commit Message**:
```
feat(hw): support CSP mode in write loop

Add CSP mode support to CanDriverHW::write():
- Add CSP branch in write loop using quickSetPosition()
- Preserve existing PP/PV mode logic

CSP mode now integrated into main control loop.

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

---

## Commit 5: feat(config): support CSP mode in configuration

**目标**: 配置解析器支持从 YAML 读取 CSP 模式

**修改文件**:
- `src/JointConfigParser.cpp`
- `config/can_driver.yaml`

**具体改动**:

1. 在 `src/JointConfigParser.cpp` 中增加模式解析：
```cpp
ControlMode parseControlMode(const std::string& modeStr) {
    if (modeStr == "position" || modeStr == "pp") {
        return ControlMode::Position;
    } else if (modeStr == "velocity" || modeStr == "pv") {
        return ControlMode::Velocity;
    } else if (modeStr == "csp") {
        return ControlMode::CSP;
    }
    return ControlMode::Position;
}
```

2. 在 `config/can_driver.yaml` 中增加示例：
```yaml
joint_groups:
  - name: "arm"
    protocol: "eyou"
    control_mode: "csp"  # 支持 "position", "velocity", "csp"
    joints: [1, 2, 3, 4, 5, 6]
```

**验证方式**:
```bash
catkin build can_driver
roslaunch can_driver can_driver.launch
```

**Commit Message**:
```
feat(config): support CSP mode in configuration

Add CSP mode support to configuration parser:
- Parse "csp" control_mode from YAML config
- Update example config with CSP mode

Users can now enable CSP mode via configuration file.

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

---

## Commit 6: test: add unit tests for CSP mode

**目标**: 增加 CSP 模式的单元测试

**修改文件**:
- `tests/test_eyou_can_csp.cpp` (新建)
- `CMakeLists.txt`

**具体改动**:

创建 `tests/test_eyou_can_csp.cpp`：
```cpp
#include <gtest/gtest.h>
#include "can_driver/EyouCan.h"

TEST(EyouCanCSPTest, QuickSetPositionGeneratesCorrectFrame) {
    // 测试快写命令的 CAN 帧格式
    // position 为协议原始脉冲数（int32_t），不是角度
    // 验证 CMD=0x05、寄存器=0x0A、大端序值的正确性
}

TEST(EyouCanCSPTest, CSPModeCanBeSet) {
    // 验证 setMode(id, MotorMode::CSP) 发出的帧：
    // CMD=0x01、寄存器=0x0F、值=5
}
```

在 `CMakeLists.txt` 中添加测试：
```cmake
catkin_add_gtest(test_eyou_can_csp tests/test_eyou_can_csp.cpp)
target_link_libraries(test_eyou_can_csp ${PROJECT_NAME})
```

**验证方式**:
```bash
catkin build can_driver --catkin-make-args run_tests
```

**Commit Message**:
```
test: add unit tests for CSP mode

Add unit tests for CSP mode implementation:
- Test quickSetPosition() CAN frame format
- Test CSP mode setting
- Verify position conversion (degrees to pulses)

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

---

## Commit 7: docs: update documentation for CSP mode

**目标**: 更新文档，说明 CSP 模式的使用方法

**修改文件**:
- `docs/使用指南.md`
- `docs/架构设计.md`
- `README.md`

**具体改动**:

在 `docs/使用指南.md` 中增加 CSP 模式章节：
```markdown
## CSP 模式使用指南

### 什么是 CSP 模式

CSP（Cyclic Synchronous Position，周期同步位置模式）是一种高频位置控制模式：
- 上层控制器以固定周期（如 4ms）持续发送位置目标
- 使用快写命令（CMD=0x05），无需等待返回
- 多轴起步时间差 < 10ms（相比 PP 模式的 100~300ms）

### 配置方法

在 `can_driver.yaml` 中设置：
```yaml
joint_groups:
  - name: "arm"
    protocol: "eyou"
    control_mode: "csp"
    joints: [1, 2, 3, 4, 5, 6]
```

### 控制周期要求

CSP 模式建议使用 250Hz（4ms）控制周期：
```xml
<param name="control_frequency" value="250" />
```

### 适用场景

- 需要多轴精确同步的应用
- 需要上层完全控制轨迹的场景
- 对起步时间一致性要求高的任务
```

**验证方式**:
```bash
# 检查文档格式
cat docs/使用指南.md
```

**Commit Message**:
```
docs: update documentation for CSP mode

Update documentation with CSP mode usage:
- Add CSP mode section to user guide
- Explain configuration and control frequency requirements
- Document use cases and benefits

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
```

---

## 实施检查清单

完成所有 commits 后，进行以下验证：

### 编译测试
```bash
cd ~/Robot24_catkin_ws
catkin clean -y
catkin build can_driver
```

### 单元测试
```bash
catkin build can_driver --catkin-make-args run_tests
catkin_test_results build/can_driver
```

### 集成测试
```bash
# 1. 单轴 CSP 测试
roslaunch can_driver can_driver.launch

# 2. 多轴同步测试（测量起步时间差）
# 使用示波器或 CAN 分析仪测量

# 3. 性能测试（总线占用率）
candump can0 | wc -l
```

### 预期结果

| 测试项 | 预期结果 |
|--------|---------|
| 编译 | 无错误、无警告 |
| 单元测试 | 全部通过 |
| 单轴 CSP | 位置跟踪正常 |
| 多轴同步 | 起步时间差 < 10ms |
| 总线占用 | 4 轴 @ 250Hz ≈ 1000 帧/秒 |

---

**路线图创建日期**: 2026-03-29
**预计完成时间**: 2-3 天
**总 Commits 数**: 7

