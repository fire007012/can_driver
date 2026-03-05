#include "can_driver/EyouCan.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>

#include <ros/ros.h>

// EyouCan.cpp

namespace {
constexpr uint8_t kWriteCommand = 0x01;
constexpr uint8_t kReadCommand = 0x03;
constexpr uint8_t kWriteAck = 0x02;
constexpr uint8_t kReadResponse = 0x04;
constexpr uint16_t kEyouIdFrameBase = 0x0000;
constexpr std::size_t kQueriesPerMotorPerCycle = 5;

std::chrono::milliseconds computeRefreshSleep(std::size_t motorCount)
{
    // 控制轮询发送频率，避免多电机场景下总线过载。
    const std::size_t intervalMs = std::max<std::size_t>(5, motorCount * kQueriesPerMotorPerCycle);
    return std::chrono::milliseconds(intervalMs);
}

uint32_t readUInt32BE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 3 >= frame.dlc) {
        return 0;
    }
    return (static_cast<uint32_t>(frame.data[index]) << 24) |
           (static_cast<uint32_t>(frame.data[index + 1]) << 16) |
           (static_cast<uint32_t>(frame.data[index + 2]) << 8) |
           static_cast<uint32_t>(frame.data[index + 3]);
}

int32_t readInt32BE(const CanTransport::Frame &frame, std::size_t index)
{
    return static_cast<int32_t>(readUInt32BE(frame, index));
}

uint8_t dataByteOrZero(const CanTransport::Frame &frame, std::size_t index)
{
    if (index >= frame.dlc) {
        return 0;
    }
    return frame.data[index];
}

int16_t clampInt32ToInt16WithWarn(int32_t value, uint8_t motorId, const char *field)
{
    constexpr int32_t kMin = static_cast<int32_t>(std::numeric_limits<int16_t>::min());
    constexpr int32_t kMax = static_cast<int32_t>(std::numeric_limits<int16_t>::max());
    if (value < kMin || value > kMax) {
        ROS_WARN_THROTTLE(
            1.0,
            "[EyouCan] Motor %u %s=%d exceeds int16 range, clamping to [%d, %d].",
            static_cast<unsigned>(motorId),
            field,
            value,
            static_cast<int>(kMin),
            static_cast<int>(kMax));
    }
    const int32_t clamped = std::max(kMin, std::min(kMax, value));
    return static_cast<int16_t>(clamped);
}
} // namespace

EyouCan::EyouCan(std::shared_ptr<CanTransport> controller)
    : canController(std::move(controller))
{
    if (canController) {
        receiveHandlerId = canController->addReceiveHandler(
            [this](const CanTransport::Frame &frame) { handleResponse(frame); });
    }
}

EyouCan::~EyouCan()
{
    stopRefreshLoop();
    if (canController && receiveHandlerId != 0) {
        canController->removeReceiveHandler(receiveHandlerId);
    }
}

void EyouCan::initializeMotorRefresh(const std::vector<MotorID> &motorIds)
{
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        refreshMotorIds.clear();
        managedMotorIds.clear();
        refreshMotorIds.reserve(motorIds.size());
        for (MotorID id : motorIds) {
            const auto motorId = static_cast<uint8_t>(id);
            refreshMotorIds.push_back(motorId);
            managedMotorIds.insert(motorId);
        }
    }

    if (motorIds.empty()) {
        stopRefreshLoop();
        return;
    }

    if (refreshLoopActive.load()) {
        return;
    }

    bool expected = false;
    if (!refreshLoopActive.compare_exchange_strong(expected, true)) {
        return;
    }

    refreshThread = std::thread([this]() {
        while (refreshLoopActive.load()) {
            refreshMotorStates();
            std::size_t motorCount = 0;
            {
                std::lock_guard<std::mutex> lock(refreshMutex);
                motorCount = refreshMotorIds.size();
            }
            std::this_thread::sleep_for(computeRefreshSleep(motorCount));
        }
    });
}

bool EyouCan::setMode(MotorID Id, MotorMode mode)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    const uint32_t modeValue = mode == MotorMode::Velocity ? 0x00000003 : 0x00000001;
    sendWriteCommand(motorId, 0x0F, modeValue, 4);
    return true;
}

bool EyouCan::setVelocity(MotorID Id, int32_t velocity)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    sendWriteCommand(motorId, 0x09, static_cast<uint32_t>(velocity), 4);
    return true;
}

// [FIX #5] 协议定义了 0x0B 为目标加速度，补充 CAN 发送
bool EyouCan::setAcceleration(MotorID Id, int32_t acceleration)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    sendWriteCommand(motorId, 0x0B, static_cast<uint32_t>(acceleration), 4);
    return true;
}

// [FIX #6] 协议定义了 0x0C 为目标减速度，补充 CAN 发送
bool EyouCan::setDeceleration(MotorID Id, int32_t deceleration)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    sendWriteCommand(motorId, 0x0C, static_cast<uint32_t>(deceleration), 4);
    return true;
}

bool EyouCan::setPosition(MotorID Id, int32_t position)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    sendWriteCommand(motorId, 0x0A, static_cast<uint32_t>(position), 4);
    return true;
}

bool EyouCan::Enable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    sendWriteCommand(motorId, 0x10, 0x00000001, 4);
    return true;
}

bool EyouCan::Disable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    sendWriteCommand(motorId, 0x10, 0x00000000, 4);
    return true;
}

// [FIX #1] 协议规定 0x11 写 01 结束当前运行，原代码写了 0x00
bool EyouCan::Stop(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    sendWriteCommand(motorId, 0x11, 0x00000001, 4);
    return true;
}

// [FIX #9] 移除 position==0 的不可靠判断，改用 hasReceived 标志
int64_t EyouCan::getPosition(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end() && it->second.positionReceived) {
            return it->second.position;
        }
    }
    // 尚未收到过位置数据，主动请求一次
    requestPosition(motorId);
    return 0;
}

// [FIX #7] 读取协议 0x05（当前电流值），返回缓存值
int16_t EyouCan::getCurrent(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end() && it->second.currentReceived) {
            return clampInt32ToInt16WithWarn(it->second.current, motorId, "current");
        }
    }
    requestCurrent(motorId);
    return 0;
}

// [FIX #8] 读取协议 0x06（当前速度值），返回缓存的实际速度
int16_t EyouCan::getVelocity(MotorID Id) const
{
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        auto it = motorStates.find(motorId);
        if (it != motorStates.end() && it->second.velocityReceived) {
            return clampInt32ToInt16WithWarn(it->second.actualVelocity, motorId, "velocity");
        }
    }
    requestVelocity(motorId);
    return 0;
}

bool EyouCan::isEnabled(MotorID Id) const
{
    const uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    std::lock_guard<std::mutex> stateLock(stateMutex);
    auto it = motorStates.find(motorId);
    return (it != motorStates.end()) ? it->second.enabled : false;
}

bool EyouCan::hasFault(MotorID Id) const
{
    const uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    std::lock_guard<std::mutex> stateLock(stateMutex);
    auto it = motorStates.find(motorId);
    return (it != motorStates.end()) ? it->second.fault : false;
}

void EyouCan::sendWriteCommand(uint8_t motorId, uint8_t subCommand, uint32_t value, std::size_t payloadBytes)
{
    if (!canController) {
        std::cerr << "[EyouCan] CAN controller not initialized\n";
        return;
    }

    if (payloadBytes < 1) {
        payloadBytes = 1;
    } else if (payloadBytes > 4) {
        payloadBytes = 4;
    }

    CanTransport::Frame frame;
    frame.id = kEyouIdFrameBase + motorId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;

    const std::size_t maxPayload = std::min<std::size_t>(payloadBytes, frame.data.size() - 2);
    // [FIX #10] 协议表头示例为8字节帧，用0填充尾部
    frame.dlc = 8;
    frame.data.fill(0);

    frame.data[0] = kWriteCommand;
    frame.data[1] = subCommand;

    for (std::size_t i = 0; i < maxPayload; ++i) {
        const int shift = static_cast<int>((payloadBytes - 1 - i) * 8);
        frame.data[2 + i] = static_cast<uint8_t>((value >> shift) & 0xFF);
    }

    canController->send(frame);
}

void EyouCan::sendReadCommand(uint8_t motorId, uint8_t subCommand) const
{
    if (!canController) {
        return;
    }

    CanTransport::Frame frame;
    frame.id = kEyouIdFrameBase + motorId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    // [FIX #10] 统一 DLC 为 8
    frame.dlc = 8;
    frame.data.fill(0);
    frame.data[0] = kReadCommand;
    frame.data[1] = subCommand;
    canController->send(frame);
}

// [FIX #2, #3, #4] 重写 handleResponse，修正写返回解析和读返回偏移
void EyouCan::handleResponse(const CanTransport::Frame &frame)
{
    if (frame.isExtended || frame.dlc < 2) {
        return;
    }

    const uint16_t canId = static_cast<uint16_t>(frame.id & 0x7FF);
    if (canId >= 0x100) {
        return;
    }
    const uint8_t motorId = static_cast<uint8_t>(canId & 0xFF);
    if (canId != kEyouIdFrameBase + motorId || !isManagedMotorId(motorId)) {
        return;
    }

    const uint8_t responseType = frame.data[0];
    const uint8_t subCommand = frame.data[1];

    bool resyncPosition = false;
    bool resyncVelocity = false;
    bool resyncMode = false;
    bool resyncEnable = false;
    bool resyncCurrent = false;

    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        MotorState &state = motorStates[motorId];

        if (responseType == kWriteAck) {
            // [FIX #2, #3] 写返回格式: 0x02 ADDR STATE
            // STATE 是写入结果状态码，不是寄存器值
            // 0x01=成功, 0x05=矫正后成功, 其它=失败
            uint8_t writeState = dataByteOrZero(frame, 2);
            if (writeState != 0x01 && writeState != 0x05) {
                std::cerr << "[EyouCan] Write to motor " << static_cast<int>(motorId)
                          << " addr 0x" << std::hex << static_cast<int>(subCommand)
                          << " failed, state=0x" << static_cast<int>(writeState)
                          << std::dec << '\n';
                // 写入失败时触发主动重读，避免本地状态与设备状态漂移。
                switch (subCommand) {
                case 0x09:
                    resyncVelocity = true;
                    break;
                case 0x0A:
                    resyncPosition = true;
                    break;
                case 0x0F:
                    resyncMode = true;
                    break;
                case 0x10:
                    resyncEnable = true;
                    break;
                case 0x15:
                    resyncCurrent = true;
                    break;
                default:
                    break;
                }
            }
        } else if (responseType == kReadResponse) {
            // 读返回格式: 0x04 ADDR data0 data1 data2 data3
            // 32位数据从 data[2] 开始（即 DAT2~DAT5）
            switch (subCommand) {
            case 0x05:
                // [FIX #7] 当前电流值，32位有符号数，1=1mA
                state.current = readInt32BE(frame, 2);
                state.currentReceived = true;
                break;
            case 0x06:
                // [FIX #8] 当前速度值，32位有符号数
                state.actualVelocity = readInt32BE(frame, 2);
                state.velocityReceived = true;
                break;
            case 0x07:
                // 当前位置值
                state.position = readInt32BE(frame, 2);
                state.positionReceived = true;  // [FIX #9]
                break;
            case 0x0F:
                // 当前工作模式
                // 读返回: data[2..5] 为32位数据，模式值在最低字节
                state.mode = dataByteOrZero(frame, 5) == 0x03
                                 ? MotorMode::Velocity
                                 : MotorMode::Position;
                break;
            case 0x10:
                // 使能/失能状态
                // 读返回: 32位数据，01=使能 00=失能
                state.enabled = dataByteOrZero(frame, 5) != 0;
                break;
            case 0x15:
                // [FIX #4] 告警指示，数据从 data[2] 开始
                {
                    uint32_t errorCode = readUInt32BE(frame, 2);
                    state.fault = (errorCode != 0);
                    if (errorCode != 0) {
                        std::cerr << "[EyouCan] Motor " << static_cast<int>(motorId)
                                  << " reported error code 0x" << std::hex
                                  << errorCode << std::dec << '\n';
                    }
                }
                break;
            default:
                break;
            }
        }
    }

    if (resyncPosition) {
        requestPosition(motorId);
    }
    if (resyncVelocity) {
        requestVelocity(motorId);
    }
    if (resyncMode) {
        requestMode(motorId);
    }
    if (resyncEnable) {
        requestEnable(motorId);
    }
    if (resyncCurrent) {
        requestCurrent(motorId);
    }
}

bool EyouCan::isManagedMotorId(uint8_t motorId) const
{
    std::lock_guard<std::mutex> lock(refreshMutex);
    if (managedMotorIds.empty()) {
        return true;
    }
    return managedMotorIds.find(motorId) != managedMotorIds.end();
}

void EyouCan::registerManagedMotorId(uint8_t motorId) const
{
    std::lock_guard<std::mutex> lock(refreshMutex);
    managedMotorIds.insert(motorId);
}

void EyouCan::requestPosition(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x07);
}

void EyouCan::requestMode(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x0F);
}

void EyouCan::requestEnable(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x10);
}

// [FIX #7] 新增：请求电流值
void EyouCan::requestCurrent(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x05);
}

// [FIX #8] 新增：请求速度值
void EyouCan::requestVelocity(uint8_t motorId) const
{
    sendReadCommand(motorId, 0x06);
}

void EyouCan::refreshMotorStates()
{
    std::vector<uint8_t> motorIds;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorIds = refreshMotorIds;
    }

    if (!canController) {
        return;
    }

    for (uint8_t motorId : motorIds) {
        if (!refreshLoopActive.load()) {
            break;
        }
        requestPosition(motorId);
        requestMode(motorId);
        requestEnable(motorId);
        requestCurrent(motorId);   // [FIX #7] 轮询电流
        requestVelocity(motorId);  // [FIX #8] 轮询实际速度
    }
}

void EyouCan::stopRefreshLoop()
{
    refreshLoopActive.store(false);

    if (refreshThread.joinable()) {
        refreshThread.join();
    }
}
