#include "can_driver/EyouCan.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>

#include <ros/ros.h>

// EyouCan.cpp

namespace {
constexpr uint8_t kWriteCommand = 0x01;
constexpr uint8_t kFastWriteCommand = 0x05;
constexpr uint8_t kReadCommand = 0x03;
constexpr uint8_t kWriteAck = 0x02;
constexpr uint8_t kReadResponse = 0x04;
constexpr uint16_t kEyouIdFrameBase = 0x0000;
constexpr std::size_t kQueriesPerMotorPerCycle = 5;

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

std::chrono::milliseconds EyouCan::computeRefreshSleep(std::size_t motorCount) const
{
    const double hz = refreshRateHz_.load(std::memory_order_relaxed);
    if (std::isfinite(hz) && hz > 0.0) {
        const auto intervalMs = static_cast<int64_t>(std::llround(1000.0 / hz));
        return std::chrono::milliseconds(std::max<int64_t>(1, intervalMs));
    }
    // 控制轮询发送频率，避免多电机场景下总线过载。
    const std::size_t intervalMs = std::max<std::size_t>(5, motorCount * kQueriesPerMotorPerCycle);
    return std::chrono::milliseconds(intervalMs);
}

EyouCan::EyouCan(std::shared_ptr<CanTransport> controller)
    : EyouCan(std::move(controller), nullptr, nullptr, "")
{
}

EyouCan::EyouCan(std::shared_ptr<CanTransport> controller,
                 std::shared_ptr<CanTxDispatcher> txDispatcher)
    : EyouCan(std::move(controller), std::move(txDispatcher), nullptr, "")
{
}

EyouCan::EyouCan(std::shared_ptr<CanTransport> controller,
                 std::shared_ptr<CanTxDispatcher> txDispatcher,
                 std::shared_ptr<can_driver::SharedDriverState> sharedState,
                 std::string deviceName)
    : canController(std::move(controller))
    , txDispatcher_(std::move(txDispatcher))
    , sharedState_(std::move(sharedState))
    , deviceName_(std::move(deviceName))
{
    if (!txDispatcher_ && canController) {
        txDispatcher_ = std::make_shared<DirectCanTxDispatcher>(canController);
    }
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
            if (sharedState_ && !deviceName_.empty()) {
                sharedState_->registerAxis(deviceName_, CanType::PP, static_cast<MotorID>(motorId));
            }
        }
    }
    resetReadTracking();
    refreshCycleCount_ = 0;

    if (motorIds.empty()) {
        stopRefreshLoop();
    }
}

void EyouCan::setRefreshRateHz(double hz)
{
    if (!std::isfinite(hz) || hz <= 0.0) {
        refreshRateHz_.store(0.0, std::memory_order_relaxed);
        return;
    }
    refreshRateHz_.store(hz, std::memory_order_relaxed);
}

void EyouCan::runRefreshCycle()
{
    refreshMotorStates();
}

std::chrono::milliseconds EyouCan::refreshSleepInterval() const
{
    std::size_t motorCount = 0;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorCount = refreshMotorIds.size();
    }
    return computeRefreshSleep(std::max<std::size_t>(1, motorCount));
}

void EyouCan::setFastWriteEnabled(bool enabled)
{
    fastWriteEnabled_.store(enabled, std::memory_order_relaxed);
    publishWriteCountersParam();
}

uint64_t EyouCan::fastWriteSentCount() const
{
    return fastWriteSentCount_.load(std::memory_order_relaxed);
}

uint64_t EyouCan::normalWriteSentCount() const
{
    return normalWriteSentCount_.load(std::memory_order_relaxed);
}

bool EyouCan::setMode(MotorID Id, MotorMode mode)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].mode = mode;
    }
    syncSharedCommand(motorId, 0, 0, mode, true);

    uint32_t modeValue;
    switch (mode) {
        case MotorMode::Position:
            modeValue = 0x00000001;
            break;
        case MotorMode::Velocity:
            modeValue = 0x00000003;
            break;
        case MotorMode::CSP:
            modeValue = 0x00000005;
            break;
        default:
            ROS_ERROR("[EyouCan] Unknown control mode");
            return false;
    }

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
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedVelocity = velocity;
    }
    syncSharedCommand(motorId, 0, velocity, MotorMode::Velocity, true);
    const bool fastWrite = fastWriteEnabled_.load(std::memory_order_relaxed);
    if (fastWrite) {
        // 兼容策略：先发快写，再补发普通写。
        // 某些固件对 0x05 的位置/速度寄存器写入不会触发运行，补发 0x01 保证生效。
        sendWriteCommand(motorId, 0x09, static_cast<uint32_t>(velocity), 4, kFastWriteCommand);
        sendWriteCommand(motorId, 0x09, static_cast<uint32_t>(velocity), 4, kWriteCommand);
    } else {
        sendWriteCommand(motorId, 0x09, static_cast<uint32_t>(velocity), 4, kWriteCommand);
    }
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
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedPosition = position;
    }
    syncSharedCommand(motorId, position, 0, MotorMode::Position, true);
    const bool fastWrite = fastWriteEnabled_.load(std::memory_order_relaxed);
    // 协议示例：先设置速度，再设置位置（位置模式需目标速度）
    // 10 RPM = 10 * 65536 / 60 ≈ 10922.666，取 0x00002AAA
    if (fastWrite) {
        // 兼容策略：先发快写（满足”快写模式”），再补发普通写（确保动作触发）。
        sendWriteCommand(motorId, 0x09, 0x00002AAA, 4, kFastWriteCommand);
        sendWriteCommand(motorId, 0x0A, static_cast<uint32_t>(position), 4, kFastWriteCommand);
        sendWriteCommand(motorId, 0x09, 0x00002AAA, 4, kWriteCommand);
        sendWriteCommand(motorId, 0x0A, static_cast<uint32_t>(position), 4, kWriteCommand);
    } else {
        sendWriteCommand(motorId, 0x09, 0x00002AAA, 4, kWriteCommand);
        sendWriteCommand(motorId, 0x0A, static_cast<uint32_t>(position), 4, kWriteCommand);
    }
    return true;
}

bool EyouCan::quickSetPosition(MotorID Id, int32_t position)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    {
        std::lock_guard<std::mutex> stateLock(stateMutex);
        motorStates[motorId].commandedPosition = position;
    }
    syncSharedCommand(motorId, position, 0, MotorMode::CSP, true);
    // CSP 模式：使用快写命令（CMD=0x05），只发送位置帧，无需等待返回
    sendWriteCommand(motorId, 0x0A, static_cast<uint32_t>(position), 4, kFastWriteCommand);
    return true;
}

bool EyouCan::Enable(MotorID Id)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    syncSharedIntent(motorId, can_driver::AxisIntent::Enable);
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
    syncSharedIntent(motorId, can_driver::AxisIntent::Disable);
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
    syncSharedIntent(motorId, can_driver::AxisIntent::Hold);
    sendWriteCommand(motorId, 0x11, 0x00000001, 4);
    return true;
}

bool EyouCan::ResetFault(MotorID Id)
{
    if (!canController) {
        return false;
    }
    const uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    syncSharedIntent(motorId, can_driver::AxisIntent::Recover);
    sendWriteCommand(motorId, 0x15, 0x00000000, 4);
    markReadResponseReceived(motorId, 0x15);
    requestFault(motorId);
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

bool EyouCan::configurePositionLimits(MotorID Id,
                                      int32_t minPositionRaw,
                                      int32_t maxPositionRaw,
                                      bool enable)
{
    if (!canController) {
        return false;
    }
    if (minPositionRaw > maxPositionRaw) {
        return false;
    }

    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    // PP 协议：0x39 上限、0x3A 下限、0x38 限位使能
    sendWriteCommand(motorId, 0x39, static_cast<uint32_t>(maxPositionRaw), 4, kWriteCommand);
    sendWriteCommand(motorId, 0x3A, static_cast<uint32_t>(minPositionRaw), 4, kWriteCommand);
    sendWriteCommand(motorId, 0x38, enable ? 0x00000001 : 0x00000000, 4, kWriteCommand);
    return true;
}

bool EyouCan::setPositionOffset(MotorID Id, int32_t offsetRaw)
{
    if (!canController) {
        return false;
    }
    uint8_t motorId = static_cast<uint8_t>(Id);
    registerManagedMotorId(motorId);
    // PP 协议：0x3B 位置偏置参数
    sendWriteCommand(motorId, 0x3B, static_cast<uint32_t>(offsetRaw), 4, kWriteCommand);
    return true;
}

void EyouCan::sendWriteCommand(uint8_t motorId,
                               uint8_t subCommand,
                               uint32_t value,
                               std::size_t payloadBytes,
                               uint8_t commandType)
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

    frame.data[0] = commandType;
    frame.data[1] = subCommand;

    for (std::size_t i = 0; i < maxPayload; ++i) {
        const int shift = static_cast<int>((payloadBytes - 1 - i) * 8);
        frame.data[2 + i] = static_cast<uint8_t>((value >> shift) & 0xFF);
    }

    const auto category = (subCommand == 0x15) ? CanTxDispatcher::Category::Recover
                                               : CanTxDispatcher::Category::Control;
    if (!submitTx(frame, category, "EyouCan::sendWriteCommand")) {
        return;
    }

    if (commandType == kFastWriteCommand) {
        fastWriteSentCount_.fetch_add(1, std::memory_order_relaxed);
    } else if (commandType == kWriteCommand) {
        normalWriteSentCount_.fetch_add(1, std::memory_order_relaxed);
    }

    // 低频发布运行时计数，避免每帧写参数。
    const uint64_t total = fastWriteSentCount_.load(std::memory_order_relaxed) +
                           normalWriteSentCount_.load(std::memory_order_relaxed);
    if (total % 20 == 0) {
        publishWriteCountersParam();
    }
}

void EyouCan::publishWriteCountersParam() const
{
    if (!ros::isInitialized()) {
        return;
    }
    if (!ros::master::check()) {
        ROS_WARN_STREAM_THROTTLE(5.0, "[EyouCan] ROS master unavailable, skip publishing PP write counters.");
        return;
    }
    static ros::NodeHandle pnh("~");
    pnh.setParam("pp_fast_write_enabled_runtime", fastWriteEnabled_.load(std::memory_order_relaxed));
    pnh.setParam("pp_fast_write_sent_count", static_cast<double>(fastWriteSentCount_.load(std::memory_order_relaxed)));
    pnh.setParam("pp_normal_write_sent_count", static_cast<double>(normalWriteSentCount_.load(std::memory_order_relaxed)));
}

void EyouCan::sendReadCommand(uint8_t motorId, uint8_t subCommand)
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
    (void)submitTx(frame, CanTxDispatcher::Category::Query, "EyouCan::sendReadCommand");
}

bool EyouCan::submitTx(const CanTransport::Frame &frame,
                       CanTxDispatcher::Category category,
                       const char *source) const
{
    if (!txDispatcher_) {
        ROS_ERROR_STREAM_THROTTLE(1.0,
                                  "[EyouCan] TX dispatcher unavailable for "
                                  << (source ? source : "unknown"));
        return false;
    }

    CanTxDispatcher::Request request;
    request.frame = frame;
    request.category = category;
    request.source = source;
    txDispatcher_->submit(request);
    return true;
}

bool EyouCan::tryIssueReadCommand(uint8_t motorId, uint8_t subCommand)
{
    if (!canController) {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto timeout = computeReadRequestTimeout();
    bool delayRetry = false;
    std::size_t consecutiveTimeouts = 0;
    std::chrono::milliseconds retryBackoff(0);
    {
        std::lock_guard<std::mutex> lock(pendingReadMutex_);
        auto &request = pendingReadRequests_[pendingReadKey(motorId, subCommand)];
        if (request.nextEligibleSend != std::chrono::steady_clock::time_point {} &&
            now < request.nextEligibleSend) {
            return false;
        }
        if (request.inFlight && (now - request.lastSent) < timeout) {
            return false;
        }
        if (request.inFlight) {
            request.inFlight = false;
            request.consecutiveTimeouts = std::min<std::size_t>(request.consecutiveTimeouts + 1, 8);
            request.nextEligibleSend = now + computeTimeoutBackoff(request.consecutiveTimeouts, timeout);
            consecutiveTimeouts = request.consecutiveTimeouts;
            retryBackoff = std::chrono::duration_cast<std::chrono::milliseconds>(
                request.nextEligibleSend - now);
            delayRetry = true;
        } else {
            request.inFlight = true;
            request.lastSent = now;
            request.nextEligibleSend = std::chrono::steady_clock::time_point {};
        }
    }

    if (delayRetry) {
        noteSharedTimeout(motorId, consecutiveTimeouts);
        ROS_WARN_STREAM_THROTTLE(
            1.0,
            "[EyouCan] Read timeout on motor " << static_cast<unsigned>(motorId)
            << " subcmd=0x" << std::hex << static_cast<unsigned>(subCommand) << std::dec
            << ", backing off for " << retryBackoff.count()
            << " ms before retry"
            << " (consecutive_timeouts=" << consecutiveTimeouts << ")");
        return false;
    }

    sendReadCommand(motorId, subCommand);
    return true;
}

void EyouCan::markReadResponseReceived(uint8_t motorId, uint8_t subCommand)
{
    std::lock_guard<std::mutex> lock(pendingReadMutex_);
    auto it = pendingReadRequests_.find(pendingReadKey(motorId, subCommand));
    if (it != pendingReadRequests_.end()) {
        it->second.inFlight = false;
        it->second.nextEligibleSend = std::chrono::steady_clock::time_point {};
        it->second.consecutiveTimeouts = 0;
    }
}

can_driver::SharedDriverState::AxisKey EyouCan::makeAxisKey(uint8_t motorId) const
{
    return can_driver::MakeAxisKey(deviceName_, CanType::PP, static_cast<MotorID>(motorId));
}

void EyouCan::syncSharedFeedback(uint8_t motorId, const MotorState &state) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    sharedState_->mutateAxisFeedback(
        makeAxisKey(motorId),
        [&](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->position = state.position;
            feedback->velocity = state.actualVelocity;
            feedback->current = state.current;
            feedback->mode = state.mode;
            feedback->positionValid = state.positionReceived;
            feedback->velocityValid = state.velocityReceived;
            feedback->currentValid = state.currentReceived;
            feedback->enabled = state.enabled;
            feedback->fault = state.fault;
            feedback->feedbackSeen = true;
            feedback->lastRxSteadyNs = nowNs;
            feedback->lastValidStateSteadyNs = nowNs;
            feedback->consecutiveTimeoutCount = 0;
        });
}

void EyouCan::syncSharedCommand(uint8_t motorId,
                                int64_t targetPosition,
                                int32_t targetVelocity,
                                MotorMode desiredMode,
                                bool valid) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    sharedState_->mutateAxisCommand(
        makeAxisKey(motorId),
        [&](can_driver::SharedDriverState::AxisCommandState *command) {
            command->targetPosition = targetPosition;
            command->targetVelocity = targetVelocity;
            command->desiredMode = desiredMode;
            command->valid = valid;
            command->lastCommandSteadyNs = nowNs;
        });
}

void EyouCan::syncSharedIntent(uint8_t motorId, can_driver::AxisIntent intent) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }
    sharedState_->setAxisIntent(makeAxisKey(motorId), intent);
}

void EyouCan::noteSharedTimeout(uint8_t motorId, std::size_t consecutiveTimeouts) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    sharedState_->mutateAxisFeedback(
        makeAxisKey(motorId),
        [consecutiveTimeouts](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->consecutiveTimeoutCount =
                static_cast<std::uint32_t>(consecutiveTimeouts);
        });
}

void EyouCan::resetReadTracking()
{
    std::lock_guard<std::mutex> lock(pendingReadMutex_);
    pendingReadRequests_.clear();
}

uint16_t EyouCan::pendingReadKey(uint8_t motorId, uint8_t subCommand)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(motorId) << 8) | subCommand);
}

std::chrono::milliseconds EyouCan::computeReadRequestTimeout() const
{
    std::size_t motorCount = 1;
    {
        std::lock_guard<std::mutex> lock(refreshMutex);
        motorCount = std::max<std::size_t>(1, refreshMotorIds.size());
    }
    const auto refreshSleep = computeRefreshSleep(motorCount);
    const auto timeout = refreshSleep * 3;
    return std::max(std::chrono::milliseconds(20), std::min(timeout, std::chrono::milliseconds(200)));
}

std::chrono::milliseconds EyouCan::computeTimeoutBackoff(std::size_t consecutiveTimeouts,
                                                         std::chrono::milliseconds baseTimeout)
{
    const std::size_t cappedTimeouts = std::min<std::size_t>(consecutiveTimeouts, 4);
    const auto multiplier = static_cast<int64_t>(1ULL << cappedTimeouts);
    const auto backoff = baseTimeout * multiplier;
    return std::min(std::chrono::milliseconds(500), std::max(baseTimeout, backoff));
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
    bool sharedFeedbackUpdated = false;
    MotorState sharedFeedbackSnapshot;

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
                    requestFault(motorId);
                    break;
                default:
                    break;
                }
            }
        } else if (responseType == kReadResponse) {
            markReadResponseReceived(motorId, subCommand);
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
            sharedFeedbackSnapshot = state;
            sharedFeedbackUpdated = true;
        }
    }

    if (sharedFeedbackUpdated) {
        syncSharedFeedback(motorId, sharedFeedbackSnapshot);
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
    if (sharedState_ && !deviceName_.empty()) {
        sharedState_->registerAxis(deviceName_, CanType::PP, static_cast<MotorID>(motorId));
    }
}

void EyouCan::requestPosition(uint8_t motorId)
{
    (void)tryIssueReadCommand(motorId, 0x07);
}

void EyouCan::requestMode(uint8_t motorId)
{
    (void)tryIssueReadCommand(motorId, 0x0F);
}

void EyouCan::requestEnable(uint8_t motorId)
{
    (void)tryIssueReadCommand(motorId, 0x10);
}

void EyouCan::requestFault(uint8_t motorId)
{
    (void)tryIssueReadCommand(motorId, 0x15);
}

// [FIX #7] 新增：请求电流值
void EyouCan::requestCurrent(uint8_t motorId)
{
    (void)tryIssueReadCommand(motorId, 0x05);
}

// [FIX #8] 新增：请求速度值
void EyouCan::requestVelocity(uint8_t motorId)
{
    (void)tryIssueReadCommand(motorId, 0x06);
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

    const bool slowCycle = (refreshCycleCount_ % kSlowDivider == 0);
    ++refreshCycleCount_;

    for (uint8_t motorId : motorIds) {
        // 位置 + 速度：每周期都查（控制闭环需要）
        requestPosition(motorId);
        requestVelocity(motorId);

        // mode / enable / fault / current：每 kSlowDivider 个周期查一次
        if (slowCycle) {
            requestMode(motorId);
            requestEnable(motorId);
            requestFault(motorId);
            requestCurrent(motorId);
        }
    }
}

void EyouCan::stopRefreshLoop()
{
    refreshCycleCount_ = 0;
    resetReadTracking();
}
