#include "can_driver/DamiaoCan.h"
#include "can_driver/DeviceRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <future>
#include <limits>
#include <thread>
#include <utility>

#include "can_driver/ros2_compat.hpp"

namespace {

using can_driver::motorIdFromProtocolNodeId;
using can_driver::toProtocolNodeId;

constexpr std::uint32_t kDamiaoMasterId = 0x000u;
constexpr std::uint32_t kDamiaoRegisterFrameId = 0x7FFu;
constexpr std::uint32_t kDamiaoSpeedFrameBase = 0x200u;
constexpr std::uint8_t kDamiaoWriteRegisterCommand = 0x55u;
constexpr std::uint8_t kDamiaoControlModeRegister = 10u;
constexpr std::uint8_t kDamiaoPMaxRegister = 21u;
constexpr std::uint8_t kDamiaoEnableCommand = 0xFCu;
constexpr std::uint8_t kDamiaoDisableCommand = 0xFDu;
constexpr std::uint8_t kDamiaoClearFaultCommand = 0xFBu;
constexpr std::uint8_t kDamiaoEnabledStateCode = 0x1u;
constexpr std::uint8_t kDamiaoDisabledStateCode = 0x0u;
constexpr std::uint32_t kDamiaoVelocityModeValue = 3u;
// Damiao 后端在线上发送/接收的是 SI 物理量：
// - 速度命令帧 data[0..3] 为 float32(rad/s)
// - 反馈帧中的位置/速度/力矩字段按协议范围映射为 rad / rad/s / Nm
//
// 但 can_driver 内部仍统一缓存为“定点 SI”整数，便于与其余协议共用
// SharedDriverState / CanProtocol 抽象：
// - 位置: 1 raw = 1e-4 rad
// - 速度: 1 raw = 1e-4 rad/s
//
// 这个缩放只存在于驱动内部，不等同于编码器脉冲，也不包含减速比语义。
constexpr double kDamiaoProtocolScale = 1e-4;
const auto kDamiaoRegisterAckTimeout = std::chrono::milliseconds(500);
const auto kDamiaoFeedbackStateTimeout = std::chrono::milliseconds(100);
const auto kDamiaoConfigStepDelay = std::chrono::milliseconds(50);

std::uint32_t readUInt32LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 3 >= frame.dlc) {
        return 0;
    }
    return static_cast<std::uint32_t>(frame.data[index]) |
           (static_cast<std::uint32_t>(frame.data[index + 1]) << 8) |
           (static_cast<std::uint32_t>(frame.data[index + 2]) << 16) |
           (static_cast<std::uint32_t>(frame.data[index + 3]) << 24);
}

} // namespace

DamiaoCan::DamiaoCan(std::shared_ptr<CanTransport> controller,
                     std::shared_ptr<CanTxDispatcher> txDispatcher)
    : DamiaoCan(std::move(controller), std::move(txDispatcher), nullptr, "")
{
}

DamiaoCan::DamiaoCan(std::shared_ptr<CanTransport> controller,
                     std::shared_ptr<CanTxDispatcher> txDispatcher,
                     std::shared_ptr<can_driver::SharedDriverState> sharedState,
                     std::string deviceName)
    : canController_(std::move(controller))
    , txDispatcher_(std::move(txDispatcher))
    , sharedState_(std::move(sharedState))
    , deviceName_(std::move(deviceName))
{
    if (canController_) {
        receiveHandlerId_ = canController_->addReceiveHandler(
            [this](const CanTransport::Frame &frame) { handleResponse(frame); });
    }
}

DamiaoCan::~DamiaoCan()
{
    shuttingDown_.store(true, std::memory_order_release);
    if (canController_ && receiveHandlerId_ != 0) {
        canController_->removeReceiveHandler(receiveHandlerId_);
        receiveHandlerId_ = 0;
    }
    if (const auto runtime = std::dynamic_pointer_cast<DeviceRuntime>(txDispatcher_)) {
        runtime->shutdown();
    }
    txDispatcher_.reset();
}

bool DamiaoCan::setMitGains(MotorID motorId, double kp, double kd)
{
    if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 500.0/4095 || kp > 500 ||
        kd < 5.0/4095 || kd > 5) return false;
    std::lock_guard<std::mutex> lock(stateMutex_);
    auto &state = motorStates_[toProtocolNodeId(motorId)];
    if (state.enabled) return false;
    state.mitKp = kp; state.mitKd = kd;
    return true;
}

bool DamiaoCan::setMode(MotorID motorId, MotorMode mode)
{
    if (mode != MotorMode::Velocity && mode != MotorMode::Position) return false;
    const auto nodeId = toProtocolNodeId(motorId);
    if (nodeId > 15) return false;
    registerManagedMotorId(motorId);
    if (!ensureModeConfigured(nodeId, mode)) return false;
    syncSharedModeSelection(nodeId, mode);
    // A disabled DM may stay silent until a control frame is received. Seed only
    // a neutral command so startup can validate real feedback before enabling.
    syncSharedIntent(nodeId, can_driver::AxisIntent::Hold);
    return sendNeutral(nodeId, CanTxDispatcher::Category::Control);
}

bool DamiaoCan::setVelocity(MotorID motorId, int32_t velocity)
{
    const auto nodeId = toProtocolNodeId(motorId);
    registerManagedMotorId(motorId);
    if (!ensureModeConfigured(nodeId, MotorMode::Velocity)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        auto &state = motorStates_[nodeId];
        // 注意：这里缓存的 velocity 仍是 can_driver 内部定点 SI 整数，
        // 不是达妙总线上直接发送的 float32。
        state.commandedVelocity = velocity;
        state.mode = MotorMode::Velocity;
    }
    syncSharedCommand(nodeId, velocity, true);
    syncSharedIntent(nodeId, can_driver::AxisIntent::Run);
    // 达妙速度帧在线上要求的是 float32 rad/s，
    // 因此这里要先把内部整数刻度还原成 SI 速度后再打包。
    const bool sent = sendSpeedFrame(nodeId,
                          rawToVelocityRadPerSec(velocity),
                          CanTxDispatcher::Category::Control,
                          "DamiaoCan::setVelocity");
    if (!sent) {
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            motorStates_[nodeId].commandedVelocity = 0;
        }
        syncSharedCommand(nodeId, 0, false);
    }
    return sent;
}

bool DamiaoCan::setAcceleration(MotorID motorId, int32_t acceleration)
{
    const auto nodeId = toProtocolNodeId(motorId);
    registerManagedMotorId(motorId);
    std::lock_guard<std::mutex> lock(stateMutex_);
    motorStates_[nodeId].acceleration = acceleration;
    return true;
}

bool DamiaoCan::setDeceleration(MotorID motorId, int32_t deceleration)
{
    const auto nodeId = toProtocolNodeId(motorId);
    registerManagedMotorId(motorId);
    std::lock_guard<std::mutex> lock(stateMutex_);
    motorStates_[nodeId].deceleration = deceleration;
    return true;
}

bool DamiaoCan::setPosition(MotorID motorId, int32_t position)
{
    const auto nodeId = toProtocolNodeId(motorId);
    registerManagedMotorId(motorId);
    if (!ensureModeConfigured(nodeId, MotorMode::Position)) return false;
    const bool sent = sendMitTarget(nodeId, position * kDamiaoProtocolScale, true,
                                    CanTxDispatcher::Category::Control);
    if (!sent) sendNeutral(nodeId, CanTxDispatcher::Category::Control);
    if (sharedState_ && !deviceName_.empty()) {
        sharedState_->mutateAxisCommand(makeAxisKey(nodeId), [&](auto command) {
            command->targetPosition = position; command->targetVelocity = 0;
            command->desiredMode = MotorMode::Position; command->desiredModeValid = true;
            command->valid = sent; command->lastCommandSteadyNs = sent ? can_driver::SharedDriverSteadyNowNs() : 0;
        });
    }
    syncSharedIntent(nodeId, can_driver::AxisIntent::Run);
    return sent;
}

bool DamiaoCan::quickSetPosition(MotorID, int32_t)
{
    ROS_ERROR_THROTTLE(1.0,
                       "[DamiaoCan] CSP quick position is not supported in the first DM backend.");
    return false;
}

bool DamiaoCan::Enable(MotorID motorId)
{
    const auto nodeId = toProtocolNodeId(motorId);
    registerManagedMotorId(motorId);
    MotorMode mode;
    { std::lock_guard<std::mutex> lock(stateMutex_); mode = motorStates_[nodeId].mode; }
    if (!ensureModeConfigured(nodeId, mode)) return false;
    syncSharedModeSelection(nodeId, mode);
    syncSharedIntent(nodeId, can_driver::AxisIntent::Enable);
    const auto requestedAt = can_driver::SharedDriverSteadyNowNs();
    if (!sendMotorCommand(nodeId,
                          kDamiaoEnableCommand,
                          CanTxDispatcher::Category::Control,
                          "DamiaoCan::Enable")) {
        return false;
    }
    std::this_thread::sleep_for(kDamiaoConfigStepDelay);
    if (!sendNeutral(nodeId, CanTxDispatcher::Category::Control)) {
        return false;
    }
    return waitForFeedbackState(nodeId,
                                kDamiaoEnabledStateCode,
                                kDamiaoFeedbackStateTimeout, requestedAt);
}

bool DamiaoCan::Disable(MotorID motorId)
{
    const auto nodeId = toProtocolNodeId(motorId);
    registerManagedMotorId(motorId);
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        motorStates_[nodeId].commandedVelocity = 0;
    }
    syncSharedCommand(nodeId, 0, true);
    syncSharedIntent(nodeId, can_driver::AxisIntent::Disable);
    const auto requestedAt = can_driver::SharedDriverSteadyNowNs();

    if (!sendNeutral(nodeId, CanTxDispatcher::Category::Control)) {
        return false;
    }
    if (!sendMotorCommand(nodeId,
                          kDamiaoDisableCommand,
                          CanTxDispatcher::Category::Control,
                          "DamiaoCan::Disable")) {
        return false;
    }
    return waitForFeedbackState(nodeId,
                                kDamiaoDisabledStateCode,
                                kDamiaoFeedbackStateTimeout, requestedAt);
}

bool DamiaoCan::Stop(MotorID motorId)
{
    const auto nodeId = toProtocolNodeId(motorId);
    registerManagedMotorId(motorId);
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        motorStates_[nodeId].commandedVelocity = 0;
    }
    syncSharedCommand(nodeId, 0, true);
    syncSharedIntent(nodeId, can_driver::AxisIntent::Hold);
    return sendNeutral(nodeId, CanTxDispatcher::Category::Control);
}

bool DamiaoCan::ResetFault(MotorID motorId)
{
    const auto nodeId = toProtocolNodeId(motorId);
    registerManagedMotorId(motorId);
    syncSharedIntent(nodeId, can_driver::AxisIntent::Recover);
    return sendMotorCommand(nodeId,
                            kDamiaoClearFaultCommand,
                            CanTxDispatcher::Category::Recover,
                            "DamiaoCan::ResetFault");
}

int64_t DamiaoCan::getPosition(MotorID motorId) const
{
    const auto nodeId = toProtocolNodeId(motorId);
    std::lock_guard<std::mutex> lock(stateMutex_);
    const auto it = motorStates_.find(nodeId);
    return (it == motorStates_.end()) ? 0 : it->second.position;
}

int16_t DamiaoCan::getCurrent(MotorID motorId) const
{
    const auto nodeId = toProtocolNodeId(motorId);
    std::lock_guard<std::mutex> lock(stateMutex_);
    const auto it = motorStates_.find(nodeId);
    return (it == motorStates_.end()) ? 0 : it->second.current;
}

int32_t DamiaoCan::getVelocity(MotorID motorId) const
{
    const auto nodeId = toProtocolNodeId(motorId);
    std::lock_guard<std::mutex> lock(stateMutex_);
    const auto it = motorStates_.find(nodeId);
    return (it == motorStates_.end()) ? 0 : it->second.velocity;
}

bool DamiaoCan::isEnabled(MotorID motorId) const
{
    const auto nodeId = toProtocolNodeId(motorId);
    std::lock_guard<std::mutex> lock(stateMutex_);
    const auto it = motorStates_.find(nodeId);
    return (it == motorStates_.end()) ? false : it->second.enabled;
}

bool DamiaoCan::hasFault(MotorID motorId) const
{
    const auto nodeId = toProtocolNodeId(motorId);
    std::lock_guard<std::mutex> lock(stateMutex_);
    const auto it = motorStates_.find(nodeId);
    return (it == motorStates_.end()) ? false : it->second.fault;
}

void DamiaoCan::initializeMotorRefresh(const std::vector<MotorID> &motorIds)
{
    std::lock_guard<std::mutex> lock(refreshMutex_);
    refreshMotorIds_.clear();
    managedMotorIds_.clear();
    systemMotorIdsByNodeId_.clear();
    refreshMotorIds_.reserve(motorIds.size());
    for (const auto motorId : motorIds) {
        const auto nodeId = toProtocolNodeId(motorId);
        refreshMotorIds_.push_back(nodeId);
        managedMotorIds_.insert(nodeId);
        systemMotorIdsByNodeId_[nodeId] = motorId;
        if (sharedState_ && !deviceName_.empty()) {
            sharedState_->registerAxis(deviceName_, CanType::DM, motorId);
        }
    }
}

void DamiaoCan::setRefreshRateHz(double hz)
{
    if (!std::isfinite(hz) || hz <= 0.0) {
        refreshRateHz_.store(0.0, std::memory_order_relaxed);
        return;
    }
    refreshRateHz_.store(hz, std::memory_order_relaxed);
}

std::chrono::milliseconds DamiaoCan::refreshSleepInterval() const
{
    std::size_t motorCount = 0;
    {
        std::lock_guard<std::mutex> lock(refreshMutex_);
        motorCount = refreshMotorIds_.size();
    }
    return computeRefreshSleep(std::max<std::size_t>(1, motorCount));
}

void DamiaoCan::setMotorMasterId(MotorID motorId, std::uint32_t masterId)
{
    const auto nodeId = toProtocolNodeId(motorId);
    std::lock_guard<std::mutex> lock(refreshMutex_);
    masterIdsByNodeId_[nodeId] = masterId;
    systemMotorIdsByNodeId_[nodeId] = motorId;
}

bool DamiaoCan::issueRefreshQuery(MotorID motorId, RefreshQuery query)
{
    if (query != RefreshQuery::Keepalive || shuttingDown_.load(std::memory_order_acquire)) {
        return false;
    }

    const auto nodeId = toProtocolNodeId(motorId);
    registerManagedMotorId(motorId);

    int32_t commandedVelocity = 0;
    CanTransport::Frame mitFrame{};
    bool mit = false, valid = false;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        auto &state = motorStates_[nodeId];
        commandedVelocity = state.commandedVelocity;
        mit = state.mode == MotorMode::Position;
        mitFrame = state.mitFrame; valid = state.mitFrameValid;
    }
    if (mit) return valid && submitTx(mitFrame, CanTxDispatcher::Category::Query,
                                      "DamiaoCan::MIT keepalive");

    return sendSpeedFrame(nodeId,
                          rawToVelocityRadPerSec(commandedVelocity),
                          CanTxDispatcher::Category::Query,
                          "DamiaoCan::issueRefreshQuery");
}

void DamiaoCan::handleResponse(const CanTransport::Frame &frame)
{
    if (frame.isExtended || frame.isRemoteRequest) return;
    if (frame.dlc >= 8 && frame.data[1] == 0 &&
        (frame.data[2] == kDamiaoWriteRegisterCommand || frame.data[2] == 0x33)) {
        const auto motorId = static_cast<std::uint8_t>(frame.data[0]);
        if (frame.id != masterIdForMotor(motorId)) {
            return;
        }
        const auto registerId = frame.data[3];
        const auto value = readUInt32LE(frame, 4);
        bool matchedPendingAck = false;
        {
            std::lock_guard<std::mutex> lock(pendingRegisterAckMutex_);
            const auto it = pendingRegisterAcks_.find(pendingRegisterAckKey(motorId, registerId));
            if (it != pendingRegisterAcks_.end() && it->second.command == frame.data[2]) {
                it->second.received = true;
                it->second.value = value;
                matchedPendingAck = true;
            }
        }
        if (matchedPendingAck) {
            pendingRegisterAckCv_.notify_all();
            return;
        }
        return;
    }

    if (frame.dlc < 8) {
        return;
    }

    const auto feedbackState = static_cast<std::uint8_t>((frame.data[0] >> 4) & 0x0Fu);
    const auto motorId = static_cast<std::uint8_t>(frame.data[0] & 0x0Fu);
    if (frame.id != masterIdForMotor(motorId)) {
        return;
    }
    if (!isManagedMotorId(motorId)) {
        return;
    }

    const auto posInt =
        static_cast<std::uint32_t>((frame.data[1] << 8) | frame.data[2]);
    const auto velInt =
        static_cast<std::uint32_t>((frame.data[3] << 4) | (frame.data[4] >> 4));
    const auto torqueInt =
        static_cast<std::uint32_t>(((frame.data[4] & 0x0Fu) << 8) | frame.data[5]);

    float pMax, vMax, tMax;
    bool mapped, modeConfirmed;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        const auto &state = motorStates_[motorId];
        pMax = state.pMax; vMax = state.vMax; tMax = state.tMax;
        mapped = state.feedbackRangeConfigured;
        modeConfirmed = state.modeConfigured;
    }
    const auto positionRad = uintToFloat(posInt, -pMax, pMax, 16);
    const auto velocityRadPerSec =
        uintToFloat(velInt, -vMax, vMax, 12);
    const auto torqueNm = uintToFloat(torqueInt, -tMax, tMax, 12);

    MotorState snapshot;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        auto &state = motorStates_[motorId];
        // 反馈帧先按协议范围还原到 SI，
        // 再转换回 can_driver 内部统一的定点 SI 整数缓存。
        state.position = rawPositionFromRadians(positionRad);
        state.velocity = rawVelocityFromRadiansPerSec(velocityRadPerSec);
        state.current = rawTorqueFromNewtonMeters(torqueNm);
        state.feedbackState = feedbackState;
        state.enabled = (feedbackState == kDamiaoEnabledStateCode);
        state.fault = (feedbackState != kDamiaoDisabledStateCode &&
                       feedbackState != kDamiaoEnabledStateCode);
        state.positionReceived = mapped; state.feedbackSteadyNs[0] = can_driver::SharedDriverSteadyNowNs();
        state.velocityReceived = mapped; state.feedbackSteadyNs[1] = can_driver::SharedDriverSteadyNowNs();
        state.currentReceived = mapped; state.feedbackSteadyNs[5] = can_driver::SharedDriverSteadyNowNs();
        state.modeReceived = modeConfirmed; state.feedbackSteadyNs[2] = can_driver::SharedDriverSteadyNowNs();
        state.enabledReceived = true; state.feedbackSteadyNs[3] = can_driver::SharedDriverSteadyNowNs();
        state.faultReceived = true; state.feedbackSteadyNs[4] = can_driver::SharedDriverSteadyNowNs();
        state.feedbackSeen = true;
        snapshot = state;
    }

    syncSharedFeedback(motorId, snapshot);
    feedbackStateCv_.notify_all();
}

bool DamiaoCan::sendNeutral(std::uint8_t motorId, CanTxDispatcher::Category category)
{
    bool mit;
    { std::lock_guard<std::mutex> lock(stateMutex_);
      auto &state = motorStates_[motorId]; state.commandedVelocity = 0;
      mit = state.mode == MotorMode::Position; }
    // MIT neutral: zero stiffness/damping/feed-forward, never jump to zero position.
    if (mit) return sendMitTarget(motorId, 0, false, category);
    return sendSpeedFrame(motorId, 0, category, "DamiaoCan::neutral");
}

bool DamiaoCan::sendMitTarget(std::uint8_t motorId, double position, bool active,
                             CanTxDispatcher::Category category)
{
    CanTransport::Frame frame{};
    frame.id = motorId; frame.dlc = 8;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        auto &state = motorStates_[motorId];
        if (!state.modeConfigured || !state.feedbackRangeConfigured ||
            state.mode != MotorMode::Position || !std::isfinite(position) ||
            std::abs(position) > state.pMax) return false;
        auto encode = [](double value, double lo, double hi, int bits) {
            return static_cast<std::uint32_t>((value - lo) * ((1u << bits) - 1u) / (hi - lo));
        };
        const auto p = encode(position, -state.pMax, state.pMax, 16);
        const auto v = encode(0, -state.vMax, state.vMax, 12);
        const auto kp = encode(active ? state.mitKp : 0, 0, 500, 12);
        const auto kd = encode(active ? state.mitKd : 0, 0, 5, 12);
        const auto t = encode(0, -state.tMax, state.tMax, 12);
        frame.data = {{static_cast<std::uint8_t>(p >> 8), static_cast<std::uint8_t>(p),
                       static_cast<std::uint8_t>(v >> 4), static_cast<std::uint8_t>((v << 4) | (kp >> 8)),
                       static_cast<std::uint8_t>(kp), static_cast<std::uint8_t>(kd >> 4),
                       static_cast<std::uint8_t>((kd << 4) | (t >> 8)), static_cast<std::uint8_t>(t)}};
        state.mitFrame = frame; state.mitFrameValid = true;
    }
    return submitTx(frame, category, "DamiaoCan::MIT target");
}

bool DamiaoCan::sendSpeedFrame(std::uint8_t motorId,
                               float velocityRadPerSec,
                               CanTxDispatcher::Category category,
                               const char *source) const
{
    CanTransport::Frame frame;
    frame.id = kDamiaoSpeedFrameBase + motorId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 4;
    frame.data.fill(0);
    const auto rawBits = floatToRawBits(velocityRadPerSec);
    frame.data[0] = static_cast<std::uint8_t>(rawBits & 0xFFu);
    frame.data[1] = static_cast<std::uint8_t>((rawBits >> 8) & 0xFFu);
    frame.data[2] = static_cast<std::uint8_t>((rawBits >> 16) & 0xFFu);
    frame.data[3] = static_cast<std::uint8_t>((rawBits >> 24) & 0xFFu);
    return submitTx(frame, category, source);
}

bool DamiaoCan::sendMotorCommand(std::uint8_t motorId,
                                 std::uint8_t command,
                                 CanTxDispatcher::Category category,
                                 const char *source) const
{
    CanTransport::Frame frame;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        const auto it = motorStates_.find(motorId);
        frame.id = (it != motorStates_.end() && it->second.mode == MotorMode::Position)
                     ? motorId : kDamiaoSpeedFrameBase + motorId;
    }
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 8;
    frame.data.fill(0xFFu);
    frame.data[7] = command;
    return submitTx(frame, category, source);
}

bool DamiaoCan::sendRegisterWrite(std::uint8_t motorId,
                                  std::uint8_t registerId,
                                  std::uint32_t value,
                                  const char *source) const
{
    CanTransport::Frame frame;
    frame.id = kDamiaoRegisterFrameId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 8;
    frame.data.fill(0);
    frame.data[0] = motorId;
    frame.data[1] = 0;
    frame.data[2] = kDamiaoWriteRegisterCommand;
    frame.data[3] = registerId;
    frame.data[4] = static_cast<std::uint8_t>(value & 0xFFu);
    frame.data[5] = static_cast<std::uint8_t>((value >> 8) & 0xFFu);
    frame.data[6] = static_cast<std::uint8_t>((value >> 16) & 0xFFu);
    frame.data[7] = static_cast<std::uint8_t>((value >> 24) & 0xFFu);
    return submitTx(frame, CanTxDispatcher::Category::Config, source);
}

bool DamiaoCan::writeRegisterAndWaitForAck(std::uint8_t motorId,
                                           std::uint8_t registerId,
                                           std::uint32_t value,
                                           std::chrono::milliseconds timeout)
{
    if (!canController_ || !txDispatcher_) {
        return false;
    }

    const auto key = pendingRegisterAckKey(motorId, registerId);
    {
        std::lock_guard<std::mutex> lock(pendingRegisterAckMutex_);
        pendingRegisterAcks_[key] = PendingRegisterAck {};
    }

    if (!sendRegisterWrite(motorId, registerId, value, "DamiaoCan::sendRegisterWrite")) {
        return false;
    }

    std::unique_lock<std::mutex> lock(pendingRegisterAckMutex_);
    const bool received = pendingRegisterAckCv_.wait_for(
        lock, timeout, [this, key, value]() {
            const auto it = pendingRegisterAcks_.find(key);
            return it != pendingRegisterAcks_.end() &&
                   it->second.received &&
                   it->second.value == value;
        });
    pendingRegisterAcks_.erase(key);
    return received;
}

bool DamiaoCan::readRegisterAndWait(std::uint8_t motorId, std::uint8_t registerId,
                                    std::uint32_t &value)
{
    const auto key = pendingRegisterAckKey(motorId, registerId);
    {
        std::lock_guard<std::mutex> lock(pendingRegisterAckMutex_);
        pendingRegisterAcks_[key] = PendingRegisterAck{};
        pendingRegisterAcks_[key].command = 0x33;
    }
    CanTransport::Frame frame{};
    frame.id = kDamiaoRegisterFrameId;
    frame.dlc = 8;
    frame.data[0] = motorId; frame.data[2] = 0x33; frame.data[3] = registerId;
    if (!submitTx(frame, CanTxDispatcher::Category::Config, "DamiaoCan::readRegister")) {
        std::lock_guard<std::mutex> lock(pendingRegisterAckMutex_);
        pendingRegisterAcks_.erase(key);
        return false;
    }
    std::unique_lock<std::mutex> lock(pendingRegisterAckMutex_);
    const bool received = pendingRegisterAckCv_.wait_for(lock, kDamiaoRegisterAckTimeout,
        [this, key]() { return pendingRegisterAcks_.at(key).received; });
    if (received) value = pendingRegisterAcks_.at(key).value;
    pendingRegisterAcks_.erase(key);
    return received;
}

bool DamiaoCan::waitForFeedbackState(std::uint8_t motorId,
                                     std::uint8_t expectedState,
                                     std::chrono::milliseconds timeout,
                                     std::int64_t requestedAt) const
{
    std::unique_lock<std::mutex> lock(stateMutex_);
    return feedbackStateCv_.wait_for(
        lock, timeout, [this, motorId, expectedState, requestedAt]() {
            const auto it = motorStates_.find(motorId);
            return it != motorStates_.end() &&
                   it->second.feedbackSeen &&
                   it->second.feedbackSteadyNs[3] >= requestedAt &&
                   it->second.feedbackState == expectedState;
        });
}

bool DamiaoCan::ensureModeConfigured(std::uint8_t motorId, MotorMode mode)
{
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        const auto it = motorStates_.find(motorId);
        if (it != motorStates_.end()) {
            if (it->second.modeConfigured && it->second.mode == mode) return true;
            if (it->second.enabled) return false;
            if (mode == MotorMode::Position && (it->second.mitKp <= 0 || it->second.mitKd <= 0)) return false;
        } else if (mode == MotorMode::Position) return false;
        auto &state = motorStates_[motorId];
        state.modeConfigured = false;
        state.modeReceived = false;
        state.mitFrameValid = false;
        state.commandedVelocity = 0;
    }

    float initialPosition = 0;
    if (mode == MotorMode::Position) {
        std::uint32_t bits = 0;
        // V1.1 recommends precise position readback before entering MIT position control.
        if (!readRegisterAndWait(motorId, 0x50, bits)) return false;
        std::memcpy(&initialPosition, &bits, sizeof(initialPosition));
        if (!std::isfinite(initialPosition)) return false;
    }
    const std::uint32_t wireMode = mode == MotorMode::Position ? 1u : kDamiaoVelocityModeValue;
    if (!writeRegisterAndWaitForAck(motorId,
                                    kDamiaoControlModeRegister,
                                    wireMode,
                                    kDamiaoRegisterAckTimeout)) {
        ROS_ERROR("[DamiaoCan] Timed out waiting for control mode ack on motor %u.",
                  static_cast<unsigned>(motorId));
        return false;
    }
    std::this_thread::sleep_for(kDamiaoConfigStepDelay);

    std::uint32_t actualMode = 0;
    if (!readRegisterAndWait(motorId, kDamiaoControlModeRegister, actualMode) ||
        actualMode != wireMode) return false;

    // Read the motor's ranges; never overwrite model-specific parameters.
    if (!ensureFeedbackMappingConfigured(motorId)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        auto &state = motorStates_[motorId];
        if (mode == MotorMode::Position) {
            if (std::abs(initialPosition) > state.pMax) return false;
            state.position = rawPositionFromRadians(initialPosition);
            state.positionReceived = false; // Require a new status frame after switching.
        }
        state.modeConfigured = true;
        state.mode = mode;
    }
    syncSharedModeSelection(motorId, mode);
    return true;
}

bool DamiaoCan::ensureFeedbackMappingConfigured(std::uint8_t motorId)
{
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        const auto it = motorStates_.find(motorId);
        if (it != motorStates_.end() && it->second.feedbackRangeConfigured) {
            return true;
        }
    }

    std::array<float, 3> ranges{};
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        std::uint32_t bits = 0;
        if (!readRegisterAndWait(motorId, kDamiaoPMaxRegister + i, bits)) return false;
        std::memcpy(&ranges[i], &bits, sizeof(float));
        // Bound conversions to the fixed-point storage used below.
        const double limit = i == 0 ? 1e8 : (i == 1 ? 1e5 : 1e3);
        if (!std::isfinite(ranges[i]) || ranges[i] <= 0 || ranges[i] > limit) return false;
    }
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        auto &state = motorStates_[motorId];
        state.pMax = ranges[0]; state.vMax = ranges[1]; state.tMax = ranges[2];
        state.feedbackRangeConfigured = true;
    }
    return true;
}

bool DamiaoCan::submitTx(const CanTransport::Frame &frame,
                         CanTxDispatcher::Category category,
                         const char *source) const
{
    if (!txDispatcher_) {
        ROS_ERROR_STREAM_THROTTLE(1.0,
                                  "[DamiaoCan] TX dispatcher unavailable for "
                                  << (source ? source : "unknown"));
        return false;
    }

    CanTxDispatcher::Request request;
    request.frame = frame;
    request.category = category;
    request.source = source;
    // Configuration is confirmed by register replies; keepalive remains asynchronous.
    // Motion/stop callers must not report queue admission as a successful send.
    if (category == CanTxDispatcher::Category::Control ||
        category == CanTxDispatcher::Category::Recover) {
        auto completion = std::make_shared<std::promise<bool>>();
        auto result = completion->get_future();
        request.completion = [completion](bool attempted, CanTransport::SendResult sent,
                                         std::chrono::steady_clock::time_point) {
            completion->set_value(attempted && sent == CanTransport::SendResult::Ok);
        };
        txDispatcher_->submit(request);
        return result.wait_for(kDamiaoFeedbackStateTimeout) == std::future_status::ready &&
               result.get();
    }
    txDispatcher_->submit(request);
    return true;
}

bool DamiaoCan::isManagedMotorId(std::uint8_t motorId) const
{
    std::lock_guard<std::mutex> lock(refreshMutex_);
    return managedMotorIds_.find(motorId) != managedMotorIds_.end();
}

std::uint32_t DamiaoCan::masterIdForMotor(std::uint8_t motorId) const
{
    std::lock_guard<std::mutex> lock(refreshMutex_);
    const auto it = masterIdsByNodeId_.find(motorId);
    return (it == masterIdsByNodeId_.end()) ? kDamiaoMasterId : it->second;
}

void DamiaoCan::registerManagedMotorId(MotorID motorId)
{
    const auto nodeId = toProtocolNodeId(motorId);
    std::lock_guard<std::mutex> lock(refreshMutex_);
    managedMotorIds_.insert(nodeId);
    systemMotorIdsByNodeId_[nodeId] = motorId;
    if (sharedState_ && !deviceName_.empty()) {
        sharedState_->registerAxis(deviceName_, CanType::DM, motorId);
    }
}

MotorID DamiaoCan::resolveSystemMotorId(std::uint8_t motorId) const
{
    std::lock_guard<std::mutex> lock(refreshMutex_);
    const auto it = systemMotorIdsByNodeId_.find(motorId);
    return (it == systemMotorIdsByNodeId_.end()) ? motorIdFromProtocolNodeId(motorId)
                                                 : it->second;
}

can_driver::SharedDriverState::AxisKey DamiaoCan::makeAxisKey(std::uint8_t motorId) const
{
    return can_driver::MakeAxisKey(deviceName_, CanType::DM, resolveSystemMotorId(motorId));
}

void DamiaoCan::syncSharedFeedback(std::uint8_t motorId, const MotorState &state) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    sharedState_->mutateAxisFeedback(
        makeAxisKey(motorId),
        [&state, nowNs](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->position = state.position;
            feedback->velocity = state.velocity;
            feedback->current = state.current;
            feedback->mode = state.mode;
            feedback->positionValid = state.positionReceived;
            feedback->velocityValid = state.velocityReceived;
            feedback->currentValid = state.currentReceived;
            feedback->modeValid = state.modeReceived;
            feedback->enabled = state.enabled;
            feedback->fault = state.fault;
            feedback->enabledValid = state.enabledReceived;
            feedback->faultValid = state.faultReceived;
            feedback->feedbackSeen = state.feedbackSeen;
            feedback->feedbackSteadyNs = state.feedbackSteadyNs;
            feedback->lastRxSteadyNs = nowNs;
            feedback->lastValidStateSteadyNs = nowNs;
            feedback->consecutiveTimeoutCount = 0;
        });
}

void DamiaoCan::syncSharedCommand(std::uint8_t motorId,
                                  int32_t targetVelocity,
                                  bool valid) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    MotorMode mode;
    { std::lock_guard<std::mutex> lock(stateMutex_); mode = motorStates_[motorId].mode; }
    sharedState_->mutateAxisCommand(
        makeAxisKey(motorId),
        [targetVelocity, valid, nowNs, mode](can_driver::SharedDriverState::AxisCommandState *command) {
            command->targetPosition = 0;
            command->targetVelocity = targetVelocity;
            command->desiredMode = mode;
            command->desiredModeValid = true;
            command->valid = valid;
            command->lastCommandSteadyNs = valid ? nowNs : 0;
        });
}

void DamiaoCan::syncSharedModeSelection(std::uint8_t motorId, MotorMode desiredMode) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    sharedState_->mutateAxisCommand(
        makeAxisKey(motorId),
        [desiredMode](can_driver::SharedDriverState::AxisCommandState *command) {
            command->targetPosition = 0;
            command->targetVelocity = 0;
            command->desiredMode = desiredMode;
            command->desiredModeValid = true;
            command->valid = false;
            command->lastCommandSteadyNs = 0;
        });
}

void DamiaoCan::syncSharedIntent(std::uint8_t motorId, can_driver::AxisIntent intent) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }
    sharedState_->setAxisIntent(makeAxisKey(motorId), intent);
}

std::uint16_t DamiaoCan::pendingRegisterAckKey(std::uint8_t motorId, std::uint8_t registerId)
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(motorId) << 8) | registerId);
}

float DamiaoCan::uintToFloat(std::uint32_t raw, float minValue, float maxValue, int bits)
{
    const auto maxRaw = static_cast<float>((1u << bits) - 1u);
    return static_cast<float>(raw) * (maxValue - minValue) / maxRaw + minValue;
}

std::uint32_t DamiaoCan::floatToRawBits(float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

float DamiaoCan::rawToVelocityRadPerSec(int32_t rawVelocity)
{
    return static_cast<float>(static_cast<double>(rawVelocity) * kDamiaoProtocolScale);
}

int64_t DamiaoCan::rawPositionFromRadians(float positionRad)
{
    return static_cast<int64_t>(std::llround(
        static_cast<double>(positionRad) / kDamiaoProtocolScale));
}

int32_t DamiaoCan::rawVelocityFromRadiansPerSec(float velocityRadPerSec)
{
    return static_cast<int32_t>(std::lround(
        static_cast<double>(velocityRadPerSec) / kDamiaoProtocolScale));
}

int16_t DamiaoCan::rawTorqueFromNewtonMeters(float torqueNm)
{
    const auto scaled = static_cast<double>(torqueNm) * 100.0;
    const auto clamped = std::max(
        static_cast<double>(std::numeric_limits<int16_t>::min()),
        std::min(static_cast<double>(std::numeric_limits<int16_t>::max()), scaled));
    return static_cast<int16_t>(std::lround(clamped));
}

std::chrono::milliseconds DamiaoCan::computeRefreshSleep(std::size_t motorCount) const
{
    const auto hz = refreshRateHz_.load(std::memory_order_relaxed);
    if (std::isfinite(hz) && hz > 0.0) {
        const auto intervalMs = static_cast<int64_t>(std::llround(1000.0 / hz));
        return std::chrono::milliseconds(std::max<int64_t>(1, intervalMs));
    }

    const auto intervalMs = static_cast<int64_t>(std::max<std::size_t>(10, motorCount * 10));
    return std::chrono::milliseconds(intervalMs);
}
