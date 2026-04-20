#include "can_driver/InnfosEcbProtocol.h"

#include <ros/ros.h>

#include "actuatorcontroller.h"
#include "actuatordefine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <thread>

namespace {
constexpr const char *kEcbPrefix = "ecb://";
constexpr double kPositionRawPerRevolution = 10000.0; // 1 raw = 1e-4 rev
constexpr double kVelocityRawPerRpm = 10.0;           // 1 raw = 0.1 rpm
constexpr double kCurrentRawPerAmp = 1000.0;          // 1 raw = 1 mA
constexpr int kResolveRetryCount = 3;
constexpr int kResolveRetrySleepMs = 120;
constexpr int kReconnectRetryCount = 2;
constexpr int kReconnectRetrySleepMs = 80;
constexpr int kRecoveryCooldownMs = 500;

bool startsWith(const std::string &value, const std::string &prefix)
{
    return value.rfind(prefix, 0) == 0;
}

double clampToFinite(double value, double fallback)
{
    return std::isfinite(value) ? value : fallback;
}

std::string decodeEcbErrorCode(uint32_t code)
{
    if (code == ERR_NONE) {
        return "ERR_NONE";
    }

    switch (code) {
    case ERR_ACTUATOR_DISCONNECTION:
        return "ERR_ACTUATOR_DISCONNECTION";
    case ERR_CAN_DISCONNECTION:
        return "ERR_CAN_DISCONNECTION";
    case ERR_IP_ADDRESS_NOT_FOUND:
        return "ERR_IP_ADDRESS_NOT_FOUND";
    case ERR_ABNORMAL_SHUTDOWN:
        return "ERR_ABNORMAL_SHUTDOWN";
    case ERR_SHUTDOWN_SAVING:
        return "ERR_SHUTDOWN_SAVING";
    case ERR_IP_HAS_BIND:
        return "ERR_IP_HAS_BIND";
    case ERR_ID_UNUNIQUE:
        return "ERR_ID_UNUNIQUE";
    case ERR_IP_CONFLICT:
        return "ERR_IP_CONFLICT";
    case ERR_UNKOWN:
        return "ERR_UNKOWN";
    default:
        break;
    }

    std::string text;
    const auto append = [&text](uint32_t mask, const char *name, uint32_t value) {
        if ((value & mask) == 0u) {
            return;
        }
        if (!text.empty()) {
            text += "|";
        }
        text += name;
    };

    append(ERR_ACTUATOR_OVERVOLTAGE, "ERR_ACTUATOR_OVERVOLTAGE", code);
    append(ERR_ACTUATOR_UNDERVOLTAGE, "ERR_ACTUATOR_UNDERVOLTAGE", code);
    append(ERR_ACTUATOR_LOCKED_ROTOR, "ERR_ACTUATOR_LOCKED_ROTOR", code);
    append(ERR_ACTUATOR_OVERHEATING, "ERR_ACTUATOR_OVERHEATING", code);
    append(ERR_ACTUATOR_READ_OR_WRITE, "ERR_ACTUATOR_READ_OR_WRITE", code);
    append(ERR_ACTUATOR_MULTI_TURN, "ERR_ACTUATOR_MULTI_TURN", code);
    append(ERR_INVERTOR_TEMPERATURE_SENSOR, "ERR_INVERTOR_TEMPERATURE_SENSOR", code);
    append(ERR_CAN_COMMUNICATION, "ERR_CAN_COMMUNICATION", code);
    append(ERR_ACTUATOR_TEMPERATURE_SENSOR, "ERR_ACTUATOR_TEMPERATURE_SENSOR", code);
    append(ERR_STEP_OVER, "ERR_STEP_OVER", code);
    append(ERR_DRV_PROTECTION, "ERR_DRV_PROTECTION", code);
    append(ERR_CODER_DISABLED, "ERR_CODER_DISABLED", code);

    return text.empty() ? "ERR_UNKNOWN_BITS" : text;
}

bool sdkModeToMotorMode(Actuator::ActuatorMode sdkMode, CanProtocol::MotorMode *mode)
{
    if (mode == nullptr) {
        return false;
    }
    switch (sdkMode) {
    case Actuator::Mode_Profile_Vel:
    case Actuator::Mode_Vel:
        *mode = CanProtocol::MotorMode::Velocity;
        return true;
    case Actuator::Mode_Profile_Pos:
    case Actuator::Mode_Pos:
        *mode = CanProtocol::MotorMode::Position;
        return true;
    default:
        return false;
    }
}

const char *sdkModeName(Actuator::ActuatorMode sdkMode)
{
    switch (sdkMode) {
    case Actuator::Mode_None:
        return "Mode_None";
    case Actuator::Mode_Cur:
        return "Mode_Cur";
    case Actuator::Mode_Vel:
        return "Mode_Vel";
    case Actuator::Mode_Pos:
        return "Mode_Pos";
    case Actuator::Mode_Teaching:
        return "Mode_Teaching";
    case Actuator::Mode_Profile_Pos:
        return "Mode_Profile_Pos";
    case Actuator::Mode_Profile_Vel:
        return "Mode_Profile_Vel";
    case Actuator::Mode_Homing:
        return "Mode_Homing";
    default:
        return "Mode_Unknown";
    }
}

} // namespace

InnfosEcbProtocol::InnfosEcbProtocol(
    std::string deviceSpec,
    std::shared_ptr<can_driver::SharedDriverState> sharedState,
    std::string deviceName)
    : deviceSpec_(std::move(deviceSpec)),
      sharedState_(std::move(sharedState)),
      deviceName_(std::move(deviceName))
{
    if (deviceName_.empty()) {
        deviceName_ = deviceSpec_;
    }
    if (startsWith(deviceSpec_, kEcbPrefix)) {
        const std::string payload = deviceSpec_.substr(std::char_traits<char>::length(kEcbPrefix));
        if (payload == "auto" || payload.empty()) {
            defaultAutoDiscovery_ = true;
        } else {
            defaultIp_ = payload;
        }
    }
}

InnfosEcbProtocol::~InnfosEcbProtocol()
{
    stopRefreshThread();
}

bool InnfosEcbProtocol::setMode(MotorID motorId, MotorMode mode)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    if (rawMotorId > std::numeric_limits<uint8_t>::max()) {
        ROS_ERROR("[InnfosEcb] motor_id=%u out of range for ECB actuator id.",
                  static_cast<unsigned>(rawMotorId));
        return false;
    }
    if (!ensureEndpointReadyLocked(rawMotorId)) {
        return false;
    }

    const auto &endpoint = endpoints_[rawMotorId];
    const auto sdkMode = (mode == MotorMode::Velocity)
                             ? Actuator::Mode_Profile_Vel
                             : Actuator::Mode_Profile_Pos;
    controller_->activateActuatorMode(endpoint.actuatorId, sdkMode, endpoint.ipAddress);
    applyProfileForModeLocked(rawMotorId, mode);
    ActuatorController::processEvents();
    modeCache_[rawMotorId] = mode;
    syncSharedCommandLocked(rawMotorId, 0, 0, mode, false);
    (void)updateCacheLocked(rawMotorId);
    return true;
}

bool InnfosEcbProtocol::setVelocity(MotorID motorId, int32_t velocity)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    if (rawMotorId > std::numeric_limits<uint8_t>::max()) {
        ROS_ERROR("[InnfosEcb] motor_id=%u out of range for ECB actuator id.",
                  static_cast<unsigned>(rawMotorId));
        return false;
    }
    if (!ensureEndpointReadyLocked(rawMotorId)) {
        return false;
    }

    const auto &endpoint = endpoints_[rawMotorId];
    const auto velModeIt = modeCache_.find(rawMotorId);
    if (velModeIt == modeCache_.end() || velModeIt->second != MotorMode::Velocity) {
        controller_->activateActuatorMode(endpoint.actuatorId,
                                          Actuator::Mode_Profile_Vel,
                                          endpoint.ipAddress);
        applyProfileForModeLocked(rawMotorId, MotorMode::Velocity);
        modeCache_[rawMotorId] = MotorMode::Velocity;
    }

    const double targetRpm = static_cast<double>(velocity) / kVelocityRawPerRpm;
    ROS_INFO_THROTTLE(0.5,
                      "[InnfosEcb] setVelocity motor_id=%u raw=%d target=%.3f RPM.",
                      static_cast<unsigned>(rawMotorId),
                      velocity,
                      targetRpm);
    controller_->setVelocity(endpoint.actuatorId,
                             targetRpm,
                             endpoint.ipAddress);
    ActuatorController::processEvents();
    syncSharedCommandLocked(rawMotorId, 0, velocity, MotorMode::Velocity, true);
    syncSharedIntentLocked(rawMotorId, can_driver::AxisIntent::Run);
    return true;
}

bool InnfosEcbProtocol::setAcceleration(MotorID motorId, int32_t acceleration)
{
    (void)motorId;
    (void)acceleration;
    return true;
}

bool InnfosEcbProtocol::setDeceleration(MotorID motorId, int32_t deceleration)
{
    (void)motorId;
    (void)deceleration;
    return true;
}

bool InnfosEcbProtocol::setPosition(MotorID motorId, int32_t position)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    if (rawMotorId > std::numeric_limits<uint8_t>::max()) {
        ROS_ERROR("[InnfosEcb] motor_id=%u out of range for ECB actuator id.",
                  static_cast<unsigned>(rawMotorId));
        return false;
    }
    if (!ensureEndpointReadyLocked(rawMotorId)) {
        return false;
    }

    const auto &endpoint = endpoints_[rawMotorId];
    const auto posModeIt = modeCache_.find(rawMotorId);
    if (posModeIt == modeCache_.end() || posModeIt->second != MotorMode::Position) {
        controller_->activateActuatorMode(endpoint.actuatorId,
                                          Actuator::Mode_Profile_Pos,
                                          endpoint.ipAddress);
        applyProfileForModeLocked(rawMotorId, MotorMode::Position);
        modeCache_[rawMotorId] = MotorMode::Position;
    }

    const double targetRev = static_cast<double>(position) / kPositionRawPerRevolution;
    ROS_INFO_THROTTLE(0.5,
                      "[InnfosEcb] setPosition motor_id=%u raw=%d target=%.4f rev.",
                      static_cast<unsigned>(rawMotorId),
                      position,
                      targetRev);
    controller_->setPosition(endpoint.actuatorId,
                             targetRev,
                             endpoint.ipAddress);
    ActuatorController::processEvents();
    syncSharedCommandLocked(rawMotorId, position, 0, MotorMode::Position, true);
    syncSharedIntentLocked(rawMotorId, can_driver::AxisIntent::Run);
    return true;
}

bool InnfosEcbProtocol::quickSetPosition(MotorID motorId, int32_t position)
{
    return setPosition(motorId, position);
}

bool InnfosEcbProtocol::Enable(MotorID motorId)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    if (rawMotorId > std::numeric_limits<uint8_t>::max()) {
        ROS_ERROR("[InnfosEcb] motor_id=%u out of range for ECB actuator id.",
                  static_cast<unsigned>(rawMotorId));
        return false;
    }
    if (!ensureEndpointReadyLocked(rawMotorId)) {
        return false;
    }

    auto &endpoint = endpoints_[rawMotorId];
    endpoint.enabled = controller_->enableActuator(endpoint.actuatorId, endpoint.ipAddress);
    ActuatorController::processEvents();
    const auto modeIt = modeCache_.find(rawMotorId);
    if (endpoint.enabled && modeIt != modeCache_.end()) {
        const auto sdkMode = (modeIt->second == MotorMode::Velocity)
                                 ? Actuator::Mode_Profile_Vel
                                 : Actuator::Mode_Profile_Pos;
        controller_->activateActuatorMode(endpoint.actuatorId, sdkMode, endpoint.ipAddress);
        applyProfileForModeLocked(rawMotorId, modeIt->second);
        ActuatorController::processEvents();
        ROS_INFO("[InnfosEcb] re-activate mode after enable motor_id=%u mode=%s.",
                 static_cast<unsigned>(rawMotorId),
                 sdkModeName(sdkMode));
    }
    syncSharedIntentLocked(rawMotorId, can_driver::AxisIntent::Enable);
    (void)updateCacheLocked(rawMotorId);
    return endpoint.enabled;
}

bool InnfosEcbProtocol::Disable(MotorID motorId)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    if (rawMotorId > std::numeric_limits<uint8_t>::max()) {
        ROS_ERROR("[InnfosEcb] motor_id=%u out of range for ECB actuator id.",
                  static_cast<unsigned>(rawMotorId));
        return false;
    }
    if (!ensureEndpointReadyLocked(rawMotorId)) {
        return false;
    }

    auto &endpoint = endpoints_[rawMotorId];
    const bool ok = controller_->disableActuator(endpoint.actuatorId, endpoint.ipAddress);
    ActuatorController::processEvents();
    if (ok) {
        endpoint.enabled = false;
    }
    syncSharedIntentLocked(rawMotorId, can_driver::AxisIntent::Disable);
    (void)updateCacheLocked(rawMotorId);
    return ok;
}

bool InnfosEcbProtocol::Stop(MotorID motorId)
{
    const bool ok = setVelocity(motorId, 0);
    if (ok) {
        syncSharedIntentLocked(toRawMotorId(motorId), can_driver::AxisIntent::Hold);
    }
    return ok;
}

bool InnfosEcbProtocol::ResetFault(MotorID motorId)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    if (rawMotorId > std::numeric_limits<uint8_t>::max()) {
        return false;
    }
    if (!ensureEndpointReadyLocked(rawMotorId)) {
        return false;
    }

    const auto &endpoint = endpoints_[rawMotorId];
    controller_->clearError(endpoint.actuatorId, endpoint.ipAddress);
    ActuatorController::processEvents();
    syncSharedIntentLocked(rawMotorId, can_driver::AxisIntent::Recover);
    (void)updateCacheLocked(rawMotorId);
    const auto cacheIt = caches_.find(rawMotorId);
    const uint32_t errorCode =
        (cacheIt != caches_.end() && cacheIt->second.valid) ? cacheIt->second.errorCode : 0u;
    ROS_INFO("[InnfosEcb] clearError motor_id=%u, error_after=0x%08x (%s).",
             static_cast<unsigned>(rawMotorId),
             static_cast<unsigned>(errorCode),
             decodeEcbErrorCode(errorCode).c_str());
    return errorCode == 0u;
}

int64_t InnfosEcbProtocol::getPosition(MotorID motorId) const
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = caches_.find(rawMotorId);
    if (it != caches_.end() && it->second.valid) {
        return it->second.positionRaw;
    }
    if (updateCacheLocked(rawMotorId)) {
        return caches_[rawMotorId].positionRaw;
    }
    return 0;
}

int16_t InnfosEcbProtocol::getCurrent(MotorID motorId) const
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = caches_.find(rawMotorId);
    if (it != caches_.end() && it->second.valid) {
        return it->second.currentRaw;
    }
    if (updateCacheLocked(rawMotorId)) {
        return caches_[rawMotorId].currentRaw;
    }
    return 0;
}

int32_t InnfosEcbProtocol::getVelocity(MotorID motorId) const
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = caches_.find(rawMotorId);
    if (it != caches_.end() && it->second.valid) {
        return it->second.velocityRaw;
    }
    if (updateCacheLocked(rawMotorId)) {
        return caches_[rawMotorId].velocityRaw;
    }
    return 0;
}

bool InnfosEcbProtocol::isEnabled(MotorID motorId) const
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto cacheIt = caches_.find(rawMotorId);
    if (cacheIt != caches_.end() && cacheIt->second.valid) {
        return cacheIt->second.enabled;
    }
    const auto it = endpoints_.find(rawMotorId);
    return it != endpoints_.end() ? it->second.enabled : false;
}

bool InnfosEcbProtocol::hasFault(MotorID motorId) const
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = caches_.find(rawMotorId);
    if (it != caches_.end() && it->second.valid) {
        return it->second.fault;
    }
    if (updateCacheLocked(rawMotorId)) {
        return caches_[rawMotorId].fault;
    }
    return false;
}

bool InnfosEcbProtocol::configurePositionLimits(MotorID motorId,
                                                int32_t minPositionRaw,
                                                int32_t maxPositionRaw,
                                                bool enable)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    if (rawMotorId > std::numeric_limits<uint8_t>::max()) {
        return false;
    }
    if (!ensureEndpointReadyLocked(rawMotorId)) {
        return false;
    }
    const auto &endpoint = endpoints_[rawMotorId];
    controller_->setMinimumPosition(endpoint.actuatorId,
                                    static_cast<double>(minPositionRaw) / kPositionRawPerRevolution,
                                    endpoint.ipAddress);
    controller_->setMaximumPosition(endpoint.actuatorId,
                                    static_cast<double>(maxPositionRaw) / kPositionRawPerRevolution,
                                    endpoint.ipAddress);
    controller_->enablePositionLimit(endpoint.actuatorId, enable, endpoint.ipAddress);
    return true;
}

bool InnfosEcbProtocol::setPositionOffset(MotorID motorId, int32_t offsetRaw)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    if (rawMotorId > std::numeric_limits<uint8_t>::max()) {
        return false;
    }
    if (!ensureEndpointReadyLocked(rawMotorId)) {
        return false;
    }
    const auto &endpoint = endpoints_[rawMotorId];
    controller_->setPositionOffset(endpoint.actuatorId,
                                   static_cast<double>(offsetRaw) / kPositionRawPerRevolution,
                                   endpoint.ipAddress);
    return true;
}

void InnfosEcbProtocol::initializeMotorRefresh(const std::vector<MotorID> &motorIds)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        refreshIds_.clear();
        for (const auto id : motorIds) {
            refreshIds_.insert(toRawMotorId(id));
        }
    }

    if (motorIds.empty()) {
        stopRefreshThread();
        return;
    }

    if (!refreshRunning_.exchange(true)) {
        refreshThread_ = std::thread(&InnfosEcbProtocol::refreshLoop, this);
    }
}

void InnfosEcbProtocol::configureMotorRouting(MotorID motorId,
                                              const std::string &ipAddress,
                                              bool autoDiscovery)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    MotorRoute route;
    route.autoDiscovery = autoDiscovery || defaultAutoDiscovery_;
    route.fixedIp = ipAddress.empty() ? defaultIp_ : ipAddress;
    routes_[rawMotorId] = route;
}

void InnfosEcbProtocol::configureMotionProfile(MotorID motorId,
                                               double positionMaxRpm,
                                               double positionAccelerationRpmS,
                                               double positionDecelerationRpmS,
                                               double velocityAccelerationRpmS,
                                               double velocityDecelerationRpmS)
{
    const uint16_t rawMotorId = toRawMotorId(motorId);
    std::lock_guard<std::mutex> lock(mutex_);
    MotionProfile profile;
    profile.positionMaxRpm = clampToFinite(positionMaxRpm, profile.positionMaxRpm);
    profile.positionAccelerationRpmS =
        clampToFinite(positionAccelerationRpmS, profile.positionAccelerationRpmS);
    profile.positionDecelerationRpmS =
        clampToFinite(positionDecelerationRpmS, profile.positionDecelerationRpmS);
    profile.velocityAccelerationRpmS =
        clampToFinite(velocityAccelerationRpmS, profile.velocityAccelerationRpmS);
    profile.velocityDecelerationRpmS =
        clampToFinite(velocityDecelerationRpmS, profile.velocityDecelerationRpmS);
    motionProfiles_[rawMotorId] = profile;
}

void InnfosEcbProtocol::setRefreshRateHz(double hz)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(hz) || hz <= 0.0) {
        refreshIntervalMs_ = 20;
        return;
    }
    const double interval = 1000.0 / hz;
    refreshIntervalMs_ = std::max(1, static_cast<int>(std::round(interval)));
}

bool InnfosEcbProtocol::ensureControllerLocked() const
{
    if (controller_ != nullptr) {
        return true;
    }
    controller_ = ActuatorController::initController();
    if (controller_ == nullptr) {
        ROS_ERROR("[InnfosEcb] initController() returned null.");
        return false;
    }
    return true;
}

bool InnfosEcbProtocol::resolveEndpointLocked(uint16_t motorId) const
{
    auto endpointIt = endpoints_.find(motorId);
    if (endpointIt != endpoints_.end() && endpointIt->second.resolved) {
        return true;
    }
    if (!ensureControllerLocked()) {
        return false;
    }

    if (motorId > std::numeric_limits<uint8_t>::max()) {
        return false;
    }

    const auto routeIt = routes_.find(motorId);
    const bool routeAuto = (routeIt != routes_.end()) ? routeIt->second.autoDiscovery : defaultAutoDiscovery_;
    const std::string routeIp = (routeIt != routes_.end()) ? routeIt->second.fixedIp : defaultIp_;

    for (int attempt = 0; attempt < kResolveRetryCount; ++attempt) {
        if (!routeAuto && !routeIp.empty()) {
            MotorEndpoint endpoint;
            endpoint.actuatorId = static_cast<uint8_t>(motorId);
            endpoint.ipAddress = routeIp;
            endpoint.resolved = true;
            endpoints_[motorId] = endpoint;
            if (controller_->isOnline(endpoint.actuatorId, endpoint.ipAddress)) {
                return true;
            }
            (void)reconnectEndpointLocked(motorId);
            if (controller_->isOnline(endpoint.actuatorId, endpoint.ipAddress)) {
                return true;
            }
        }

        Actuator::ErrorsDefine ec = Actuator::ErrorsDefine(0);
        const auto unifiedIds = controller_->lookupActuators(ec);
        bool foundById = false;
        MotorEndpoint discovered;
        for (const auto &u : unifiedIds) {
            if (u.actuatorID != static_cast<uint8_t>(motorId)) {
                continue;
            }
            foundById = true;
            if (!routeIp.empty() && u.ipAddress != routeIp) {
                if (!routeAuto && attempt == (kResolveRetryCount - 1)) {
                    discovered.actuatorId = u.actuatorID;
                    discovered.ipAddress = u.ipAddress;
                    discovered.resolved = true;
                }
                continue;
            }
            discovered.actuatorId = u.actuatorID;
            discovered.ipAddress = u.ipAddress;
            discovered.resolved = true;
            break;
        }

        if (discovered.resolved) {
            endpoints_[motorId] = discovered;
            if (controller_->isOnline(discovered.actuatorId, discovered.ipAddress)) {
                if (!routeIp.empty() && discovered.ipAddress != routeIp) {
                    ROS_WARN_THROTTLE(1.0,
                                      "[InnfosEcb] motor_id=%u configured ip=%s unreachable, fallback discovered ip=%s.",
                                      static_cast<unsigned>(motorId),
                                      routeIp.c_str(),
                                      discovered.ipAddress.c_str());
                }
                return true;
            }
            (void)reconnectEndpointLocked(motorId);
            if (controller_->isOnline(discovered.actuatorId, discovered.ipAddress)) {
                return true;
            }
        }

        ROS_WARN_THROTTLE(1.0,
                          "[InnfosEcb] resolve attempt %d/%d failed for motor_id=%u (auto=%s, ip=%s, found_by_id=%s).",
                          attempt + 1,
                          kResolveRetryCount,
                          static_cast<unsigned>(motorId),
                          routeAuto ? "true" : "false",
                          routeIp.empty() ? "<empty>" : routeIp.c_str(),
                          foundById ? "true" : "false");
        std::this_thread::sleep_for(std::chrono::milliseconds(kResolveRetrySleepMs));
    }

    ROS_WARN_THROTTLE(1.0,
                      "[InnfosEcb] Unable to resolve actuator for motor_id=%u (auto=%s, ip=%s).",
                      static_cast<unsigned>(motorId),
                      routeAuto ? "true" : "false",
                      routeIp.empty() ? "<empty>" : routeIp.c_str());
    return false;
}

bool InnfosEcbProtocol::updateCacheLocked(uint16_t motorId) const
{
    if (motorId > std::numeric_limits<uint8_t>::max()) {
        return false;
    }
    if (!ensureEndpointReadyLocked(motorId)) {
        return false;
    }
    const auto &endpoint = endpoints_.at(motorId);

    const double posRev = clampToFinite(
        controller_->getPosition(endpoint.actuatorId, true, endpoint.ipAddress),
        0.0);
    const double velRpm = clampToFinite(
        controller_->getVelocity(endpoint.actuatorId, true, endpoint.ipAddress),
        0.0);
    const double curAmp = clampToFinite(
        controller_->getCurrent(endpoint.actuatorId, true, endpoint.ipAddress),
        0.0);
    const bool enabled = controller_->isEnable(endpoint.actuatorId, endpoint.ipAddress);
    const uint32_t errorCode = controller_->getErrorCode(endpoint.actuatorId, endpoint.ipAddress);
    const auto sdkMode = controller_->getActuatorMode(endpoint.actuatorId, endpoint.ipAddress);
    MotorMode actualMode = MotorMode::Position;
    const bool actualModeValid = sdkModeToMotorMode(sdkMode, &actualMode);

    auto &cache = caches_[motorId];
    cache.positionRaw = static_cast<int64_t>(std::llround(posRev * kPositionRawPerRevolution));
    const long long velRaw = std::llround(velRpm * kVelocityRawPerRpm);
    const long long currentMilliAmp = std::llround(curAmp * kCurrentRawPerAmp);
    cache.velocityRaw = static_cast<int16_t>(
        std::clamp<long long>(velRaw,
                              std::numeric_limits<int16_t>::min(),
                              std::numeric_limits<int16_t>::max()));
    cache.currentRaw = static_cast<int16_t>(
        std::clamp<long long>(currentMilliAmp,
                              std::numeric_limits<int16_t>::min(),
                              std::numeric_limits<int16_t>::max()));
    cache.errorCode = errorCode;
    cache.mode = actualMode;
    cache.modeValid = actualModeValid;
    cache.enabled = enabled;
    cache.fault = (errorCode != 0u);
    endpoints_[motorId].enabled = enabled;
    cache.valid = true;
    if (actualModeValid) {
        modeCache_[motorId] = actualMode;
    } else {
        ROS_WARN_THROTTLE(2.0,
                          "[InnfosEcb] motor_id=%u actual mode unsupported: %s(%d).",
                          static_cast<unsigned>(motorId),
                          sdkModeName(sdkMode),
                          static_cast<int>(sdkMode));
    }
    if (cache.fault) {
        ROS_WARN_THROTTLE(1.0,
                          "[InnfosEcb] motor_id=%u fault error_code=0x%08x (%s).",
                          static_cast<unsigned>(motorId),
                          static_cast<unsigned>(errorCode),
                          decodeEcbErrorCode(errorCode).c_str());
    }
    syncSharedFeedbackLocked(motorId, cache);
    return true;
}

can_driver::SharedDriverState::AxisKey InnfosEcbProtocol::makeAxisKey(uint16_t motorId) const
{
    return can_driver::MakeAxisKey(
        deviceName_, CanType::ECB, static_cast<MotorID>(motorId));
}

InnfosEcbProtocol::MotionProfile InnfosEcbProtocol::motionProfileLocked(uint16_t motorId) const
{
    const auto it = motionProfiles_.find(motorId);
    return (it == motionProfiles_.end()) ? MotionProfile{} : it->second;
}

void InnfosEcbProtocol::applyProfileForModeLocked(uint16_t motorId, MotorMode mode) const
{
    const auto endpointIt = endpoints_.find(motorId);
    if (endpointIt == endpoints_.end() || !endpointIt->second.resolved || controller_ == nullptr) {
        return;
    }

    const auto &endpoint = endpointIt->second;
    const auto profile = motionProfileLocked(motorId);
    if (mode == MotorMode::Velocity) {
        controller_->setProfileVelocityAcceleration(endpoint.actuatorId,
                                                    profile.velocityAccelerationRpmS,
                                                    endpoint.ipAddress);
        controller_->setProfileVelocityDeceleration(endpoint.actuatorId,
                                                    profile.velocityDecelerationRpmS,
                                                    endpoint.ipAddress);
        ROS_INFO_THROTTLE(2.0,
                          "[InnfosEcb] velocity profile motor_id=%u accel=%.3f RPM/s decel=%.3f RPM/s.",
                          static_cast<unsigned>(motorId),
                          profile.velocityAccelerationRpmS,
                          profile.velocityDecelerationRpmS);
        return;
    }

    controller_->setProfilePositionAcceleration(endpoint.actuatorId,
                                                profile.positionAccelerationRpmS,
                                                endpoint.ipAddress);
    controller_->setProfilePositionDeceleration(endpoint.actuatorId,
                                                profile.positionDecelerationRpmS,
                                                endpoint.ipAddress);
    controller_->setProfilePositionMaxVelocity(endpoint.actuatorId,
                                               profile.positionMaxRpm,
                                               endpoint.ipAddress);
    ROS_INFO_THROTTLE(2.0,
                      "[InnfosEcb] position profile motor_id=%u max=%.3f RPM accel=%.3f RPM/s decel=%.3f RPM/s.",
                      static_cast<unsigned>(motorId),
                      profile.positionMaxRpm,
                      profile.positionAccelerationRpmS,
                      profile.positionDecelerationRpmS);
}

void InnfosEcbProtocol::syncSharedFeedbackLocked(uint16_t motorId,
                                                 const MotorCache &cache) const
{
    if (!sharedState_ || deviceName_.empty() || !cache.valid) {
        return;
    }

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    sharedState_->mutateAxisFeedback(
        makeAxisKey(motorId),
        [&](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->position = cache.positionRaw;
            feedback->velocity = cache.velocityRaw;
            feedback->current = cache.currentRaw;
            if (cache.modeValid) {
                feedback->mode = cache.mode;
                feedback->modeValid = true;
            }
            feedback->positionValid = true;
            feedback->velocityValid = true;
            feedback->currentValid = true;
            feedback->enabled = cache.enabled;
            feedback->fault = cache.fault;
            feedback->enabledValid = true;
            feedback->faultValid = true;
            feedback->feedbackSeen = true;
            feedback->lastRxSteadyNs = nowNs;
            feedback->lastValidStateSteadyNs = nowNs;
            feedback->consecutiveTimeoutCount = 0;
        });
}

void InnfosEcbProtocol::syncSharedCommandLocked(uint16_t motorId,
                                                std::int64_t targetPosition,
                                                std::int32_t targetVelocity,
                                                MotorMode desiredMode,
                                                bool valid) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }

    const auto nowNs = can_driver::SharedDriverSteadyNowNs();
    sharedState_->mutateAxisCommand(
        makeAxisKey(motorId),
        [targetPosition, targetVelocity, desiredMode, valid, nowNs](
            can_driver::SharedDriverState::AxisCommandState *command) {
            command->targetPosition = targetPosition;
            command->targetVelocity = targetVelocity;
            command->desiredMode = desiredMode;
            command->desiredModeValid = true;
            command->valid = valid;
            command->lastCommandSteadyNs = nowNs;
        });
}

void InnfosEcbProtocol::syncSharedIntentLocked(uint16_t motorId,
                                               can_driver::AxisIntent intent) const
{
    if (!sharedState_ || deviceName_.empty()) {
        return;
    }
    sharedState_->setAxisIntent(makeAxisKey(motorId), intent);
}

bool InnfosEcbProtocol::ensureEndpointReadyLocked(uint16_t motorId) const
{
    if (!ensureControllerLocked()) {
        return false;
    }

    auto endpointIt = endpoints_.find(motorId);
    if (endpointIt != endpoints_.end() && endpointIt->second.resolved) {
        if (controller_->isOnline(endpointIt->second.actuatorId, endpointIt->second.ipAddress)) {
            return true;
        }
        if (canAttemptRecoveryLocked(motorId)) {
            (void)reconnectEndpointLocked(motorId);
            if (controller_->isOnline(endpointIt->second.actuatorId, endpointIt->second.ipAddress)) {
                return true;
            }
        } else {
            return false;
        }
    }

    if (!resolveEndpointLocked(motorId)) {
        return false;
    }

    endpointIt = endpoints_.find(motorId);
    if (endpointIt == endpoints_.end() || !endpointIt->second.resolved) {
        return false;
    }

    if (controller_->isOnline(endpointIt->second.actuatorId, endpointIt->second.ipAddress)) {
        return true;
    }

    if (!canAttemptRecoveryLocked(motorId)) {
        return false;
    }

    (void)reconnectEndpointLocked(motorId);
    return controller_->isOnline(endpointIt->second.actuatorId, endpointIt->second.ipAddress);
}

bool InnfosEcbProtocol::reconnectEndpointLocked(uint16_t motorId) const
{
    const auto endpointIt = endpoints_.find(motorId);
    if (endpointIt == endpoints_.end() || !endpointIt->second.resolved || !ensureControllerLocked()) {
        return false;
    }

    bool recovered = false;
    for (int i = 0; i < kReconnectRetryCount; ++i) {
        controller_->reconnect(endpointIt->second.actuatorId, endpointIt->second.ipAddress);
        ActuatorController::processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(kReconnectRetrySleepMs));
        if (controller_->isOnline(endpointIt->second.actuatorId, endpointIt->second.ipAddress)) {
            recovered = true;
            break;
        }
    }

    if (!recovered) {
        ROS_WARN_THROTTLE(1.0,
                          "[InnfosEcb] reconnect failed for motor_id=%u ip=%s.",
                          static_cast<unsigned>(motorId),
                          endpointIt->second.ipAddress.c_str());
    }
    return recovered;
}

bool InnfosEcbProtocol::canAttemptRecoveryLocked(uint16_t motorId) const
{
    const auto now = std::chrono::steady_clock::now();
    const auto it = nextRecoveryTry_.find(motorId);
    if (it != nextRecoveryTry_.end() && now < it->second) {
        return false;
    }
    nextRecoveryTry_[motorId] = now + std::chrono::milliseconds(kRecoveryCooldownMs);
    return true;
}

void InnfosEcbProtocol::refreshLoop()
{
    while (refreshRunning_.load()) {
        std::vector<uint16_t> ids;
        int intervalMs = 20;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ids.assign(refreshIds_.begin(), refreshIds_.end());
            intervalMs = refreshIntervalMs_;
            if (ensureControllerLocked()) {
                ActuatorController::processEvents();
            }
        }

        for (const auto id : ids) {
            std::lock_guard<std::mutex> lock(mutex_);
            (void)updateCacheLocked(id);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }
}

void InnfosEcbProtocol::stopRefreshThread()
{
    if (!refreshRunning_.exchange(false)) {
        return;
    }
    if (refreshThread_.joinable()) {
        refreshThread_.join();
    }
}

uint16_t InnfosEcbProtocol::toRawMotorId(MotorID motorId)
{
    return static_cast<uint16_t>(motorId);
}
