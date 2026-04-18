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

} // namespace

InnfosEcbProtocol::InnfosEcbProtocol(std::string deviceSpec)
    : deviceSpec_(std::move(deviceSpec))
{
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
    ActuatorController::processEvents();
    modeCache_[rawMotorId] = mode;
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
        modeCache_[rawMotorId] = MotorMode::Velocity;
    }

    const double targetRpm = static_cast<double>(velocity) / kVelocityRawPerRpm;
    controller_->setVelocity(endpoint.actuatorId,
                             targetRpm,
                             endpoint.ipAddress);
    ActuatorController::processEvents();
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
        modeCache_[rawMotorId] = MotorMode::Position;
    }

    const double targetRev = static_cast<double>(position) / kPositionRawPerRevolution;
    controller_->setPosition(endpoint.actuatorId,
                             targetRev,
                             endpoint.ipAddress);
    ActuatorController::processEvents();
    return true;
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
    return ok;
}

bool InnfosEcbProtocol::Stop(MotorID motorId)
{
    return setVelocity(motorId, 0);
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

int16_t InnfosEcbProtocol::getVelocity(MotorID motorId) const
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
    cache.enabled = enabled;
    cache.fault = (errorCode != 0u);
    endpoints_[motorId].enabled = enabled;
    cache.valid = true;
    return true;
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
