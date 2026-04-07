#include "can_driver/DeviceManager.h"
#include "can_driver/DeviceRuntime.h"
#include "can_driver/RefreshScheduler.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <ros/ros.h>

namespace {

constexpr std::uint64_t kQueryPressureHoldCycles = 20;

std::chrono::milliseconds clampRefreshSleep(std::chrono::milliseconds sleepFor)
{
    if (sleepFor < std::chrono::milliseconds(1)) {
        return std::chrono::milliseconds(1);
    }
    return sleepFor;
}

std::vector<std::uint8_t> normalizeMotorIds(const std::vector<MotorID> &motorIds)
{
    std::vector<std::uint8_t> normalized;
    normalized.reserve(motorIds.size());
    for (const auto motorId : motorIds) {
        normalized.push_back(can_driver::toProtocolNodeId(motorId));
    }
    return normalized;
}

can_driver::PpAxisRefreshSnapshot buildPpAxisRefreshSnapshot(
    const std::shared_ptr<can_driver::SharedDriverState> &sharedState,
    const std::string &deviceName,
    std::uint8_t motorId)
{
    can_driver::PpAxisRefreshSnapshot snapshot;
    if (!sharedState) {
        return snapshot;
    }

    const auto axisKey =
        can_driver::MakeAxisKey(deviceName, CanType::PP, static_cast<MotorID>(motorId));

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    if (sharedState->getAxisFeedback(axisKey, &feedback)) {
        snapshot.feedbackSeen = feedback.feedbackSeen;
        if (feedback.enabledValid) {
            snapshot.enabled = feedback.enabled;
        }
        if (feedback.faultValid) {
            snapshot.fault = feedback.fault;
        }
        snapshot.degraded = feedback.degraded;
        if (feedback.modeValid) {
            snapshot.feedbackMode = feedback.mode;
        }
    }

    can_driver::SharedDriverState::AxisCommandState command;
    if (sharedState->getAxisCommand(axisKey, &command)) {
        snapshot.desiredModeValid = command.desiredModeValid;
        snapshot.desiredMode = command.desiredMode;
    }

    snapshot.intent = sharedState->getAxisIntent(axisKey);
    return snapshot;
}

} // namespace

void DeviceManager::syncDeviceRefreshRuntimeLocked(const std::string &device)
{
    auto &runtime = deviceRefreshRuntimes_[device];
    if (!runtime) {
        runtime = std::make_shared<DeviceRefreshRuntime>();
    }
    runtime->deviceName = device;
    runtime->sharedState = sharedState_;
    const auto mtIt = mtProtocols_.find(device);
    runtime->mtProtocol =
        (mtIt != mtProtocols_.end()) ? mtIt->second : std::shared_ptr<MtCan>();
    const auto ppIt = eyouProtocols_.find(device);
    runtime->ppProtocol = (ppIt != eyouProtocols_.end()) ? ppIt->second
                                                         : std::shared_ptr<EyouCan>();

    if (runtime->worker) {
        return;
    }

    std::weak_ptr<DeviceRefreshRuntime> weakRuntime = runtime;
    runtime->worker = std::make_shared<can_driver::DeviceRefreshWorker>(
        [weakRuntime]() {
            const auto runtime = weakRuntime.lock();
            if (!runtime) {
                return;
            }

            bool queryPressureActive = false;
            const auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                const auto cycle = runtime->refreshCycleCount++;
                if (const auto sharedState = runtime->sharedState.lock()) {
                    can_driver::SharedDriverState::DeviceHealthState deviceHealth;
                    if (sharedState->getDeviceHealth(runtime->deviceName, &deviceHealth)) {
                        if (deviceHealth.txBackpressure > runtime->lastObservedTxBackpressure) {
                            runtime->queryPressureUntilCycle = std::max(
                                runtime->queryPressureUntilCycle, cycle + kQueryPressureHoldCycles);
                        }
                        runtime->lastObservedTxBackpressure = deviceHealth.txBackpressure;
                    }
                }
                queryPressureActive = cycle < runtime->queryPressureUntilCycle;
            }

            if (runtime->mtActive.load(std::memory_order_acquire)) {
                const auto protocol = runtime->mtProtocol.lock();
                bool due = false;
                std::uint64_t cycle = 0;
                std::vector<std::uint8_t> motorIds;
                {
                    std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                    due = (runtime->nextMtTick == std::chrono::steady_clock::time_point {}) ||
                          (now >= runtime->nextMtTick);
                    if (due) {
                        cycle = runtime->mtScheduleCycleCount++;
                        motorIds = runtime->mtMotorIds;
                    }
                }
                if (protocol && due) {
                    for (std::size_t i = 0; i < motorIds.size(); ++i) {
                        const auto plan =
                            can_driver::BuildMtRefreshPlan(cycle, i, queryPressureActive);
                        for (std::size_t j = 0; j < plan.count; ++j) {
                            protocol->issueRefreshQuery(
                                static_cast<MotorID>(motorIds[i]), plan.items[j]);
                        }
                    }
                    const auto nextSleep = clampRefreshSleep(protocol->refreshSleepInterval());
                    std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                    runtime->nextMtTick = std::chrono::steady_clock::now() + nextSleep;
                }
            } else {
                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                runtime->nextMtTick = std::chrono::steady_clock::time_point {};
            }

            if (runtime->ppActive.load(std::memory_order_acquire)) {
                const auto protocol = runtime->ppProtocol.lock();
                bool due = false;
                std::vector<std::uint8_t> motorIds;
                {
                    std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                    due = (runtime->nextPpTick == std::chrono::steady_clock::time_point {}) ||
                          (now >= runtime->nextPpTick);
                    if (due) {
                        ++runtime->ppScheduleCycleCount;
                        motorIds = runtime->ppMotorIds;
                    }
                }
                if (protocol && due) {
                    const auto sharedState = runtime->sharedState.lock();
                    for (std::size_t i = 0; i < motorIds.size(); ++i) {
                        const auto snapshot = buildPpAxisRefreshSnapshot(
                            sharedState, runtime->deviceName, motorIds[i]);
                        can_driver::PpRefreshPlan plan;
                        {
                            std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                            auto &scheduleState = runtime->ppScheduleStates[motorIds[i]];
                            plan = can_driver::BuildPpRefreshPlan(
                                now, queryPressureActive, snapshot, &scheduleState);
                            for (std::size_t j = 0; j < plan.count; ++j) {
                                can_driver::NotePpRefreshQueryDue(
                                    now, &scheduleState, plan.items[j]);
                            }
                        }
                        for (std::size_t j = 0; j < plan.count; ++j) {
                            const bool issued = protocol->issueRefreshQuery(
                                static_cast<MotorID>(motorIds[i]), plan.items[j]);
                            if (!issued) {
                                continue;
                            }
                            std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                            auto &scheduleState = runtime->ppScheduleStates[motorIds[i]];
                            can_driver::NotePpRefreshQueryIssued(
                                now, &scheduleState, plan.items[j]);
                        }
                    }
                    const auto nextSleep = clampRefreshSleep(protocol->refreshSleepInterval());
                    std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                    runtime->nextPpTick = std::chrono::steady_clock::now() + nextSleep;
                }
            } else {
                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                runtime->nextPpTick = std::chrono::steady_clock::time_point {};
            }
        },
        [weakRuntime]() {
            const auto runtime = weakRuntime.lock();
            if (!runtime) {
                return std::chrono::milliseconds(5);
            }

            const auto now = std::chrono::steady_clock::now();
            std::chrono::milliseconds sleepFor(5);
            bool hasPending = false;
            {
                std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
                if (runtime->mtActive.load(std::memory_order_acquire)) {
                    if (runtime->nextMtTick == std::chrono::steady_clock::time_point {}) {
                        return std::chrono::milliseconds(1);
                    }
                    const auto mtWait = std::chrono::duration_cast<std::chrono::milliseconds>(
                        runtime->nextMtTick > now ? (runtime->nextMtTick - now)
                                                  : std::chrono::steady_clock::duration::zero());
                    sleepFor = hasPending ? std::min(sleepFor, mtWait) : mtWait;
                    hasPending = true;
                }
                if (runtime->ppActive.load(std::memory_order_acquire)) {
                    if (runtime->nextPpTick == std::chrono::steady_clock::time_point {}) {
                        return std::chrono::milliseconds(1);
                    }
                    const auto ppWait = std::chrono::duration_cast<std::chrono::milliseconds>(
                        runtime->nextPpTick > now ? (runtime->nextPpTick - now)
                                                  : std::chrono::steady_clock::duration::zero());
                    sleepFor = hasPending ? std::min(sleepFor, ppWait) : ppWait;
                    hasPending = true;
                }
            }

            if (!hasPending) {
                return std::chrono::milliseconds(5);
            }
            return clampRefreshSleep(sleepFor);
        });
}

void DeviceManager::startDeviceRefreshWorkerLocked(const std::string &device)
{
    const auto it = deviceRefreshRuntimes_.find(device);
    if (it == deviceRefreshRuntimes_.end() || !it->second) {
        return;
    }

    const auto &runtime = it->second;
    const bool anyActive = runtime->mtActive.load(std::memory_order_acquire) ||
                           runtime->ppActive.load(std::memory_order_acquire);
    if (!anyActive || !runtime->worker) {
        return;
    }

    runtime->worker->start();
    runtime->worker->notify();
}

double DeviceManager::normalizeRefreshRateHz(double hz)
{
    return (std::isfinite(hz) && hz > 0.0) ? hz : 0.0;
}

double DeviceManager::effectiveRefreshRateHzLocked(const std::string &device) const
{
    const auto overrideIt = deviceRefreshRateOverrides_.find(device);
    if (overrideIt != deviceRefreshRateOverrides_.end()) {
        return overrideIt->second;
    }
    return refreshRateHz_;
}

void DeviceManager::applyRefreshRateLocked(const std::string &device, double hz)
{
    if (const auto mtIt = mtProtocols_.find(device); mtIt != mtProtocols_.end() && mtIt->second) {
        mtIt->second->setRefreshRateHz(hz);
    }
    if (const auto ppIt = eyouProtocols_.find(device);
        ppIt != eyouProtocols_.end() && ppIt->second) {
        ppIt->second->setRefreshRateHz(hz);
    }
    const auto runtimeIt = deviceRefreshRuntimes_.find(device);
    if (runtimeIt == deviceRefreshRuntimes_.end() || !runtimeIt->second) {
        return;
    }

    {
        std::lock_guard<std::mutex> scheduleLock(runtimeIt->second->scheduleMutex);
        runtimeIt->second->nextMtTick = std::chrono::steady_clock::time_point {};
        runtimeIt->second->nextPpTick = std::chrono::steady_clock::time_point {};
    }
    if (runtimeIt->second->worker) {
        runtimeIt->second->worker->notify();
    }
}

void DeviceManager::stopDeviceRefreshWorkerLocked(const std::string &device)
{
    const auto it = deviceRefreshRuntimes_.find(device);
    if (it == deviceRefreshRuntimes_.end()) {
        return;
    }

    if (it->second && it->second->worker) {
        it->second->worker->stop();
    }
    deviceRefreshRuntimes_.erase(it);
}

void DeviceManager::stopAllDeviceRefreshWorkersLocked()
{
    for (auto &entry : deviceRefreshRuntimes_) {
        if (entry.second && entry.second->worker) {
            entry.second->worker->stop();
        }
    }
    deviceRefreshRuntimes_.clear();
}

void DeviceManager::resetDeviceRuntimeLocked(const std::string &device)
{
    stopDeviceRefreshWorkerLocked(device);
    mtProtocols_.erase(device);
    eyouProtocols_.erase(device);
    txDispatchers_.erase(device);
}

void DeviceManager::shutdownDeviceLocked(const std::string &device)
{
    const auto transportIt = transports_.find(device);
    const auto transport =
        (transportIt != transports_.end()) ? transportIt->second : std::shared_ptr<SocketCanController>();

    if (sharedState_) {
        const auto stats = transport ? transport->snapshotStats() : SocketCanController::Stats {};
        sharedState_->mutateDeviceHealth(
            device,
            [&stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = false;
                health->txBackpressure = stats.txBackpressure;
                health->txLinkUnavailable = stats.txLinkUnavailable;
                health->txError = stats.txError;
                health->rxError = stats.rxError;
                health->lastTxLinkUnavailableSteadyNs = stats.lastTxLinkUnavailableSteadyNs;
                health->lastRxSteadyNs = stats.lastRxSteadyNs;
            });
    }

    resetDeviceRuntimeLocked(device);
    if (transport) {
        transport->shutdown();
    }
    transports_.erase(device);
    deviceCmdMutexes_.erase(device);
}

// 幂等创建 transport：同一 device 只创建一次。
bool DeviceManager::ensureTransport(const std::string &device, bool loopback)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    auto it = transports_.find(device);
    if (it != transports_.end()) {
        return true;
    }

    // 创建底层 SocketCAN 传输并尝试初始化。
    auto transport = std::make_shared<SocketCanController>();
    if (!transport->initialize(device, loopback)) {
        ROS_ERROR("[CanDriverHW] Failed to initialize CAN device '%s'.", device.c_str());
        return false;
    }

    transports_[device] = transport;
    txDispatchers_[device] = std::make_shared<DeviceRuntime>(transport, device);
    std::static_pointer_cast<DeviceRuntime>(txDispatchers_[device])->setSharedDriverState(sharedState_);
    if (sharedState_) {
        const auto stats = transport->snapshotStats();
        sharedState_->mutateDeviceHealth(
            device,
            [&stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = true;
                health->txBackpressure = stats.txBackpressure;
                health->txLinkUnavailable = stats.txLinkUnavailable;
                health->txError = stats.txError;
                health->rxError = stats.rxError;
                health->lastTxLinkUnavailableSteadyNs = stats.lastTxLinkUnavailableSteadyNs;
                health->lastRxSteadyNs = stats.lastRxSteadyNs;
            });
    }
    // 同步创建该设备的命令互斥锁，供上层 write() 串行下发命令使用。
    deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    ROS_INFO("[CanDriverHW] Opened CAN device '%s'.", device.c_str());
    return true;
}

bool DeviceManager::ensureProtocol(const std::string &device, CanType type)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    auto transportIt = transports_.find(device);
    // protocol 依赖 transport，未就绪直接失败。
    if (transportIt == transports_.end()) {
        return false;
    }
    auto transport = transportIt->second;
    auto txDispatcherIt = txDispatchers_.find(device);
    if (txDispatcherIt == txDispatchers_.end()) {
        txDispatchers_[device] = std::make_shared<DeviceRuntime>(transport, device);
        txDispatcherIt = txDispatchers_.find(device);
    }
    auto txDispatcher = txDispatcherIt->second;

    // 按协议类型按需懒加载实例。
    if (type == CanType::MT) {
        if (mtProtocols_.find(device) == mtProtocols_.end()) {
            auto mt = std::make_shared<MtCan>(transport, txDispatcher, sharedState_, device);
            mt->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
            mtProtocols_[device] = std::move(mt);
        }
    } else {
        if (eyouProtocols_.find(device) == eyouProtocols_.end()) {
            auto eyou =
                std::make_shared<EyouCan>(transport, txDispatcher, sharedState_, device);
            eyou->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
            eyou->setFastWriteEnabled(ppFastWriteEnabled_);
            eyou->setDefaultPositionVelocityRaw(ppPositionDefaultVelocityRaw_);
            eyou->setDefaultCspVelocityRaw(ppCspDefaultVelocityRaw_);
            eyouProtocols_[device] = std::move(eyou);
        }
    }
    return true;
}

bool DeviceManager::initDevice(const std::string &device,
                               const std::vector<std::pair<CanType, MotorID>> &motors,
                               bool loopback)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    // 已存在 transport 时先 shutdown 再 re-init，确保监听器和内部状态被重置。
    auto transportIt = transports_.find(device);
    if (transportIt == transports_.end()) {
        auto transport = std::make_shared<SocketCanController>();
        if (!transport->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Failed to init '%s'.", device.c_str());
            return false;
        }
        transports_[device] = transport;
        txDispatchers_[device] = std::make_shared<DeviceRuntime>(transport, device);
        std::static_pointer_cast<DeviceRuntime>(txDispatchers_[device])->setSharedDriverState(sharedState_);
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
        transportIt = transports_.find(device);
    } else {
        const auto &transport = transportIt->second;
        // Re-init must rebuild protocol handlers and TX runtime for this bus.
        resetDeviceRuntimeLocked(device);
        transport->shutdown();
        if (!transport->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Re-init of '%s' failed.", device.c_str());
            shutdownDeviceLocked(device);
            return false;
        }
    }

    if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    }
    txDispatchers_[device] = std::make_shared<DeviceRuntime>(transportIt->second, device);
    std::static_pointer_cast<DeviceRuntime>(txDispatchers_[device])->setSharedDriverState(sharedState_);
    if (sharedState_) {
        for (const auto &entry : motors) {
            sharedState_->registerAxis(device, entry.first, entry.second);
        }
        const auto stats = transportIt->second->snapshotStats();
        sharedState_->mutateDeviceHealth(
            device,
            [&stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = true;
                health->txBackpressure = stats.txBackpressure;
                health->txLinkUnavailable = stats.txLinkUnavailable;
                health->txError = stats.txError;
                health->rxError = stats.rxError;
                health->lastTxLinkUnavailableSteadyNs = stats.lastTxLinkUnavailableSteadyNs;
                health->lastRxSteadyNs = stats.lastRxSteadyNs;
            });
    }

    // 按协议拆分电机列表，避免不必要地创建协议实例。
    std::vector<MotorID> mtIds;
    std::vector<MotorID> ppIds;
    for (const auto &entry : motors) {
        if (entry.first == CanType::MT) {
            mtIds.push_back(entry.second);
        } else {
            ppIds.push_back(entry.second);
        }
    }
    // 初始化协议对象并启动状态刷新任务。
    if (!mtIds.empty() && mtProtocols_.find(device) == mtProtocols_.end()) {
        auto mt = std::make_shared<MtCan>(
            transportIt->second, txDispatchers_[device], sharedState_, device);
        mt->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
        mtProtocols_[device] = std::move(mt);
    }
    if (!ppIds.empty() && eyouProtocols_.find(device) == eyouProtocols_.end()) {
        auto eyou = std::make_shared<EyouCan>(
            transportIt->second, txDispatchers_[device], sharedState_, device);
        eyou->setRefreshRateHz(effectiveRefreshRateHzLocked(device));
        eyou->setFastWriteEnabled(ppFastWriteEnabled_);
        eyou->setDefaultPositionVelocityRaw(ppPositionDefaultVelocityRaw_);
        eyou->setDefaultCspVelocityRaw(ppCspDefaultVelocityRaw_);
        eyouProtocols_[device] = std::move(eyou);
    }

    syncDeviceRefreshRuntimeLocked(device);
    const auto runtime = deviceRefreshRuntimes_[device];

    if (!mtIds.empty()) {
        mtProtocols_[device]->initializeMotorRefresh(mtIds);
        runtime->mtActive.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->mtMotorIds = normalizeMotorIds(mtIds);
        runtime->mtScheduleCycleCount = 0;
        runtime->nextMtTick = std::chrono::steady_clock::time_point {};
    } else {
        runtime->mtActive.store(false, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->mtMotorIds.clear();
        runtime->mtScheduleCycleCount = 0;
        runtime->nextMtTick = std::chrono::steady_clock::time_point {};
    }
    if (!ppIds.empty()) {
        eyouProtocols_[device]->initializeMotorRefresh(ppIds);
        runtime->ppActive.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->ppMotorIds = normalizeMotorIds(ppIds);
        runtime->ppScheduleStates.clear();
        runtime->ppScheduleCycleCount = 0;
        runtime->nextPpTick = std::chrono::steady_clock::time_point {};
    } else {
        runtime->ppActive.store(false, std::memory_order_release);
        std::lock_guard<std::mutex> lock(runtime->scheduleMutex);
        runtime->ppMotorIds.clear();
        runtime->ppScheduleStates.clear();
        runtime->ppScheduleCycleCount = 0;
        runtime->nextPpTick = std::chrono::steady_clock::time_point {};
    }
    startDeviceRefreshWorkerLocked(device);

    ROS_INFO("[CanDriverHW] Initialized '%s'.", device.c_str());
    return true;
}

void DeviceManager::startRefresh(const std::string &device,
                                 CanType type,
                                 const std::vector<MotorID> &ids)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (type == CanType::MT) {
        auto it = mtProtocols_.find(device);
        if (it != mtProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
            syncDeviceRefreshRuntimeLocked(device);
            auto runtime = deviceRefreshRuntimes_[device];
            runtime->mtActive.store(!ids.empty(), std::memory_order_release);
            const auto normalizedIds = normalizeMotorIds(ids);
            {
                std::lock_guard<std::mutex> scheduleLock(runtime->scheduleMutex);
                if (runtime->mtMotorIds != normalizedIds) {
                    runtime->mtMotorIds = normalizedIds;
                    runtime->mtScheduleCycleCount = 0;
                }
                runtime->nextMtTick = std::chrono::steady_clock::time_point {};
            }
            if (runtime->mtActive.load(std::memory_order_acquire) ||
                runtime->ppActive.load(std::memory_order_acquire)) {
                startDeviceRefreshWorkerLocked(device);
            } else {
                stopDeviceRefreshWorkerLocked(device);
            }
        }
    } else {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
            syncDeviceRefreshRuntimeLocked(device);
            auto runtime = deviceRefreshRuntimes_[device];
            runtime->ppActive.store(!ids.empty(), std::memory_order_release);
            const auto normalizedIds = normalizeMotorIds(ids);
            {
                std::lock_guard<std::mutex> scheduleLock(runtime->scheduleMutex);
                if (runtime->ppMotorIds != normalizedIds) {
                    runtime->ppMotorIds = normalizedIds;
                    runtime->ppScheduleStates.clear();
                    runtime->ppScheduleCycleCount = 0;
                }
                runtime->nextPpTick = std::chrono::steady_clock::time_point {};
            }
            if (runtime->mtActive.load(std::memory_order_acquire) ||
                runtime->ppActive.load(std::memory_order_acquire)) {
                startDeviceRefreshWorkerLocked(device);
            } else {
                stopDeviceRefreshWorkerLocked(device);
            }
        }
    }
}

void DeviceManager::setRefreshRateHz(double hz)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    refreshRateHz_ = normalizeRefreshRateHz(hz);
    for (const auto &kv : deviceCmdMutexes_) {
        applyRefreshRateLocked(kv.first, effectiveRefreshRateHzLocked(kv.first));
    }
}

void DeviceManager::setDeviceRefreshRateHz(const std::string &device, double hz)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    const double normalizedHz = normalizeRefreshRateHz(hz);
    if (normalizedHz > 0.0) {
        deviceRefreshRateOverrides_[device] = normalizedHz;
    } else {
        deviceRefreshRateOverrides_.erase(device);
    }

    if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
        return;
    }
    applyRefreshRateLocked(device, effectiveRefreshRateHzLocked(device));
}

void DeviceManager::setPpFastWriteEnabled(bool enabled)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    ppFastWriteEnabled_ = enabled;
    for (auto &kv : eyouProtocols_) {
        if (kv.second) {
            kv.second->setFastWriteEnabled(enabled);
        }
    }
}

void DeviceManager::setPpDefaultPositionVelocityRaw(int32_t velocityRaw)
{
    setPpPositionDefaultVelocityRaw(velocityRaw);
    setPpCspDefaultVelocityRaw(velocityRaw);
}

void DeviceManager::setPpPositionDefaultVelocityRaw(int32_t velocityRaw)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (velocityRaw <= 0) {
        ROS_WARN("[CanDriverHW] Ignore invalid pp_position_default_velocity_raw=%d.", velocityRaw);
        return;
    }
    ppPositionDefaultVelocityRaw_ = velocityRaw;
    for (auto &kv : eyouProtocols_) {
        if (kv.second) {
            kv.second->setDefaultPositionVelocityRaw(velocityRaw);
        }
    }
}

void DeviceManager::setPpCspDefaultVelocityRaw(int32_t velocityRaw)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (velocityRaw <= 0) {
        ROS_WARN("[CanDriverHW] Ignore invalid pp_csp_default_velocity_raw=%d.", velocityRaw);
        return;
    }
    ppCspDefaultVelocityRaw_ = velocityRaw;
    for (auto &kv : eyouProtocols_) {
        if (kv.second) {
            kv.second->setDefaultCspVelocityRaw(velocityRaw);
        }
    }
}

void DeviceManager::shutdownAll()
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    std::vector<std::string> devices;
    devices.reserve(transports_.size());
    for (const auto &kv : transports_) {
        devices.push_back(kv.first);
    }

    for (const auto &device : devices) {
        shutdownDeviceLocked(device);
    }

    stopAllDeviceRefreshWorkersLocked();
    mtProtocols_.clear();
    eyouProtocols_.clear();
    txDispatchers_.clear();
    transports_.clear();
    deviceCmdMutexes_.clear();
}

void DeviceManager::shutdownDevice(const std::string &device)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    shutdownDeviceLocked(device);
}

std::shared_ptr<CanProtocol> DeviceManager::getProtocol(const std::string &device, CanType type) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (type == CanType::MT) {
        auto it = mtProtocols_.find(device);
        if (it != mtProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    } else {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    }
    return nullptr;
}

std::shared_ptr<std::mutex> DeviceManager::getDeviceMutex(const std::string &device) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    auto it = deviceCmdMutexes_.find(device);
    return (it != deviceCmdMutexes_.end()) ? it->second : nullptr;
}

std::shared_ptr<SocketCanController> DeviceManager::getTransport(const std::string &device) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    auto it = transports_.find(device);
    return (it != transports_.end()) ? it->second : nullptr;
}

bool DeviceManager::isDeviceReady(const std::string &device) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    auto it = transports_.find(device);
    if (it == transports_.end() || !it->second) {
        return false;
    }
    const auto stats = it->second->snapshotStats();
    const bool linkRecovered =
        stats.lastTxLinkUnavailableSteadyNs == 0 ||
        stats.lastRxSteadyNs > stats.lastTxLinkUnavailableSteadyNs;
    const bool ready = it->second->isReady() && linkRecovered;
    if (sharedState_) {
        sharedState_->mutateDeviceHealth(
            device,
            [ready, &stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = ready;
                health->txBackpressure = stats.txBackpressure;
                health->txLinkUnavailable = stats.txLinkUnavailable;
                health->txError = stats.txError;
                health->rxError = stats.rxError;
                health->lastTxLinkUnavailableSteadyNs = stats.lastTxLinkUnavailableSteadyNs;
                health->lastRxSteadyNs = stats.lastRxSteadyNs;
            });
    }
    return ready;
}

std::size_t DeviceManager::deviceCount() const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return transports_.size();
}

std::shared_ptr<can_driver::SharedDriverState> DeviceManager::getSharedDriverState() const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return sharedState_;
}
