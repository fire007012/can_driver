#include "can_driver/DeviceManager.h"
#include "can_driver/DeviceRuntime.h"

#include <chrono>

#include <ros/ros.h>

void DeviceManager::startRefreshWorkerLocked(const std::string &device, CanType type)
{
    if (type == CanType::MT) {
        const auto protoIt = mtProtocols_.find(device);
        if (protoIt == mtProtocols_.end() || !protoIt->second) {
            return;
        }

        auto &worker = mtRefreshWorkers_[device];
        if (!worker) {
            std::weak_ptr<MtCan> weakProtocol = protoIt->second;
            worker = std::make_shared<can_driver::ProtocolRefreshWorker>(
                [weakProtocol]() {
                    if (const auto protocol = weakProtocol.lock()) {
                        protocol->runRefreshCycle();
                    }
                },
                [weakProtocol]() {
                    if (const auto protocol = weakProtocol.lock()) {
                        return protocol->refreshSleepInterval();
                    }
                    return std::chrono::milliseconds(5);
                });
        }

        worker->start();
        worker->notify();
        return;
    }

    const auto protoIt = eyouProtocols_.find(device);
    if (protoIt == eyouProtocols_.end() || !protoIt->second) {
        return;
    }

    auto &worker = eyouRefreshWorkers_[device];
    if (!worker) {
        std::weak_ptr<EyouCan> weakProtocol = protoIt->second;
        worker = std::make_shared<can_driver::ProtocolRefreshWorker>(
            [weakProtocol]() {
                if (const auto protocol = weakProtocol.lock()) {
                    protocol->runRefreshCycle();
                }
            },
            [weakProtocol]() {
                if (const auto protocol = weakProtocol.lock()) {
                    return protocol->refreshSleepInterval();
                }
                return std::chrono::milliseconds(5);
            });
    }

    worker->start();
    worker->notify();
}

void DeviceManager::stopRefreshWorkerLocked(const std::string &device, CanType type)
{
    auto *workers = (type == CanType::MT) ? &mtRefreshWorkers_ : &eyouRefreshWorkers_;
    const auto it = workers->find(device);
    if (it == workers->end()) {
        return;
    }

    if (it->second) {
        it->second->stop();
    }
    workers->erase(it);
}

void DeviceManager::stopAllRefreshWorkersLocked()
{
    for (auto &entry : mtRefreshWorkers_) {
        if (entry.second) {
            entry.second->stop();
        }
    }
    mtRefreshWorkers_.clear();

    for (auto &entry : eyouRefreshWorkers_) {
        if (entry.second) {
            entry.second->stop();
        }
    }
    eyouRefreshWorkers_.clear();
}

void DeviceManager::resetDeviceRuntimeLocked(const std::string &device)
{
    stopRefreshWorkerLocked(device, CanType::MT);
    stopRefreshWorkerLocked(device, CanType::PP);
    mtProtocols_.erase(device);
    eyouProtocols_.erase(device);
    txDispatchers_.erase(device);
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
            mtProtocols_[device] =
                std::make_shared<MtCan>(transport, txDispatcher, sharedState_, device);
        }
    } else {
        if (eyouProtocols_.find(device) == eyouProtocols_.end()) {
            auto eyou =
                std::make_shared<EyouCan>(transport, txDispatcher, sharedState_, device);
            eyou->setFastWriteEnabled(ppFastWriteEnabled_);
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
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
        transportIt = transports_.find(device);
    } else {
        const auto &transport = transportIt->second;
        // Re-init must rebuild protocol handlers and TX runtime for this bus.
        resetDeviceRuntimeLocked(device);
        transport->shutdown();
        if (!transport->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Re-init of '%s' failed.", device.c_str());
            return false;
        }
    }

    if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    }
    txDispatchers_[device] = std::make_shared<DeviceRuntime>(transportIt->second, device);
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
        mtProtocols_[device] = std::make_shared<MtCan>(
            transportIt->second, txDispatchers_[device], sharedState_, device);
    }
    if (!ppIds.empty() && eyouProtocols_.find(device) == eyouProtocols_.end()) {
        auto eyou = std::make_shared<EyouCan>(
            transportIt->second, txDispatchers_[device], sharedState_, device);
        eyou->setFastWriteEnabled(ppFastWriteEnabled_);
        eyouProtocols_[device] = std::move(eyou);
    }
    if (!mtIds.empty()) {
        mtProtocols_[device]->initializeMotorRefresh(mtIds);
        startRefreshWorkerLocked(device, CanType::MT);
    } else {
        stopRefreshWorkerLocked(device, CanType::MT);
    }
    if (!ppIds.empty()) {
        eyouProtocols_[device]->initializeMotorRefresh(ppIds);
        startRefreshWorkerLocked(device, CanType::PP);
    } else {
        stopRefreshWorkerLocked(device, CanType::PP);
    }

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
            if (ids.empty()) {
                stopRefreshWorkerLocked(device, type);
            } else {
                startRefreshWorkerLocked(device, type);
            }
        }
    } else {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
            if (ids.empty()) {
                stopRefreshWorkerLocked(device, type);
            } else {
                startRefreshWorkerLocked(device, type);
            }
        }
    }
}

void DeviceManager::setRefreshRateHz(double hz)
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    for (auto &kv : mtProtocols_) {
        if (kv.second) {
            kv.second->setRefreshRateHz(hz);
        }
    }
    for (auto &kv : eyouProtocols_) {
        if (kv.second) {
            kv.second->setRefreshRateHz(hz);
        }
    }
    for (auto &kv : mtRefreshWorkers_) {
        if (kv.second) {
            kv.second->notify();
        }
    }
    for (auto &kv : eyouRefreshWorkers_) {
        if (kv.second) {
            kv.second->notify();
        }
    }
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

void DeviceManager::shutdownAll()
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (sharedState_) {
        for (const auto &kv : transports_) {
            const auto stats =
                kv.second ? kv.second->snapshotStats() : SocketCanController::Stats {};
            sharedState_->mutateDeviceHealth(
                kv.first,
                [&stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                    health->transportReady = false;
                    health->txBackpressure = stats.txBackpressure;
                    health->txLinkUnavailable = stats.txLinkUnavailable;
                    health->txError = stats.txError;
                    health->rxError = stats.rxError;
                    health->lastRxSteadyNs = stats.lastRxSteadyNs;
                });
        }
    }
    stopAllRefreshWorkersLocked();
    // 先释放协议（包含内部线程/handler），再关闭 transport。
    mtProtocols_.clear();
    eyouProtocols_.clear();
    txDispatchers_.clear();
    for (auto &kv : transports_) {
        kv.second->shutdown();
    }
    transports_.clear();
    deviceCmdMutexes_.clear();
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
    const bool ready = it->second->isReady();
    if (sharedState_) {
        const auto stats = it->second->snapshotStats();
        sharedState_->mutateDeviceHealth(
            device,
            [ready, &stats](can_driver::SharedDriverState::DeviceHealthState *health) {
                health->transportReady = ready;
                health->txBackpressure = stats.txBackpressure;
                health->txLinkUnavailable = stats.txLinkUnavailable;
                health->txError = stats.txError;
                health->rxError = stats.rxError;
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
