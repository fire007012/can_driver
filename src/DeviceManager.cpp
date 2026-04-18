#include "can_driver/DeviceManager.h"

#include <ros/ros.h>

namespace {
constexpr const char *kUdpPrefix = "udp://";
constexpr const char *kEcbPrefix = "ecb://";
}

bool DeviceManager::isUdpDevice(const std::string &device) const
{
    return device.rfind(kUdpPrefix, 0) == 0;
}

bool DeviceManager::isEcbDevice(const std::string &device) const
{
    return device.rfind(kEcbPrefix, 0) == 0;
}

std::shared_ptr<CanTransport> DeviceManager::getTransportBaseLocked(const std::string &device) const
{
    auto canIt = canTransports_.find(device);
    if (canIt != canTransports_.end()) {
        return std::static_pointer_cast<CanTransport>(canIt->second);
    }
    auto udpIt = udpTransports_.find(device);
    if (udpIt != udpTransports_.end()) {
        return std::static_pointer_cast<CanTransport>(udpIt->second);
    }
    return nullptr;
}

bool DeviceManager::shutdownTransportLocked(const std::string &device)
{
    auto canIt = canTransports_.find(device);
    if (canIt != canTransports_.end() && canIt->second) {
        canIt->second->shutdown();
        return true;
    }
    auto udpIt = udpTransports_.find(device);
    if (udpIt != udpTransports_.end() && udpIt->second) {
        udpIt->second->shutdown();
        return true;
    }
    return false;
}

bool DeviceManager::initializeTransportLocked(const std::string &device, bool loopback)
{
    if (isUdpDevice(device)) {
        auto it = udpTransports_.find(device);
        if (it == udpTransports_.end()) {
            auto transport = std::make_shared<UdpCanTransport>();
            if (!transport->initialize(device)) {
                ROS_ERROR("[CanDriverHW] Failed to initialize UDP device '%s'.", device.c_str());
                return false;
            }
            udpTransports_[device] = transport;
        } else if (!it->second->initialize(device)) {
            ROS_ERROR("[CanDriverHW] Failed to re-initialize UDP device '%s'.", device.c_str());
            return false;
        }
        return true;
    }

    auto it = canTransports_.find(device);
    if (it == canTransports_.end()) {
        auto transport = std::make_shared<SocketCanController>();
        if (!transport->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Failed to initialize CAN device '%s'.", device.c_str());
            return false;
        }
        canTransports_[device] = transport;
    } else if (!it->second->initialize(device, loopback)) {
        ROS_ERROR("[CanDriverHW] Failed to re-initialize CAN device '%s'.", device.c_str());
        return false;
    }
    return true;
}

// 幂等创建 transport：同一 device 只创建一次。
bool DeviceManager::ensureTransport(const std::string &device, bool loopback)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (isEcbDevice(device)) {
        ecbDevices_.insert(device);
        if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
            deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
        }
        return true;
    }
    if (getTransportBaseLocked(device)) {
        return true;
    }

    if (!initializeTransportLocked(device, loopback)) {
        return false;
    }
    // 同步创建该设备的命令互斥锁，供上层 write() 串行下发命令使用。
    deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    ROS_INFO("[CanDriverHW] Opened device '%s'.", device.c_str());
    return true;
}

bool DeviceManager::ensureProtocol(const std::string &device, CanType type)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (type == CanType::ECB) {
        if (ecbProtocols_.find(device) == ecbProtocols_.end()) {
            ecbProtocols_[device] = std::make_shared<InnfosEcbProtocol>(device);
        }
        ecbDevices_.insert(device);
        if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
            deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
        }
        return true;
    }

    auto transport = getTransportBaseLocked(device);
    // protocol 依赖 transport，未就绪直接失败。
    if (!transport) {
        return false;
    }

    // 按协议类型按需懒加载实例。
    if (type == CanType::MT) {
        if (mtProtocols_.find(device) == mtProtocols_.end()) {
            mtProtocols_[device] = std::make_shared<MtCan>(transport);
        }
    } else if (type == CanType::PP) {
        if (eyouProtocols_.find(device) == eyouProtocols_.end()) {
            auto eyou = std::make_shared<EyouCan>(transport);
            eyou->setFastWriteEnabled(ppFastWriteEnabled_);
            eyouProtocols_[device] = std::move(eyou);
        }
    } else {
        return false;
    }
    return true;
}

bool DeviceManager::initDevice(const std::string &device,
                               const std::vector<std::pair<CanType, MotorID>> &motors,
                               bool loopback)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);

    std::vector<MotorID> ecbIds;
    for (const auto &entry : motors) {
        if (entry.first == CanType::ECB) {
            ecbIds.push_back(entry.second);
        }
    }
    if (!ecbIds.empty() && isEcbDevice(device)) {
        if (ecbProtocols_.find(device) == ecbProtocols_.end()) {
            ecbProtocols_[device] = std::make_shared<InnfosEcbProtocol>(device);
        }
        if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
            deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
        }
        ecbDevices_.insert(device);
        ecbProtocols_[device]->initializeMotorRefresh(ecbIds);
        ROS_INFO("[CanDriverHW] Initialized ECB device '%s'.", device.c_str());
        return true;
    }

    // 已存在 transport 时先 shutdown 再 re-init，确保监听器和内部状态被重置。
    if (!getTransportBaseLocked(device)) {
        if (!initializeTransportLocked(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Failed to init '%s'.", device.c_str());
            return false;
        }
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    } else {
        shutdownTransportLocked(device);
        if (!initializeTransportLocked(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Re-init of '%s' failed.", device.c_str());
            return false;
        }
    }

    auto transport = getTransportBaseLocked(device);
    if (!transport) {
        ROS_ERROR("[CanDriverHW] Transport unavailable for '%s'.", device.c_str());
        return false;
    }

    if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    }

    // 按协议拆分电机列表，避免不必要地创建协议实例。
    std::vector<MotorID> mtIds;
    std::vector<MotorID> ppIds;
    for (const auto &entry : motors) {
        if (entry.first == CanType::MT) {
            mtIds.push_back(entry.second);
        } else if (entry.first == CanType::PP) {
            ppIds.push_back(entry.second);
        }
    }
    // 初始化协议对象并启动状态刷新任务。
    if (!mtIds.empty() && mtProtocols_.find(device) == mtProtocols_.end()) {
        mtProtocols_[device] = std::make_shared<MtCan>(transport);
    }
    if (!ppIds.empty() && eyouProtocols_.find(device) == eyouProtocols_.end()) {
        auto eyou = std::make_shared<EyouCan>(transport);
        eyou->setFastWriteEnabled(ppFastWriteEnabled_);
        eyouProtocols_[device] = std::move(eyou);
    }
    if (!mtIds.empty()) {
        mtProtocols_[device]->initializeMotorRefresh(mtIds);
    }
    if (!ppIds.empty()) {
        eyouProtocols_[device]->initializeMotorRefresh(ppIds);
    }

    ROS_INFO("[CanDriverHW] Initialized '%s'.", device.c_str());
    return true;
}

void DeviceManager::startRefresh(const std::string &device,
                                 CanType type,
                                 const std::vector<MotorID> &ids)
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    // 空列表表示无需刷新，直接返回。
    if (ids.empty()) {
        return;
    }
    if (type == CanType::MT) {
        auto it = mtProtocols_.find(device);
        if (it != mtProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
        }
    } else if (type == CanType::PP) {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
        }
    } else if (type == CanType::ECB) {
        auto it = ecbProtocols_.find(device);
        if (it != ecbProtocols_.end()) {
            it->second->initializeMotorRefresh(ids);
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
    for (auto &kv : ecbProtocols_) {
        if (kv.second) {
            kv.second->setRefreshRateHz(hz);
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
    // 先释放协议（包含内部线程/handler），再关闭 transport。
    mtProtocols_.clear();
    eyouProtocols_.clear();
    ecbProtocols_.clear();

    for (auto &kv : canTransports_) {
        kv.second->shutdown();
    }
    for (auto &kv : udpTransports_) {
        kv.second->shutdown();
    }

    canTransports_.clear();
    udpTransports_.clear();
    ecbDevices_.clear();
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
    } else if (type == CanType::PP) {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    } else if (type == CanType::ECB) {
        auto it = ecbProtocols_.find(device);
        if (it != ecbProtocols_.end()) {
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
    auto it = canTransports_.find(device);
    return (it != canTransports_.end()) ? it->second : nullptr;
}

bool DeviceManager::isDeviceReady(const std::string &device) const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    if (isEcbDevice(device)) {
        return ecbProtocols_.find(device) != ecbProtocols_.end();
    }
    auto canIt = canTransports_.find(device);
    if (canIt != canTransports_.end() && canIt->second) {
        return canIt->second->isReady();
    }

    auto udpIt = udpTransports_.find(device);
    if (udpIt != udpTransports_.end() && udpIt->second) {
        return udpIt->second->isReady();
    }
    return false;
}

std::size_t DeviceManager::deviceCount() const
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return canTransports_.size() + udpTransports_.size() + ecbDevices_.size();
}
