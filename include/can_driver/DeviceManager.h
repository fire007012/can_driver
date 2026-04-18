#ifndef CAN_DRIVER_DEVICE_MANAGER_H
#define CAN_DRIVER_DEVICE_MANAGER_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanType.h"
#include "can_driver/EyouCan.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/InnfosEcbProtocol.h"
#include "can_driver/MotorID.h"
#include "can_driver/MtCan.h"
#include "can_driver/SocketCanController.h"
#include "can_driver/UdpCanTransport.h"

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <set>
#include <string>
#include <vector>

/**
 * @brief 管理 CAN 设备、传输层和协议实例的集中入口。
 *
 * 一个 device（如 can0/vcan0）只会持有一个传输层实例，
 * 但可以按需同时挂载 MT/PP 两套协议对象。
 */
class DeviceManager : public IDeviceManager {
public:
    /// 确保底层传输已建立。重复调用是幂等的。
    bool ensureTransport(const std::string &device, bool loopback = false) override;
    /// 在指定设备上确保目标协议实例存在（要求 transport 已创建）。
    bool ensureProtocol(const std::string &device, CanType type) override;
    /// 按设备完成 transport/protocol 初始化，并触发首轮状态刷新。
    bool initDevice(const std::string &device,
                    const std::vector<std::pair<CanType, MotorID>> &motors,
                    bool loopback = false) override;
    /// 为协议注册需要周期刷新状态的电机列表。
    void startRefresh(const std::string &device,
                      CanType type,
                      const std::vector<MotorID> &ids) override;
    /// 设置所有协议实例的状态轮询频率（Hz）；<=0 恢复协议默认策略。
    void setRefreshRateHz(double hz) override;
    /// 设置 PP 协议是否使用快写命令（CMD=0x05）。
    void setPpFastWriteEnabled(bool enabled) override;
    /// 停止并释放所有设备资源。
    void shutdownAll() override;

    /// 读取协议实例（不存在返回 nullptr）。
    std::shared_ptr<CanProtocol> getProtocol(const std::string &device, CanType type) const override;
    /// 返回该设备命令互斥锁（不存在返回 nullptr）。
    std::shared_ptr<std::mutex> getDeviceMutex(const std::string &device) const override;
    /// 返回设备是否 ready。
    bool isDeviceReady(const std::string &device) const override;
    /// 返回传输层实例（不存在返回 nullptr）。
    std::shared_ptr<SocketCanController> getTransport(const std::string &device) const;
    /// 当前已初始化的 transport 数量。
    std::size_t deviceCount() const override;

private:
    bool isUdpDevice(const std::string &device) const;
    bool isEcbDevice(const std::string &device) const;
    std::shared_ptr<CanTransport> getTransportBaseLocked(const std::string &device) const;
    bool shutdownTransportLocked(const std::string &device);
    bool initializeTransportLocked(const std::string &device, bool loopback);

    // 读多写少：读取协议/transport 时使用 shared_lock，创建/销毁时 unique_lock。
    mutable std::shared_mutex mutex_;
    // key = can device name（例如 can0/vcan0）。
    std::map<std::string, std::shared_ptr<SocketCanController>> canTransports_;
    std::map<std::string, std::shared_ptr<UdpCanTransport>> udpTransports_;
    std::map<std::string, std::shared_ptr<MtCan>> mtProtocols_;
    std::map<std::string, std::shared_ptr<EyouCan>> eyouProtocols_;
    std::map<std::string, std::shared_ptr<InnfosEcbProtocol>> ecbProtocols_;
    std::set<std::string> ecbDevices_;
    bool ppFastWriteEnabled_{false};
    // 每个设备一把命令互斥锁，避免多个控制线程并发下发命令时互相打断。
    std::map<std::string, std::shared_ptr<std::mutex>> deviceCmdMutexes_;
};

#endif // CAN_DRIVER_DEVICE_MANAGER_H
