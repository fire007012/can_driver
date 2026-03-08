#ifndef CAN_DRIVER_IDEVICEMANAGER_H
#define CAN_DRIVER_IDEVICEMANAGER_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanType.h"
#include "can_driver/MotorID.h"

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

class IDeviceManager {
public:
    virtual ~IDeviceManager() = default;

    virtual bool ensureTransport(const std::string &device, bool loopback = false) = 0;
    virtual bool ensureProtocol(const std::string &device, CanType type) = 0;
    virtual bool initDevice(const std::string &device,
                            const std::vector<std::pair<CanType, MotorID>> &motors,
                            bool loopback = false) = 0;
    virtual void startRefresh(const std::string &device,
                              CanType type,
                              const std::vector<MotorID> &ids) = 0;
    virtual void setRefreshRateHz(double hz) = 0;
    virtual void shutdownAll() = 0;

    virtual std::shared_ptr<CanProtocol> getProtocol(const std::string &device, CanType type) const = 0;
    virtual std::shared_ptr<std::mutex> getDeviceMutex(const std::string &device) const = 0;
    virtual bool isDeviceReady(const std::string &device) const = 0;
    virtual std::size_t deviceCount() const = 0;
};

#endif // CAN_DRIVER_IDEVICEMANAGER_H
