#ifndef CAN_DRIVER_SOCKETCAN_CONTROLLER_H
#define CAN_DRIVER_SOCKETCAN_CONTROLLER_H

#include "can_driver/CanTransport.h"

#include <linux/can.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

/**
 * @brief Transport implementation backed by Linux SocketCAN (PF_CAN/CAN_RAW).
 */
class SocketCanController : public CanTransport {
    friend class SocketCanControllerTestAccessor;

public:
    SocketCanController();
    ~SocketCanController() override;

    /**
     * @brief Initialize the CAN interface.
     * @param device SocketCAN device name (e.g. "can0").
     * @param loopback Enable CAN loopback frames.
     */
    bool initialize(const std::string &device, bool loopback = false);

    void shutdown();

    void send(const CanTransport::Frame &frame) override;

    std::size_t addReceiveHandler(ReceiveHandler handler) override;
    void removeReceiveHandler(std::size_t handlerId) override;

    bool isReady() const;
    std::string device() const;

private:
    void receiveLoop();
    void dispatchReceive(const CanTransport::Frame &frame);
    struct can_frame toLinuxCanFrame(const CanTransport::Frame &frame) const;
    CanTransport::Frame fromLinuxCanFrame(const struct can_frame &frame) const;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> stopRequested_{false};
    std::atomic<std::size_t> nextHandlerId_{1};
    std::unordered_map<std::size_t, ReceiveHandler> handlers_;
    mutable std::mutex handlerMutex_;
    std::thread receiveThread_;
    int socketFd_{-1};
    std::string deviceName_;
};

#endif // CAN_DRIVER_SOCKETCAN_CONTROLLER_H
