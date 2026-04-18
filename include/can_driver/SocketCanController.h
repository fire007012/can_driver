#ifndef CAN_DRIVER_SOCKETCAN_CONTROLLER_H
#define CAN_DRIVER_SOCKETCAN_CONTROLLER_H

#include "can_driver/CanTransport.h"

#include <linux/can.h>

#include <atomic>
#include <cstdint>
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
    friend class DeviceManagerTestAccessor;

public:
    struct Stats {
        std::uint64_t txOk{0};
        std::uint64_t txBackpressure{0};
        std::uint64_t txLinkUnavailable{0};
        std::uint64_t txError{0};
        std::uint64_t txPartial{0};
        std::uint64_t rxOk{0};
        std::uint64_t rxError{0};
        std::uint64_t rxShortRead{0};
        std::int64_t lastTxLinkUnavailableSteadyNs{0};
        std::int64_t lastRxSteadyNs{0};
    };

    SocketCanController();
    ~SocketCanController() override;

    /**
     * @brief Initialize the CAN interface.
     * @param device SocketCAN device name (e.g. "can0").
     * @param loopback Whether this socket should receive its own transmitted
     * frames. Kernel local loopback remains enabled so local sniffers such as
     * candump can still observe outgoing traffic.
     */
    bool initialize(const std::string &device, bool loopback = false);

    void shutdown();

    SendResult send(const CanTransport::Frame &frame) override;

    std::size_t addReceiveHandler(ReceiveHandler handler) override;
    void removeReceiveHandler(std::size_t handlerId) override;

    bool isReady() const;
    std::string device() const;
    Stats snapshotStats() const;

private:
    void receiveLoop();
    void dispatchReceive(const CanTransport::Frame &frame);
    struct can_frame toLinuxCanFrame(const CanTransport::Frame &frame) const;
    CanTransport::Frame fromLinuxCanFrame(const struct can_frame &frame) const;
    static bool shouldEnableLocalLoopback();
    static bool shouldReceiveOwnMessages(bool loopback);
    static bool isBackpressureSendError(int errorCode);
    static bool isLinkUnavailableSendError(int errorCode);
    void resetStats();

    std::atomic<bool> initialized_{false};
    std::atomic<bool> stopRequested_{false};
    std::atomic<std::size_t> nextHandlerId_{1};
    std::atomic<std::uint64_t> txOkCount_{0};
    std::atomic<std::uint64_t> txBackpressureCount_{0};
    std::atomic<std::uint64_t> txLinkUnavailableCount_{0};
    std::atomic<std::uint64_t> txErrorCount_{0};
    std::atomic<std::uint64_t> txPartialCount_{0};
    std::atomic<std::uint64_t> rxOkCount_{0};
    std::atomic<std::uint64_t> rxErrorCount_{0};
    std::atomic<std::uint64_t> rxShortReadCount_{0};
    std::atomic<std::int64_t> lastTxLinkUnavailableSteadyNs_{0};
    std::atomic<std::int64_t> lastRxSteadyNs_{0};
    std::unordered_map<std::size_t, ReceiveHandler> handlers_;
    mutable std::mutex handlerMutex_;
    std::thread receiveThread_;
    int socketFd_{-1};
    std::string deviceName_;
};

#endif // CAN_DRIVER_SOCKETCAN_CONTROLLER_H
