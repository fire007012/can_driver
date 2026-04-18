#ifndef CAN_DRIVER_UDP_CAN_TRANSPORT_H
#define CAN_DRIVER_UDP_CAN_TRANSPORT_H

#include "can_driver/CanTransport.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <thread>
#include <unordered_map>

/**
 * @brief 基于 UDP 的“CAN 帧封装”传输层。
 *
 * 设备字符串格式：
 *   - udp://<local_port>@<remote_ip>:<remote_port>
 *   - udp://<local_ip>:<local_port>@<remote_ip>:<remote_port>
 *
 * 例：udp://7101@192.168.1.253:1031
 */
class UdpCanTransport : public CanTransport {
public:
    UdpCanTransport();
    ~UdpCanTransport() override;

    bool initialize(const std::string &deviceSpec);
    void shutdown();

    void send(const CanTransport::Frame &frame) override;

    std::size_t addReceiveHandler(ReceiveHandler handler) override;
    void removeReceiveHandler(std::size_t handlerId) override;

    bool isReady() const;
    std::string device() const;

private:
    struct EndpointConfig {
        std::string localIp;
        std::uint16_t localPort{0};
        std::string remoteIp;
        std::uint16_t remotePort{0};
    };

    bool parseDeviceSpec(const std::string &deviceSpec, EndpointConfig &cfg, std::string &errMsg) const;
    void receiveLoop();
    void dispatchReceive(const CanTransport::Frame &frame);

    std::atomic<bool> initialized_{false};
    std::atomic<bool> stopRequested_{false};
    std::atomic<std::size_t> nextHandlerId_{1};
    std::unordered_map<std::size_t, ReceiveHandler> handlers_;
    mutable std::mutex handlerMutex_;

    std::thread receiveThread_;
    int socketFd_{-1};

    sockaddr_in remoteAddr_{};
    std::string deviceSpec_;
};

#endif // CAN_DRIVER_UDP_CAN_TRANSPORT_H
