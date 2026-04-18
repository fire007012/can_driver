#include "can_driver/UdpCanTransport.h"

#include <ros/console.h>

#include <arpa/inet.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <string>
#include <vector>

namespace {

bool isAllDigits(const std::string &s)
{
    if (s.empty()) {
        return false;
    }
    return std::all_of(s.begin(), s.end(), [](unsigned char ch) { return std::isdigit(ch) != 0; });
}

bool parsePort(const std::string &s, std::uint16_t &port)
{
    if (!isAllDigits(s)) {
        return false;
    }
    const auto raw = std::strtoul(s.c_str(), nullptr, 10);
    if (raw == 0 || raw > 65535UL) {
        return false;
    }
    port = static_cast<std::uint16_t>(raw);
    return true;
}

} // namespace

UdpCanTransport::UdpCanTransport()
{
}

UdpCanTransport::~UdpCanTransport()
{
    shutdown();
}

bool UdpCanTransport::parseDeviceSpec(const std::string &deviceSpec,
                                      EndpointConfig &cfg,
                                      std::string &errMsg) const
{
    constexpr const char *kPrefix = "udp://";
    if (deviceSpec.rfind(kPrefix, 0) != 0) {
        errMsg = "device spec must start with udp://";
        return false;
    }

    const std::string body = deviceSpec.substr(std::strlen(kPrefix));
    const std::size_t atPos = body.find('@');
    if (atPos == std::string::npos) {
        errMsg = "missing '@' in udp device spec";
        return false;
    }

    const std::string localPart = body.substr(0, atPos);
    const std::string remotePart = body.substr(atPos + 1);

    if (localPart.empty() || remotePart.empty()) {
        errMsg = "invalid local/remote endpoint in udp device spec";
        return false;
    }

    cfg.localIp = "0.0.0.0";
    std::string localPortStr;
    const std::size_t localColon = localPart.rfind(':');
    if (localColon == std::string::npos) {
        localPortStr = localPart;
    } else {
        cfg.localIp = localPart.substr(0, localColon);
        localPortStr = localPart.substr(localColon + 1);
        if (cfg.localIp.empty()) {
            cfg.localIp = "0.0.0.0";
        }
    }

    const std::size_t remoteColon = remotePart.rfind(':');
    if (remoteColon == std::string::npos) {
        errMsg = "remote endpoint must be <ip>:<port>";
        return false;
    }
    cfg.remoteIp = remotePart.substr(0, remoteColon);
    const std::string remotePortStr = remotePart.substr(remoteColon + 1);

    if (cfg.remoteIp.empty()) {
        errMsg = "remote ip is empty";
        return false;
    }

    if (!parsePort(localPortStr, cfg.localPort)) {
        errMsg = "invalid local port";
        return false;
    }
    if (!parsePort(remotePortStr, cfg.remotePort)) {
        errMsg = "invalid remote port";
        return false;
    }

    return true;
}

bool UdpCanTransport::initialize(const std::string &deviceSpec)
{
    shutdown();

    EndpointConfig cfg;
    std::string errMsg;
    if (!parseDeviceSpec(deviceSpec, cfg, errMsg)) {
        ROS_ERROR_STREAM("[UdpCanTransport] Invalid device spec '" << deviceSpec << "': " << errMsg);
        return false;
    }

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        ROS_ERROR_STREAM("[UdpCanTransport] Failed to create UDP socket for '" << deviceSpec
                         << "', errno=" << errno);
        return false;
    }

    sockaddr_in localAddr {};
    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(cfg.localPort);
    if (::inet_pton(AF_INET, cfg.localIp.c_str(), &localAddr.sin_addr) != 1) {
        ROS_ERROR_STREAM("[UdpCanTransport] Invalid local IP '" << cfg.localIp << "'.");
        ::close(fd);
        return false;
    }

    if (::bind(fd, reinterpret_cast<sockaddr *>(&localAddr), sizeof(localAddr)) < 0) {
        ROS_ERROR_STREAM("[UdpCanTransport] Failed to bind local endpoint " << cfg.localIp << ":"
                         << cfg.localPort << ", errno=" << errno);
        ::close(fd);
        return false;
    }

    sockaddr_in remoteAddr {};
    remoteAddr.sin_family = AF_INET;
    remoteAddr.sin_port = htons(cfg.remotePort);
    if (::inet_pton(AF_INET, cfg.remoteIp.c_str(), &remoteAddr.sin_addr) != 1) {
        ROS_ERROR_STREAM("[UdpCanTransport] Invalid remote IP '" << cfg.remoteIp << "'.");
        ::close(fd);
        return false;
    }

    socketFd_ = fd;
    remoteAddr_ = remoteAddr;
    deviceSpec_ = deviceSpec;

    stopRequested_.store(false);
    initialized_.store(true);
    receiveThread_ = std::thread(&UdpCanTransport::receiveLoop, this);

    ROS_INFO_STREAM("[UdpCanTransport] Initialized " << deviceSpec_);
    return true;
}

void UdpCanTransport::shutdown()
{
    stopRequested_.store(true);
    initialized_.store(false);

    if (receiveThread_.joinable()) {
        receiveThread_.join();
    }

    if (socketFd_ >= 0) {
        ::close(socketFd_);
        socketFd_ = -1;
    }

    {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        handlers_.clear();
        nextHandlerId_.store(1);
    }

    deviceSpec_.clear();
    std::memset(&remoteAddr_, 0, sizeof(remoteAddr_));
}

CanTransport::SendResult UdpCanTransport::send(const CanTransport::Frame &frame)
{
    if (!initialized_.load() || socketFd_ < 0) {
        return CanTransport::SendResult::LinkDown;
    }

    std::array<std::uint8_t, 13> packet{};
    packet.fill(0);

    packet[0] = 0x08;
    packet[3] = static_cast<std::uint8_t>((frame.id >> 8) & 0xFF);
    packet[4] = static_cast<std::uint8_t>(frame.id & 0xFF);

    const std::uint8_t dlc = std::min<std::uint8_t>(frame.dlc, 8);
    for (std::size_t i = 0; i < dlc; ++i) {
        packet[5 + i] = frame.data[i];
    }

    const auto sent = ::sendto(socketFd_,
                               packet.data(),
                               packet.size(),
                               0,
                               reinterpret_cast<const sockaddr *>(&remoteAddr_),
                               sizeof(remoteAddr_));
    if (sent == static_cast<ssize_t>(packet.size())) {
        return CanTransport::SendResult::Ok;
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS) {
        return CanTransport::SendResult::Backpressure;
    }
    if (errno == ENETDOWN || errno == ENODEV || errno == ENETUNREACH) {
        return CanTransport::SendResult::LinkDown;
    }

    if (sent != static_cast<ssize_t>(packet.size())) {
        ROS_WARN_STREAM("[UdpCanTransport] sendto failed on '" << deviceSpec_ << "', errno=" << errno);
    }
    return CanTransport::SendResult::Error;
}

std::size_t UdpCanTransport::addReceiveHandler(ReceiveHandler handler)
{
    if (!handler) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(handlerMutex_);
    const std::size_t id = nextHandlerId_++;
    handlers_.emplace(id, std::move(handler));
    return id;
}

void UdpCanTransport::removeReceiveHandler(std::size_t handlerId)
{
    if (handlerId == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(handlerMutex_);
    handlers_.erase(handlerId);
}

bool UdpCanTransport::isReady() const
{
    return initialized_.load();
}

std::string UdpCanTransport::device() const
{
    return deviceSpec_;
}

void UdpCanTransport::receiveLoop()
{
    while (!stopRequested_.load()) {
        if (socketFd_ < 0) {
            break;
        }

        fd_set readFds;
        FD_ZERO(&readFds);
        FD_SET(socketFd_, &readFds);

        timeval tv {};
        tv.tv_sec = 0;
        tv.tv_usec = 200000;

        const int sel = ::select(socketFd_ + 1, &readFds, nullptr, nullptr, &tv);
        if (sel < 0) {
            if (errno == EINTR) {
                continue;
            }
            ROS_WARN_STREAM("[UdpCanTransport] select() failed on '" << deviceSpec_ << "', errno=" << errno);
            continue;
        }
        if (sel == 0 || !FD_ISSET(socketFd_, &readFds)) {
            continue;
        }

        std::array<std::uint8_t, 64> rx{};
        const auto n = ::recvfrom(socketFd_, rx.data(), rx.size(), 0, nullptr, nullptr);
        if (n < 13) {
            continue;
        }

        CanTransport::Frame frame;
        frame.isExtended = false;
        frame.isRemoteRequest = false;
        frame.id = (static_cast<std::uint32_t>(rx[3]) << 8) | static_cast<std::uint32_t>(rx[4]);
        frame.dlc = 8;
        frame.data.fill(0);
        for (std::size_t i = 0; i < 8; ++i) {
            frame.data[i] = rx[5 + i];
        }

        dispatchReceive(frame);
    }
}

void UdpCanTransport::dispatchReceive(const CanTransport::Frame &frame)
{
    std::vector<ReceiveHandler> handlersCopy;
    {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        handlersCopy.reserve(handlers_.size());
        for (const auto &entry : handlers_) {
            handlersCopy.push_back(entry.second);
        }
    }

    for (auto &handler : handlersCopy) {
        if (handler) {
            handler(frame);
        }
    }
}
