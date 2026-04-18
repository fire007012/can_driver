#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/UdpCanTransport.h"

#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace {

class RosTimeFixture : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }
};

int bindLoopbackUdpSocket(std::uint16_t port)
{
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return -1;
    }

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return -1;
    }

    return fd;
}

std::uint16_t socketPort(int fd)
{
    sockaddr_in addr {};
    socklen_t len = sizeof(addr);
    if (::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len) != 0) {
        return 0;
    }
    return ntohs(addr.sin_port);
}

std::uint16_t reserveEphemeralLoopbackPort()
{
    const int fd = bindLoopbackUdpSocket(0);
    if (fd < 0) {
        return 0;
    }
    const std::uint16_t port = socketPort(fd);
    ::close(fd);
    return port;
}

std::string makeDeviceSpec(std::uint16_t localPort, std::uint16_t remotePort)
{
    return "udp://" + std::to_string(localPort) + "@127.0.0.1:" + std::to_string(remotePort);
}

bool recvPacketWithTimeout(int fd,
                           std::array<std::uint8_t, 13> *packet,
                           std::chrono::milliseconds timeout)
{
    fd_set readFds;
    FD_ZERO(&readFds);
    FD_SET(fd, &readFds);

    timeval tv {};
    tv.tv_sec = static_cast<long>(timeout.count() / 1000);
    tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);
    const int selected = ::select(fd + 1, &readFds, nullptr, nullptr, &tv);
    if (selected <= 0 || !FD_ISSET(fd, &readFds)) {
        return false;
    }

    const auto received =
        ::recvfrom(fd, packet->data(), packet->size(), 0, nullptr, nullptr);
    return received == static_cast<ssize_t>(packet->size());
}

void sendPacketToLoopback(int fd,
                          std::uint16_t destinationPort,
                          const std::array<std::uint8_t, 13> &packet)
{
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(destinationPort);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    ASSERT_EQ(::sendto(fd,
                       packet.data(),
                       packet.size(),
                       0,
                       reinterpret_cast<const sockaddr *>(&addr),
                       sizeof(addr)),
              static_cast<ssize_t>(packet.size()));
}

TEST_F(RosTimeFixture, SendPreservesFrameDlcInUdpWrapper)
{
    const int receiverFd = bindLoopbackUdpSocket(0);
    ASSERT_GE(receiverFd, 0);
    const std::uint16_t remotePort = socketPort(receiverFd);
    ASSERT_NE(remotePort, 0);

    const std::uint16_t localPort = reserveEphemeralLoopbackPort();
    ASSERT_NE(localPort, 0);

    UdpCanTransport transport;
    ASSERT_TRUE(transport.initialize(makeDeviceSpec(localPort, remotePort)));

    CanTransport::Frame frame {};
    frame.id = 0x123;
    frame.dlc = 3;
    frame.data[0] = 0xAA;
    frame.data[1] = 0xBB;
    frame.data[2] = 0xCC;

    EXPECT_EQ(transport.send(frame), CanTransport::SendResult::Ok);

    std::array<std::uint8_t, 13> packet {};
    ASSERT_TRUE(recvPacketWithTimeout(receiverFd, &packet, std::chrono::milliseconds(200)));
    EXPECT_EQ(packet[0], 0x03);
    EXPECT_EQ(packet[3], 0x01);
    EXPECT_EQ(packet[4], 0x23);
    EXPECT_EQ(packet[5], 0xAA);
    EXPECT_EQ(packet[6], 0xBB);
    EXPECT_EQ(packet[7], 0xCC);
    EXPECT_EQ(packet[8], 0x00);

    transport.shutdown();
    ::close(receiverFd);
}

TEST_F(RosTimeFixture, ReceiveUsesPacketDlcAndIgnoresUnexpectedSender)
{
    const int expectedSenderFd = bindLoopbackUdpSocket(0);
    ASSERT_GE(expectedSenderFd, 0);
    const std::uint16_t expectedSenderPort = socketPort(expectedSenderFd);
    ASSERT_NE(expectedSenderPort, 0);

    const int wrongSenderFd = bindLoopbackUdpSocket(0);
    ASSERT_GE(wrongSenderFd, 0);

    const std::uint16_t localPort = reserveEphemeralLoopbackPort();
    ASSERT_NE(localPort, 0);

    UdpCanTransport transport;
    ASSERT_TRUE(transport.initialize(makeDeviceSpec(localPort, expectedSenderPort)));

    std::mutex mutex;
    std::condition_variable cv;
    bool received = false;
    CanTransport::Frame captured;
    const auto handlerId = transport.addReceiveHandler(
        [&](const CanTransport::Frame &frame) {
            std::lock_guard<std::mutex> lock(mutex);
            captured = frame;
            received = true;
            cv.notify_all();
        });
    ASSERT_NE(handlerId, 0u);

    std::array<std::uint8_t, 13> packet {};
    packet[0] = 0x03;
    packet[3] = 0x04;
    packet[4] = 0x56;
    packet[5] = 0x11;
    packet[6] = 0x22;
    packet[7] = 0x33;

    sendPacketToLoopback(wrongSenderFd, localPort, packet);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    {
        std::lock_guard<std::mutex> lock(mutex);
        EXPECT_FALSE(received);
    }

    sendPacketToLoopback(expectedSenderFd, localPort, packet);
    {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(cv.wait_for(
            lock, std::chrono::milliseconds(200), [&]() { return received; }));
    }

    EXPECT_EQ(captured.id, 0x0456u);
    EXPECT_EQ(captured.dlc, 0x03);
    EXPECT_EQ(captured.data[0], 0x11);
    EXPECT_EQ(captured.data[1], 0x22);
    EXPECT_EQ(captured.data[2], 0x33);
    EXPECT_EQ(captured.data[3], 0x00);

    transport.shutdown();
    ::close(wrongSenderFd);
    ::close(expectedSenderFd);
}

} // namespace
