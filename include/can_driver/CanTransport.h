#ifndef CAN_DRIVER_CAN_TRANSPORT_H
#define CAN_DRIVER_CAN_TRANSPORT_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

/**
 * @brief Lightweight transport abstraction to decouple protocol logic from the underlying bus.
 */
class CanTransport {
public:
    struct Frame {
        std::uint32_t id = 0;                      ///< CAN identifier (standard or extended)
        bool isExtended = false;                   ///< Whether the identifier uses 29 bits
        bool isRemoteRequest = false;              ///< RTR bit
        std::uint8_t dlc = 0;                      ///< Number of valid data bytes (0-8)
        std::array<std::uint8_t, 8> data { {0} };  ///< Payload bytes
    };

    /**
     * @brief Result of a send() call.
     *
     * - Ok:            Frame was successfully handed to the kernel.
     * - Backpressure:  Transient failure (EAGAIN / ENOBUFS); caller may retry.
     * - LinkDown:      Interface is down (ENETDOWN / ENODEV); retrying is futile
     *                  until the link recovers.
     * - Error:         Any other send error.
     */
    enum class SendResult : std::uint8_t {
        Ok,
        Backpressure,
        LinkDown,
        Error,
    };

    using ReceiveHandler = std::function<void(const Frame &)>;

    virtual ~CanTransport() = default;

    virtual SendResult send(const Frame &frame) = 0;

    virtual std::size_t addReceiveHandler(ReceiveHandler handler) = 0;
    virtual void removeReceiveHandler(std::size_t handlerId) = 0;
};

#endif // CAN_DRIVER_CAN_TRANSPORT_H
