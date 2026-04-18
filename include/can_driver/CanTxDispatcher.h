#ifndef CAN_DRIVER_CAN_TX_DISPATCHER_H
#define CAN_DRIVER_CAN_TX_DISPATCHER_H

#include "can_driver/CanTransport.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

class CanTxDispatcher {
public:
    enum class Category : std::uint8_t {
        Control,
        Query,
        Recover,
        Config,
    };

    struct Request {
        using Completion = std::function<void(bool attemptedSend,
                                              CanTransport::SendResult sendResult,
                                              std::chrono::steady_clock::time_point eventTime)>;
        CanTransport::Frame frame;
        Category category{Category::Control};
        const char *source{nullptr};
        Completion completion;
    };

    virtual ~CanTxDispatcher() = default;

    virtual void submit(const Request &request) = 0;
};

#endif // CAN_DRIVER_CAN_TX_DISPATCHER_H
