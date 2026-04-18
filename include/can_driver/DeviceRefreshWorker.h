#ifndef CAN_DRIVER_DEVICE_REFRESH_WORKER_H
#define CAN_DRIVER_DEVICE_REFRESH_WORKER_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace can_driver {

class DeviceRefreshWorker {
public:
    using TickFn = std::function<void()>;
    using SleepFn = std::function<std::chrono::milliseconds()>;

    DeviceRefreshWorker(TickFn tick, SleepFn sleepFor)
        : tick_(std::move(tick))
        , sleepFor_(std::move(sleepFor))
    {
    }

    ~DeviceRefreshWorker()
    {
        stop();
    }

    void start()
    {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true)) {
            notify();
            return;
        }

        workerThread_ = std::thread([this]() { workerLoop(); });
    }

    void stop()
    {
        const bool wasRunning = running_.exchange(false);
        notify();

        if (!wasRunning && !workerThread_.joinable()) {
            return;
        }
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
    }

    void notify()
    {
        {
            std::lock_guard<std::mutex> lock(notifyMutex_);
            notified_ = true;
        }
        notifyCv_.notify_all();
    }

    bool running() const
    {
        return running_.load(std::memory_order_acquire);
    }

private:
    void workerLoop()
    {
        while (running_.load(std::memory_order_acquire)) {
            if (tick_) {
                tick_();
            }
            if (!running_.load(std::memory_order_acquire)) {
                break;
            }

            auto sleepFor = sleepFor_ ? sleepFor_() : std::chrono::milliseconds(5);
            if (sleepFor < std::chrono::milliseconds(1)) {
                sleepFor = std::chrono::milliseconds(1);
            }

            std::unique_lock<std::mutex> lock(notifyMutex_);
            notifyCv_.wait_for(lock, sleepFor, [this]() {
                return !running_.load(std::memory_order_acquire) || notified_;
            });
            notified_ = false;
        }
    }

    TickFn tick_;
    SleepFn sleepFor_;
    std::atomic<bool> running_{false};
    std::thread workerThread_;
    mutable std::mutex notifyMutex_;
    std::condition_variable notifyCv_;
    bool notified_{false};
};

} // namespace can_driver

#endif // CAN_DRIVER_DEVICE_REFRESH_WORKER_H
