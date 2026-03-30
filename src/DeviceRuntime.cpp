#include "can_driver/DeviceRuntime.h"

#include <ros/ros.h>

#include <utility>

DeviceRuntime::DeviceRuntime(std::shared_ptr<CanTransport> transport,
                             std::string deviceName)
    : DeviceRuntime(std::move(transport), std::move(deviceName), Options {})
{
}

DeviceRuntime::DeviceRuntime(std::shared_ptr<CanTransport> transport,
                             std::string deviceName,
                             Options options)
    : transport_(std::move(transport))
    , deviceName_(std::move(deviceName))
    , options_(options)
{
    if (options_.autostart) {
        start();
    }
}

DeviceRuntime::~DeviceRuntime()
{
    shutdown();
}

void DeviceRuntime::submit(const Request &request)
{
    bool accepted = false;
    std::uint64_t droppedQuery = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopRequested_) {
            switch (request.category) {
            case Category::Control:
                droppedControlCount_.fetch_add(1, std::memory_order_relaxed);
                break;
            case Category::Recover:
                droppedRecoverCount_.fetch_add(1, std::memory_order_relaxed);
                break;
            case Category::Config:
                droppedConfigCount_.fetch_add(1, std::memory_order_relaxed);
                break;
            case Category::Query:
                droppedQueryCount_.fetch_add(1, std::memory_order_relaxed);
                break;
            }
        } else {
            Queue &queue = queueFor(request.category);
            if (queue.size() >= maxDepthFor(request.category)) {
                switch (request.category) {
                case Category::Control:
                    droppedControlCount_.fetch_add(1, std::memory_order_relaxed);
                    break;
                case Category::Recover:
                    droppedRecoverCount_.fetch_add(1, std::memory_order_relaxed);
                    break;
                case Category::Config:
                    droppedConfigCount_.fetch_add(1, std::memory_order_relaxed);
                    break;
                case Category::Query:
                    droppedQueryCount_.fetch_add(1, std::memory_order_relaxed);
                    break;
                }
            } else {
                queue.push_back(request);
                submittedCount_.fetch_add(1, std::memory_order_relaxed);
                accepted = true;
            }
        }
        droppedQuery = droppedQueryCount_.load(std::memory_order_relaxed);
    }

    if (!accepted) {
        ROS_WARN_STREAM_THROTTLE(
            1.0,
            "[DeviceRuntime] Dropping " << categoryName(request.category)
            << " frame on " << deviceName_
            << " because queue is full or runtime is stopping"
            << " stats={submitted=" << submittedCount_.load(std::memory_order_relaxed)
            << ", sent=" << sentCount_.load(std::memory_order_relaxed)
            << ", dropped_query=" << droppedQuery
            << ", pending=" << snapshotStats().pendingTotal << "}");
        return;
    }

    cv_.notify_one();
}

void DeviceRuntime::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_) {
        return;
    }

    stopRequested_ = false;
    started_ = true;
    workerThread_ = std::thread(&DeviceRuntime::workerLoop, this);
}

void DeviceRuntime::shutdown()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_) {
            stopRequested_ = true;
        } else {
            stopRequested_ = true;
        }
    }
    cv_.notify_all();
    idleCv_.notify_all();
    if (workerThread_.joinable()) {
        workerThread_.join();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        started_ = false;
        sending_ = false;
    }
}

bool DeviceRuntime::waitUntilIdleFor(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    return idleCv_.wait_for(lock, timeout, [this]() {
        return allQueuesEmptyLocked() && !sending_;
    });
}

DeviceRuntime::Stats DeviceRuntime::snapshotStats() const
{
    Stats stats;
    stats.submitted = submittedCount_.load(std::memory_order_relaxed);
    stats.sent = sentCount_.load(std::memory_order_relaxed);
    stats.droppedControl = droppedControlCount_.load(std::memory_order_relaxed);
    stats.droppedRecover = droppedRecoverCount_.load(std::memory_order_relaxed);
    stats.droppedConfig = droppedConfigCount_.load(std::memory_order_relaxed);
    stats.droppedQuery = droppedQueryCount_.load(std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stats.pendingControl = controlQueue_.size();
        stats.pendingRecover = recoverQueue_.size();
        stats.pendingConfig = configQueue_.size();
        stats.pendingQuery = queryQueue_.size();
        stats.pendingTotal = stats.pendingControl + stats.pendingRecover +
                             stats.pendingConfig + stats.pendingQuery;
        stats.workerRunning = started_ && !stopRequested_;
    }
    return stats;
}

void DeviceRuntime::workerLoop()
{
    for (;;) {
        Request request;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() {
                return stopRequested_ || !allQueuesEmptyLocked();
            });

            if (stopRequested_ && allQueuesEmptyLocked()) {
                sending_ = false;
                idleCv_.notify_all();
                break;
            }

            if (!popNextLocked(&request)) {
                continue;
            }
            sending_ = true;
        }

        if (transport_) {
            transport_->send(request.frame);
        }
        sentCount_.fetch_add(1, std::memory_order_relaxed);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            sending_ = false;
        }
        idleCv_.notify_all();
    }
}

bool DeviceRuntime::allQueuesEmptyLocked() const
{
    return controlQueue_.empty() && recoverQueue_.empty() &&
           configQueue_.empty() && queryQueue_.empty();
}

bool DeviceRuntime::popNextLocked(Request *request)
{
    if (!controlQueue_.empty()) {
        *request = controlQueue_.front();
        controlQueue_.pop_front();
        return true;
    }
    if (!recoverQueue_.empty()) {
        *request = recoverQueue_.front();
        recoverQueue_.pop_front();
        return true;
    }
    if (!configQueue_.empty()) {
        *request = configQueue_.front();
        configQueue_.pop_front();
        return true;
    }
    if (!queryQueue_.empty()) {
        *request = queryQueue_.front();
        queryQueue_.pop_front();
        return true;
    }
    return false;
}

DeviceRuntime::Queue &DeviceRuntime::queueFor(Category category)
{
    switch (category) {
    case Category::Control:
        return controlQueue_;
    case Category::Recover:
        return recoverQueue_;
    case Category::Config:
        return configQueue_;
    case Category::Query:
        return queryQueue_;
    }
    return controlQueue_;
}

const DeviceRuntime::Queue &DeviceRuntime::queueFor(Category category) const
{
    switch (category) {
    case Category::Control:
        return controlQueue_;
    case Category::Recover:
        return recoverQueue_;
    case Category::Config:
        return configQueue_;
    case Category::Query:
        return queryQueue_;
    }
    return controlQueue_;
}

std::size_t DeviceRuntime::maxDepthFor(Category category) const
{
    switch (category) {
    case Category::Control:
        return options_.maxControlQueueDepth;
    case Category::Recover:
        return options_.maxRecoverQueueDepth;
    case Category::Config:
        return options_.maxConfigQueueDepth;
    case Category::Query:
        return options_.maxQueryQueueDepth;
    }
    return options_.maxControlQueueDepth;
}

const char *DeviceRuntime::categoryName(Category category) const
{
    switch (category) {
    case Category::Control:
        return "control";
    case Category::Recover:
        return "recover";
    case Category::Config:
        return "config";
    case Category::Query:
        return "query";
    }
    return "unknown";
}
