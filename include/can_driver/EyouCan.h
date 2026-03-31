#ifndef EyouCan_H
#define EyouCan_H
#include "CanProtocol.h"
#include "can_driver/CanTransport.h"
#include "can_driver/SharedDriverState.h"
#include "can_driver/CanTxDispatcher.h"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstdint>

class EyouCan : public CanProtocol {
    friend class EyouCanTestAccessor;

public:

    /**
     * @brief 构造函数
        * @param controller 基于 CanTransport 的 CAN 传输实现（标准 8 字节帧）
     */
    explicit EyouCan(std::shared_ptr<CanTransport> controller);
    EyouCan(std::shared_ptr<CanTransport> controller,
            std::shared_ptr<CanTxDispatcher> txDispatcher);
    EyouCan(std::shared_ptr<CanTransport> controller,
            std::shared_ptr<CanTxDispatcher> txDispatcher,
            std::shared_ptr<can_driver::SharedDriverState> sharedState,
            std::string deviceName);

    ~EyouCan();

    /**
     * @brief 设置电机工作模式（位置/速度）
     * 对应原 PP 协议 0x0F 子命令
     */
    bool setMode(MotorID motorId, MotorMode mode) override;

    /**
     * @brief 设置目标速度
     * 对应 PP 协议 0x09 子命令
     */
    bool setVelocity(MotorID motorId, int32_t velocity) override;

    /**
     * @brief 设置加速度（目前仅缓存，协议未提供）
     */
    bool setAcceleration(MotorID motorId, int32_t acceleration) override;

    /**
     * @brief 设置减速度（目前仅缓存，协议未提供）
     */
    bool setDeceleration(MotorID motorId, int32_t deceleration) override;

    /**
     * @brief 设置目标位置
     * 对应 PP 协议 0x0A 子命令
     */
    bool setPosition(MotorID motorId, int32_t position) override;

    /**
     * @brief 快写位置命令（用于 CSP 模式）
     * 对应 CMD=0x05 快写命令
     */
    bool quickSetPosition(MotorID motorId, int32_t position) override;

    /**
     * @brief 下发使能命令（0x10 子命令）
     */
    bool Enable(MotorID motorId) override;

    /**
     * @brief 下发失能命令（0x10，值为 0）
     */
    bool Disable(MotorID motorId) override;

    /**
     * @brief 紧急停止（0x11）
     */
    bool Stop(MotorID motorId) override;
    bool ResetFault(MotorID motorId) override;

    /**
     * @brief 返回缓存的电机位置
     */
    int64_t getPosition(MotorID motorId) const override;

    /**
     * @brief 返回缓存的实际电流
     */
    int16_t getCurrent(MotorID motorId) const override;

    /**
     * @brief 返回缓存的实际速度
     */
    int16_t getVelocity(MotorID motorId) const override;
    bool isEnabled(MotorID motorId) const override;
    bool hasFault(MotorID motorId) const override;
    bool configurePositionLimits(MotorID motorId,
                                 int32_t minPositionRaw,
                                 int32_t maxPositionRaw,
                                 bool enable) override;
    bool setPositionOffset(MotorID motorId, int32_t offsetRaw) override;
    void initializeMotorRefresh(const std::vector<MotorID> &motorIds) override;
    /// 设置状态轮询频率（Hz）；<=0 表示使用默认自适应周期。
    void setRefreshRateHz(double hz);
    /// 执行一轮状态查询，由外部刷新 worker 调度。
    void runRefreshCycle();
    /// 返回当前注册电机的建议查询周期。
    std::chrono::milliseconds refreshSleepInterval() const;
    /// 设置是否启用 PP 快写命令（CMD=0x05）。
    void setFastWriteEnabled(bool enabled);
    uint64_t fastWriteSentCount() const;
    uint64_t normalWriteSentCount() const;

private:
    /**
     * @brief 内部状态缓存，记录最新目标/实际数据
     */
    struct MotorState {
        int32_t position = 0;
        int32_t commandedPosition = 0;
        int32_t commandedVelocity = 0;
        int32_t actualVelocity = 0;
        int32_t current = 0;
        int32_t acceleration = 0;
        int32_t deceleration = 0;
        bool enabled = false;
        bool fault = false;
        bool positionReceived = false;
        bool velocityReceived = false;
        bool currentReceived = false;
        MotorMode mode = MotorMode::Position;
    };
    struct PendingReadRequest {
        std::chrono::steady_clock::time_point lastSent {};
        std::chrono::steady_clock::time_point nextEligibleSend {};
        bool inFlight {false};
        std::size_t consecutiveTimeouts {0};
    };

    std::shared_ptr<CanTransport> canController;
    std::shared_ptr<CanTxDispatcher> txDispatcher_;
    std::shared_ptr<can_driver::SharedDriverState> sharedState_;
    std::string deviceName_;
    mutable std::unordered_map<uint8_t, MotorState> motorStates;
    mutable std::mutex stateMutex;
    std::size_t receiveHandlerId = 0;
    std::vector<uint8_t> refreshMotorIds;
    mutable std::unordered_set<uint8_t> managedMotorIds;
    mutable std::mutex refreshMutex;
    std::atomic<double> refreshRateHz_{0.0};
    std::atomic<bool> fastWriteEnabled_{false};
    std::atomic<uint64_t> fastWriteSentCount_{0};
    std::atomic<uint64_t> normalWriteSentCount_{0};
    mutable std::mutex pendingReadMutex_;
    std::unordered_map<uint16_t, PendingReadRequest> pendingReadRequests_;

    /**
     * @brief 发送写指令帧（0x01）
     */
    void sendWriteCommand(uint8_t motorId,
                          uint8_t subCommand,
                          uint32_t value,
                          std::size_t payloadBytes,
                          uint8_t commandType = 0x01);
    /**
     * @brief 发送读指令帧（0x03）
     */
    void sendReadCommand(uint8_t motorId, uint8_t subCommand);
    /**
     * @brief 处理来自底层传输的回复帧（0x02/0x04）
     */
    void handleResponse(const CanTransport::Frame &data);
    void requestPosition(uint8_t motorId);
    void requestMode(uint8_t motorId);
    void requestEnable(uint8_t motorId);
    void requestFault(uint8_t motorId);
    void requestCurrent(uint8_t motorId);
    void requestVelocity(uint8_t motorId);
    bool isManagedMotorId(uint8_t motorId) const;
    void registerManagedMotorId(uint8_t motorId) const;
    void refreshMotorStates();
    std::chrono::milliseconds computeRefreshSleep(std::size_t motorCount) const;
    std::chrono::milliseconds computeReadRequestTimeout() const;
    void stopRefreshLoop();
    void publishWriteCountersParam() const;
    bool submitTx(const CanTransport::Frame &frame,
                  CanTxDispatcher::Category category,
                  const char *source) const;
    bool tryIssueReadCommand(uint8_t motorId, uint8_t subCommand);
    void markReadResponseReceived(uint8_t motorId, uint8_t subCommand);
    void resetReadTracking();
    can_driver::SharedDriverState::AxisKey makeAxisKey(uint8_t motorId) const;
    void syncSharedFeedback(uint8_t motorId, const MotorState &state) const;
    void syncSharedCommand(uint8_t motorId,
                           int64_t targetPosition,
                           int32_t targetVelocity,
                           MotorMode desiredMode,
                           bool valid) const;
    void syncSharedIntent(uint8_t motorId, can_driver::AxisIntent intent) const;
    void noteSharedTimeout(uint8_t motorId, std::size_t consecutiveTimeouts) const;
    static uint16_t pendingReadKey(uint8_t motorId, uint8_t subCommand);
    static std::chrono::milliseconds computeTimeoutBackoff(std::size_t consecutiveTimeouts,
                                                           std::chrono::milliseconds baseTimeout);

    /// refresh 轮询周期计数，用于 slow 项分频
    uint64_t refreshCycleCount_{0};
    /// slow 项分频系数（每 N 个 refresh 周期查一次 mode/enable/fault/current）
    static constexpr uint64_t kSlowDivider = 50;
};

#endif // EyouCan_H
