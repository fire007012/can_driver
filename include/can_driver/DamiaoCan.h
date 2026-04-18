#ifndef CAN_DRIVER_DAMIAO_CAN_H
#define CAN_DRIVER_DAMIAO_CAN_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanTransport.h"
#include "can_driver/CanTxDispatcher.h"
#include "can_driver/RefreshScheduler.h"
#include "can_driver/SharedDriverState.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class DamiaoCan : public CanProtocol {
    friend class DamiaoCanTestAccessor;

public:
    DamiaoCan(std::shared_ptr<CanTransport> controller,
              std::shared_ptr<CanTxDispatcher> txDispatcher);
    DamiaoCan(std::shared_ptr<CanTransport> controller,
              std::shared_ptr<CanTxDispatcher> txDispatcher,
              std::shared_ptr<can_driver::SharedDriverState> sharedState,
              std::string deviceName);
    ~DamiaoCan() override;

    bool setMode(MotorID motorId, MotorMode mode) override;
    bool setVelocity(MotorID motorId, int32_t velocity) override;
    bool setAcceleration(MotorID motorId, int32_t acceleration) override;
    bool setDeceleration(MotorID motorId, int32_t deceleration) override;
    bool setPosition(MotorID motorId, int32_t position) override;
    bool quickSetPosition(MotorID motorId, int32_t position) override;
    bool Enable(MotorID motorId) override;
    bool Disable(MotorID motorId) override;
    bool Stop(MotorID motorId) override;
    bool ResetFault(MotorID motorId) override;

    int64_t getPosition(MotorID motorId) const override;
    int16_t getCurrent(MotorID motorId) const override;
    int32_t getVelocity(MotorID motorId) const override;
    bool isEnabled(MotorID motorId) const override;
    bool hasFault(MotorID motorId) const override;

    void initializeMotorRefresh(const std::vector<MotorID> &motorIds) override;
    void setRefreshRateHz(double hz);
    std::chrono::milliseconds refreshSleepInterval() const;

    using RefreshQuery = can_driver::DmRefreshQuery;
    bool issueRefreshQuery(MotorID motorId, RefreshQuery query);

private:
    struct MotorState {
        int64_t position{0};
        int32_t velocity{0};
        int16_t current{0};
        int32_t commandedVelocity{0};
        int32_t acceleration{0};
        int32_t deceleration{0};
        std::uint8_t feedbackState{0};
        bool enabled{false};
        bool fault{false};
        bool positionReceived{false};
        bool velocityReceived{false};
        bool currentReceived{false};
        bool modeReceived{false};
        bool enabledReceived{false};
        bool faultReceived{false};
        bool feedbackSeen{false};
        bool velocityModeConfigured{false};
        bool feedbackRangeConfigured{false};
        MotorMode mode{MotorMode::Velocity};
    };

    struct PendingRegisterAck {
        bool received{false};
        std::uint32_t value{0};
    };

    void handleResponse(const CanTransport::Frame &frame);
    bool sendSpeedFrame(std::uint8_t motorId,
                        float velocityRadPerSec,
                        CanTxDispatcher::Category category,
                        const char *source) const;
    bool sendMotorCommand(std::uint8_t motorId,
                          std::uint8_t command,
                          CanTxDispatcher::Category category,
                          const char *source) const;
    bool sendRegisterWrite(std::uint8_t motorId,
                           std::uint8_t registerId,
                           std::uint32_t value,
                           const char *source) const;
    bool writeRegisterAndWaitForAck(std::uint8_t motorId,
                                    std::uint8_t registerId,
                                    std::uint32_t value,
                                    std::chrono::milliseconds timeout);
    bool waitForFeedbackState(std::uint8_t motorId,
                              std::uint8_t expectedState,
                              std::chrono::milliseconds timeout) const;
    bool ensureVelocityModeConfigured(std::uint8_t motorId);
    bool ensureFeedbackMappingConfigured(std::uint8_t motorId);
    bool submitTx(const CanTransport::Frame &frame,
                  CanTxDispatcher::Category category,
                  const char *source) const;
    bool isManagedMotorId(std::uint8_t motorId) const;
    void registerManagedMotorId(MotorID motorId);
    MotorID resolveSystemMotorId(std::uint8_t motorId) const;
    can_driver::SharedDriverState::AxisKey makeAxisKey(std::uint8_t motorId) const;
    void syncSharedFeedback(std::uint8_t motorId, const MotorState &state) const;
    void syncSharedCommand(std::uint8_t motorId,
                           int32_t targetVelocity,
                           bool valid) const;
    void syncSharedModeSelection(std::uint8_t motorId, MotorMode desiredMode) const;
    void syncSharedIntent(std::uint8_t motorId, can_driver::AxisIntent intent) const;
    static std::uint16_t pendingRegisterAckKey(std::uint8_t motorId, std::uint8_t registerId);
    static float uintToFloat(std::uint32_t raw, float minValue, float maxValue, int bits);
    static std::uint32_t floatToRawBits(float value);
    static float rawToVelocityRadPerSec(int32_t rawVelocity);
    static int64_t rawPositionFromRadians(float positionRad);
    static int32_t rawVelocityFromRadiansPerSec(float velocityRadPerSec);
    static int16_t rawTorqueFromNewtonMeters(float torqueNm);
    std::chrono::milliseconds computeRefreshSleep(std::size_t motorCount) const;

    std::shared_ptr<CanTransport> canController_;
    std::shared_ptr<CanTxDispatcher> txDispatcher_;
    std::shared_ptr<can_driver::SharedDriverState> sharedState_;
    std::string deviceName_;
    mutable std::unordered_map<std::uint8_t, MotorState> motorStates_;
    mutable std::mutex stateMutex_;
    std::size_t receiveHandlerId_{0};
    std::vector<std::uint8_t> refreshMotorIds_;
    std::unordered_set<std::uint8_t> managedMotorIds_;
    std::unordered_map<std::uint8_t, MotorID> systemMotorIdsByNodeId_;
    mutable std::mutex refreshMutex_;
    std::atomic<double> refreshRateHz_{0.0};
    std::atomic<bool> shuttingDown_{false};
    mutable std::mutex pendingRegisterAckMutex_;
    std::condition_variable pendingRegisterAckCv_;
    std::unordered_map<std::uint16_t, PendingRegisterAck> pendingRegisterAcks_;
    mutable std::condition_variable feedbackStateCv_;
};

#endif // CAN_DRIVER_DAMIAO_CAN_H
