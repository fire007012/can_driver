#ifndef CAN_DRIVER_INNFOS_ECB_PROTOCOL_H
#define CAN_DRIVER_INNFOS_ECB_PROTOCOL_H

#include "can_driver/CanProtocol.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

class ActuatorController;

class InnfosEcbProtocol : public CanProtocol {
public:
    explicit InnfosEcbProtocol(std::string deviceSpec);
    ~InnfosEcbProtocol() override;

    bool setMode(MotorID motorId, MotorMode mode) override;
    bool setVelocity(MotorID motorId, int32_t velocity) override;
    bool setAcceleration(MotorID motorId, int32_t acceleration) override;
    bool setDeceleration(MotorID motorId, int32_t deceleration) override;
    bool setPosition(MotorID motorId, int32_t position) override;
    bool quickSetPosition(MotorID motorId, int32_t position) override;
    bool Enable(MotorID motorId) override;
    bool Disable(MotorID motorId) override;
    bool Stop(MotorID motorId) override;

    [[nodiscard]] int64_t getPosition(MotorID motorId) const override;
    [[nodiscard]] int16_t getCurrent(MotorID motorId) const override;
    [[nodiscard]] int32_t getVelocity(MotorID motorId) const override;
    [[nodiscard]] bool isEnabled(MotorID motorId) const override;
    [[nodiscard]] bool hasFault(MotorID motorId) const override;

    bool configurePositionLimits(MotorID motorId,
                                 int32_t minPositionRaw,
                                 int32_t maxPositionRaw,
                                 bool enable) override;
    bool setPositionOffset(MotorID motorId, int32_t offsetRaw) override;

    void initializeMotorRefresh(const std::vector<MotorID> &motorIds) override;

    void configureMotorRouting(MotorID motorId,
                               const std::string &ipAddress,
                               bool autoDiscovery);
    void setRefreshRateHz(double hz);

private:
    struct MotorRoute {
        bool autoDiscovery{false};
        std::string fixedIp;
    };

    struct MotorEndpoint {
        uint8_t actuatorId{0};
        std::string ipAddress;
        bool resolved{false};
        bool enabled{false};
    };

    struct MotorCache {
        int64_t positionRaw{0};
        int16_t velocityRaw{0};
        int16_t currentRaw{0};
        bool enabled{false};
        bool fault{false};
        bool valid{false};
    };

    bool ensureControllerLocked() const;
    bool ensureEndpointReadyLocked(uint16_t motorId) const;
    bool reconnectEndpointLocked(uint16_t motorId) const;
    bool resolveEndpointLocked(uint16_t motorId) const;
    bool updateCacheLocked(uint16_t motorId) const;
    bool canAttemptRecoveryLocked(uint16_t motorId) const;
    void refreshLoop();
    void stopRefreshThread();

    static uint16_t toRawMotorId(MotorID motorId);

    mutable std::mutex mutex_;
    mutable ActuatorController *controller_{nullptr};

    std::string deviceSpec_;
    std::string defaultIp_;
    bool defaultAutoDiscovery_{false};

    mutable std::map<uint16_t, MotorRoute> routes_;
    mutable std::map<uint16_t, MotorEndpoint> endpoints_;
    mutable std::map<uint16_t, MotorCache> caches_;
    mutable std::map<uint16_t, std::chrono::steady_clock::time_point> nextRecoveryTry_;
    std::map<uint16_t, MotorMode> modeCache_;
    std::set<uint16_t> refreshIds_;

    int refreshIntervalMs_{20};
    std::atomic<bool> refreshRunning_{false};
    std::thread refreshThread_;
};

#endif // CAN_DRIVER_INNFOS_ECB_PROTOCOL_H
