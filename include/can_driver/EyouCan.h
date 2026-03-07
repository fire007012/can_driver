#ifndef EyouCan_H
#define EyouCan_H
#include "CanProtocol.h"
#include "can_driver/CanTransport.h"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class EyouCan : public CanProtocol {

public:

    /**
     * @brief 构造函数
     * @param controller 基于 socketcan_interface 的 CAN 传输实现（标准 8 字节帧）
     */
    explicit EyouCan(std::shared_ptr<CanTransport> controller);

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

    /**
     * @brief 读取/缓存电机位置（如无缓存则触发 0x07 读取）
     */
    int64_t getPosition(MotorID motorId) const override;

    /**
     * @brief 返回缓存的实际电流（无缓存时触发 0x05 读取）
     */
    int16_t getCurrent(MotorID motorId) const override;

    /**
     * @brief 返回缓存的实际速度（无缓存时触发 0x06 读取）
     */
    int16_t getVelocity(MotorID motorId) const override;
    bool isEnabled(MotorID motorId) const override;
    bool hasFault(MotorID motorId) const override;
    void initializeMotorRefresh(const std::vector<MotorID> &motorIds) override;
    /// 设置状态轮询频率（Hz）；<=0 表示使用默认自适应周期。
    void setRefreshRateHz(double hz);

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

    std::shared_ptr<CanTransport> canController;
    mutable std::unordered_map<uint8_t, MotorState> motorStates;
    mutable std::mutex stateMutex;
    std::size_t receiveHandlerId = 0;
    std::vector<uint8_t> refreshMotorIds;
    mutable std::unordered_set<uint8_t> managedMotorIds;
    mutable std::mutex refreshMutex;
    std::atomic<bool> refreshLoopActive {false};
    std::thread refreshThread;
    std::atomic<double> refreshRateHz_{0.0};

    /**
     * @brief 发送写指令帧（0x01）
     */
    void sendWriteCommand(uint8_t motorId, uint8_t subCommand, uint32_t value, std::size_t payloadBytes);
    /**
     * @brief 发送读指令帧（0x03）
     */
    void sendReadCommand(uint8_t motorId, uint8_t subCommand) const;
    /**
     * @brief 处理来自底层传输的回复帧（0x02/0x04）
     */
    void handleResponse(const CanTransport::Frame &data);
    void requestPosition(uint8_t motorId) const;
    void requestMode(uint8_t motorId) const;
    void requestEnable(uint8_t motorId) const;
    void requestCurrent(uint8_t motorId) const;
    void requestVelocity(uint8_t motorId) const;
    bool isManagedMotorId(uint8_t motorId) const;
    void registerManagedMotorId(uint8_t motorId) const;
    void refreshMotorStates();
    std::chrono::milliseconds computeRefreshSleep(std::size_t motorCount) const;
    void stopRefreshLoop();
};

#endif // EyouCan_H
