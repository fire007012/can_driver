#ifndef MTCAN_H
#define MTCAN_H
#include "CanProtocol.h"
#include "can_driver/CanTransport.h"
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

class MtCan : public CanProtocol {

public:
    /**
     * @brief 构造函数
     * @param controller 基于 socketcan_interface 的 CAN 传输实现
     */
    explicit MtCan(std::shared_ptr<CanTransport> controller);

    ~MtCan();

    /**
     * @brief 设置工作模式（目前仅记录状态，不发送额外命令）
     */
    bool setMode(MotorID motorId, MotorMode mode) override;

    /**
     * @brief 设置目标速度（命令 0xA2）
     */
    bool setVelocity(MotorID motorId, int32_t velocity) override;

    /**
     * @brief 设置加速度（协议未提供，暂忽略）
     */
    bool setAcceleration(MotorID motorId, int32_t acceleration) override;

    /**
     * @brief 设置减速度（协议未提供，暂忽略）
     */
    bool setDeceleration(MotorID motorId, int32_t deceleration) override;

    /**
     * @brief 位置控制命令（0xA4），同时携带最大速度
     */
    bool setPosition(MotorID motorId, int32_t position) override;

    /**
     * @brief 标记电机进入可控状态（协议无独立使能命令）
     */
    bool Enable(MotorID motorId) override;

    /**
     * @brief 关闭电机输出（0x80, Motor Off）
     */
    bool Disable(MotorID motorId) override;

    /**
     * @brief 停止（0x81）
     */
    bool Stop(MotorID motorId) override;

    /**
     * @brief 返回缓存的电机位置
     */
    int64_t getPosition(MotorID motorId) const override;

    /**
     * @brief 返回电流（0x9C 返回的值，单位 0.01A）
     */
    int16_t getCurrent(MotorID motorId) const override;

    /**
     * @brief 返回电机速度（0x9C 返回的值，内部除以 6）
     */
    int16_t getVelocity(MotorID motorId) const override;
    bool isEnabled(MotorID motorId) const override;
    bool hasFault(MotorID motorId) const override;

    /**
     * @brief 配置需轮询的电机并启动 1ms 刷新任务
     */
    void initializeMotorRefresh(const std::vector<MotorID> &motorIds) override;

private:
    struct MotorState {
        int32_t position = 0;
        int64_t multiTurnAngle = 0;  ///< 0x92 多圈角度，单位 0.01°
        int16_t velocity = 0;
        double current = 0.0;
        int32_t commandedVelocity = 0;
        int8_t temperature = 0;      ///< 温度（°C）
        uint16_t encoderPosition = 0; ///< 单圈编码器位置
        bool enabled = false;
        bool error = false;
        MotorMode mode = MotorMode::Velocity;
    };

    std::shared_ptr<CanTransport> canController;
    mutable std::unordered_map<uint8_t, MotorState> motorStates;
    mutable std::mutex stateMutex;
    std::size_t receiveHandlerId = 0;
    std::vector<uint8_t> refreshMotorIds;
    mutable std::mutex refreshMutex;
    std::atomic<bool> refreshLoopActive {false};
    std::thread refreshThread;

    /**
     * @brief 将节点 ID 组合成 CAN ID（高位取 canBaseId，高 8 位 + motorId）
     */
    uint16_t encodeSendCanId(uint8_t motorId) const;
    /**
     * @brief 发送标准 8 字节 CAN 帧
     */
    void sendFrame(uint16_t canId, uint8_t command, const std::array<uint8_t, 4> &payload) const;
    /**
     * @brief 触发读状态命令（0x9C）
     */
    void requestState(uint8_t motorId) const;
    /**
     * @brief 触发读错误命令（0x9A）
     */
    void requestError(uint8_t motorId) const;
    /**
     * @brief 触发读多圈角度命令（0x92）
     */
    void requestMultiTurnAngle(uint8_t motorId) const;
    /**
     * @brief 复位系统（0x76）
     */
    void resetSystem(uint8_t motorId) const;
    /**
     * @brief 设置零点（0x64）
     */
    void setZeroPosition(uint8_t motorId) const;
    void refreshMotorStates();
    void stopRefreshLoop();
    /**
     * @brief 解析 CAN 返回帧，更新缓存
     */
    void handleResponse(const CanTransport::Frame &data);
};

#endif // MTCAN_H
