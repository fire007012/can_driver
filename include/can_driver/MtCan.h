#ifndef MTCAN_H
#define MTCAN_H
#include "CanProtocol.h"
#include "can_driver/CanTransport.h"
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

class MtCan : public CanProtocol {

public:
    /**
     * @brief 构造函数
        * @param controller 基于 CanTransport 的 CAN 传输实现
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
     * @brief 设置加速度（映射到 0x43 速度规划加速度）
     */
    bool setAcceleration(MotorID motorId, int32_t acceleration) override;

    /**
     * @brief 设置减速度（映射到 0x43 速度规划减速度）
     */
    bool setDeceleration(MotorID motorId, int32_t deceleration) override;

    /**
     * @brief 设置通讯中断保护时间（0xB3）
     */
    bool setCommunicationTimeout(uint32_t timeoutMs);

    /**
     * @brief 设置速度规划加速度（0x43, index=0x02）
     */
    bool setSpeedAcceleration(MotorID id, uint32_t accelDpsPerSec);

    /**
     * @brief 设置速度规划减速度（0x43, index=0x03）
     */
    bool setSpeedDeceleration(MotorID id, uint32_t decelDpsPerSec);

    /**
     * @brief 设置位置规划加速度（0x43, index=0x00）
     */
    bool setPositionAcceleration(MotorID id, uint32_t accelDpsPerSec);

    /**
     * @brief 设置位置规划减速度（0x43, index=0x01）
     */
    bool setPositionDeceleration(MotorID id, uint32_t decelDpsPerSec);

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
     * @brief 返回电机速度（0x9C 返回的值，单位 1 dps/LSB）
     */
    int16_t getVelocity(MotorID motorId) const override;
    bool isEnabled(MotorID motorId) const override;
    bool hasFault(MotorID motorId) const override;

    /**
     * @brief 配置需轮询的电机并启动 1ms 刷新任务
     */
    void initializeMotorRefresh(const std::vector<MotorID> &motorIds) override;

    /// 设置状态轮询频率（Hz）；<=0 表示使用默认自适应周期。
    void setRefreshRateHz(double hz);

private:
    struct MotorState {
        int32_t position = 0;
        int64_t multiTurnAngle = 0;  ///< 0x92 多圈角度，单位 0.01°
        int16_t velocity = 0;        ///< 速度，单位 1 dps/LSB（协议原始单位）
        double current = 0.0;
        int32_t commandedVelocity = 0;
        int8_t temperature = 0;      ///< 温度（°C）
        uint16_t voltageRaw1 = 0;    ///< 0x9A DATA[2..3]（电压/保留，协议相关）
        uint16_t voltageRaw2 = 0;    ///< 0x9A DATA[4..5]（电压/保留，协议相关）
        uint16_t encoderPosition = 0; ///< 单圈编码器位置
        bool enabled = false;
        bool error = false;
        bool mitModePrimed = false;
        uint8_t runMode = 0; ///< 0x01电流环, 0x02速度环, 0x03位置环
        MotorMode mode = MotorMode::Velocity;
    };

    struct MitCommand {
        double positionRad = 0.0; // p_des
        double velocityRadPerSec = 0.0; // v_des
        double kp = 10.0;
        double kd = 1.0;
        double torqueNm = 0.0; // t_ff
    };

    std::shared_ptr<CanTransport> canController;
    mutable std::unordered_map<uint8_t, MotorState> motorStates;
    mutable std::mutex stateMutex;
    std::size_t receiveHandlerId = 0;
    std::vector<uint8_t> refreshMotorIds;
    mutable std::mutex refreshMutex;
    std::atomic<bool> refreshLoopActive {false};
    std::thread refreshThread;
    std::atomic<double> refreshRateHz_{0.0};
    bool mtPositionUseMit_ = true;
    bool mtDebugMit_ = true;
    // 现场默认值：10/1 常偏软，重载下位置模式不明显。
    // 提升到更实用的默认刚度，并给一个小前馈力矩克服静摩擦，仍可通过环境变量覆盖。
    double mitDefaultKp_ = 80.0;
    double mitDefaultKd_ = 2.5;
    double mitDefaultTorqueNm_ = 0.8;

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
     * @brief 触发读取系统运行模式（0x70）
     */
    void requestRunMode(uint8_t motorId) const;
    /**
     * @brief 触发读多圈角度命令（0x92）
     */
    void requestMultiTurnAngle(uint8_t motorId) const;
    /**
     * @brief 发送 A4 位置保持命令，强制驱动切入位置环（runmode=0x03）
     */
    void sendA4PositionHoldRaw(uint8_t motorId, int32_t positionRaw, uint16_t maxSpeedDps) const;
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
    bool sendMitPositionCommand(uint16_t motorIdRaw, uint8_t nodeId, int32_t positionRaw);
    void sendMitFrame(uint16_t mitCanId, const MitCommand &cmd) const;
    void loadMitConfigFromEnv();
    /**
     * @brief 通用加减速写入（0x43）
     */
    bool writeAcceleration(uint8_t motorId, uint8_t index, uint32_t value);
    /**
     * @brief 广播通讯超时保护给所有已注册电机
     */
    void broadcastCommunicationTimeout(uint32_t timeoutMs);
    std::chrono::milliseconds computeRefreshSleep(std::size_t motorCount) const;
    /**
     * @brief 解析 CAN 返回帧，更新缓存
     */
    void handleResponse(const CanTransport::Frame &data);
};

#endif // MTCAN_H
