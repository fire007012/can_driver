#ifndef CANPROTOCOL_H
#define CANPROTOCOL_H

#include <cstdint>
#include <vector>

#include "MotorID.h"
/**
 * @brief 协议解析的抽象基类 (接口)
 */
class CanProtocol {
public:
    /**
     * @brief 电机运行模式
     *
     * Position 代表位置环（如点到点控制），Velocity 代表速度环（连续转速）。
     * 若具体协议支持更多模式，可以在派生类中转译后再调用 setMode。
     */
    enum class MotorMode : uint8_t {
        Position = 0,
        Velocity = 1,
    };

    virtual ~CanProtocol() = default;

    /**
     * @brief 切换指定电机的工作模式
     * @param motorId 电机或节点在总线中的标识，通常对应并行编号
     * @param mode 目标工作模式
     *
     * 实现类应负责发送对应的指令并缓存当前模式，以便调用方读取。
     */
    virtual bool setMode(MotorID motorId, MotorMode mode) = 0;

    /**
     * @brief 设置目标速度
     * @param motorId 目标电机 ID
     * @param velocity 协议规定的速度值（单位由协议决定）
     *
     * 上层只需给出逻辑速度值，具体缩放和单位转换由派生类处理。
     */
    virtual bool setVelocity(MotorID motorId, int32_t velocity) = 0;

    /**
     * @brief 设置目标加速度
     * @param motorId 目标电机 ID
     * @param acceleration 协议规定的加速度值
     *
     * 某些协议仅缓存该值而不实际下发命令，接口仍然定义以保持一致性。
     */
    virtual bool setAcceleration(MotorID motorId, int32_t acceleration) = 0;

    /**
     * @brief 设置目标减速度
     * @param motorId 目标电机 ID
     * @param deceleration 协议规定的减速度值
     */
    virtual bool setDeceleration(MotorID motorId, int32_t deceleration) = 0;

    /**
     * @brief 设置目标位置
     * @param motorId 目标电机 ID
     * @param position 位置值（编码器计数或角度，由协议决定）
     *
     * 通常用于位置模式，速度模式可以选择忽略或复用该接口实现为位置跟随。
     */
    virtual bool setPosition(MotorID motorId, int32_t position) = 0;

    /**
     * @brief 使能指定电机
     * @param motorId 目标电机 ID
     *
     * 多数协议需要写入控制字或状态字，本接口不做具体约定。
     */
    virtual bool Enable(MotorID motorId) = 0;

    /**
     * @brief 失能指定电机
     * @param motorId 目标电机 ID
     *
     * 常用于关闭驱动器输出或进入待机。
     */
    virtual bool Disable(MotorID motorId) = 0;

    /**
     * @brief 紧急停止
     * @param motorId 目标电机 ID
     *
     * 若协议支持广播停机，可忽略 motorId 并一次性停止全部节点。
     */
    virtual bool Stop(MotorID motorId) = 0;

    /**
     * @brief 读取或返回缓存的实际位置
     * @param motorId 目标电机 ID
     * @return 位置值（协议单位）
     *
     * 没有实时数据时可主动触发一次读取，并在读取完成后更新缓存。
     */
    [[nodiscard]] virtual int64_t getPosition(MotorID motorId) const = 0;

    /**
     * @brief 读取或返回缓存的实际电流
     * @param motorId 目标电机 ID
     * @return 电流数值（协议单位）
     */
    [[nodiscard]] virtual int16_t getCurrent(MotorID motorId) const = 0;

    /**
     * @brief 读取或返回缓存的实际速度
     * @param motorId 目标电机 ID
     * @return 速度数值（协议单位）
     */
    [[nodiscard]] virtual int16_t getVelocity(MotorID motorId) const = 0;

    /**
     * @brief 返回缓存的使能状态（未实现协议默认 false）。
     */
    [[nodiscard]] virtual bool isEnabled(MotorID motorId) const
    {
        (void)motorId;
        return false;
    }

    /**
     * @brief 返回缓存的故障状态（未实现协议默认 false）。
     */
    [[nodiscard]] virtual bool hasFault(MotorID motorId) const
    {
        (void)motorId;
        return false;
    }

    /**
     * @brief 设置设备侧位置限位（物理限位）
     * @param motorId 目标电机 ID
     * @param minPositionRaw 下限（协议原始单位）
     * @param maxPositionRaw 上限（协议原始单位）
     * @param enable 是否使能设备侧限位
     * @return 协议支持并发送成功返回 true，否则 false
     */
    virtual bool configurePositionLimits(MotorID motorId,
                                         int32_t minPositionRaw,
                                         int32_t maxPositionRaw,
                                         bool enable)
    {
        (void)motorId;
        (void)minPositionRaw;
        (void)maxPositionRaw;
        (void)enable;
        return false;
    }

    /**
     * @brief 设置位置零点偏置（协议原始单位）
     * @param motorId 目标电机 ID
     * @param offsetRaw 偏置值
     * @return 协议支持并发送成功返回 true，否则 false
     */
    virtual bool setPositionOffset(MotorID motorId, int32_t offsetRaw)
    {
        (void)motorId;
        (void)offsetRaw;
        return false;
    }

    /**
     * @brief 初始化电机信息刷新机制
     * @param motorIds 需要周期刷新状态的电机 ID 列表
     *
     * 实现类应当使用 boost::asio 或其它定时机制，以约 1ms 的周期轮询
     * 底层硬件并更新缓存。若列表为空则停止轮询。
     */
    virtual void initializeMotorRefresh(const std::vector<MotorID> &motorIds) = 0;
};

#endif // CANPROTOCOL_H
