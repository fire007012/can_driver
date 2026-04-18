#ifndef MOTORID_H
#define MOTORID_H

#include <cstdint>

/**
 * @brief 设备/电机的逻辑 ID。
 *
 * 值使用完整的 16 位 CAN ID，方便与现有宏定义保持一致；
 * 通过辅助函数可以取得低 8 位索引用于调用 CanProtocol 接口。
 */
enum class MotorID : std::uint16_t {
    LeftWheel = 0x141,
    RightWheel = 0x142,
    
    RotaryTable = 0x6,
    LargerArm = 0x5,
    SmallerArm = 0x13,
    WristX = 0x14,
    WristY = 0x15,
    WristZ = 0x16,
    Actuator = 0x17,

    PTZ1 = 0x21,
    PTZ2 = 0x22,
};

// enum class EyouMotorID : std::uint16_t {

//     Arm3 = 0x13,
//     Arm4 = 0x14,
//     Arm5 = 0x15,
//     Arm6 = 0x16,
//     TailEnd = 0x17,

// };

// enum class MTMotorID : std::uint16_t {

//     MainWheel1 = 0x141,
//     MainWheel2 = 0x142,

//     PTZ1 = 0x21,
//     PTZ2 = 0x22,

// };
// /**
//  * @brief 返回 电机ID 对应的完整 CAN ID。
//  */
// constexpr std::uint16_t toCanId(EyouMotorID id)
// {
//     return static_cast<std::uint16_t>(id);
// }
// constexpr std::uint16_t toCanId(MTMotorID id)
// {
//     return static_cast<std::uint16_t>(id);
// }

/**
 * @brief 返回 电机ID 对应的完整 CAN ID。
 */
constexpr std::uint16_t toCanId(MotorID id)
{
    return static_cast<std::uint16_t>(id);
}

#endif // MOTORID_H
