#ifndef CAN_TYPE_H
#define CAN_TYPE_H

/**
 * @brief CAN 协议类型标识。
 *
 * MT 对应雷赛 MT 系列协议，PP 对应 Eyou/PP 协议。
 * 该枚举用于在同一总线上按电机关联到不同协议栈实例。
 */
enum class CanType {
    MT, ///< 雷赛 MT 协议
    PP, ///< Eyou PP 协议
    ECB ///< Innfos ECB 协议
};

#endif // PROTOCOLTYPE_H
